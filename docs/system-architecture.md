# JVS-AGV 阿克曼导航系统架构文档

> ROS2 Jazzy | Ackermann 底盘 | Nav2 导航 | openTCS 车队调度 | 三硬件配置

---

## 一、系统概述

JVS-AGV 是一套基于 ROS2 Jazzy 的阿克曼（Ackermann）转向 AGV 导航系统，支持**三种硬件配置**一键切换：

| 配置 | 底盘 | 传感器 | 适用场景 |
|------|------|--------|---------|
| **gazebo** | Gazebo 仿真 (ros2_control) | 仿真 (ros_gz_bridge) | 开发调试、算法验证 |
| **rs485** | RS-485 协议 MCU (BX-S40) | RPLIDAR S2L + 编码器 + IMU | 实车部署（RS-485 底盘） |
| **raspberry** | STM32 UART (YeahBot) | RPLIDAR C1 + 编码器 + IMU | 树莓派小车部署 |

### 1.1 核心能力

- **SLAM 建图**：slam_toolbox 在线异步建图
- **自主导航**：Nav2 + SmacPlannerHybrid (Reeds-Shepp) + RegulatedPurePursuitController
- **自主探索**：frontier_explorer 前端探索 + explore_lite
- **车队调度**：openTCS-NeNa 集成，车辆状态上报、任务下发、路线图管理
- **物料操作**：上下货动作服务 (jvs_agv_material_actions)
- **系统监控**：node_watchdog 节点/话题健康检测

### 1.2 技术栈

| 组件 | 版本/说明 |
|------|----------|
| ROS2 | Jazzy Jalisco |
| DDS | Fast-DDS (rmw_fastrtps_cpp) |
| 仿真器 | Gazebo Harmonic |
| 导航栈 | Nav2 (SmacPlannerHybrid + RPP) |
| SLAM | slam_toolbox (third-party fork) |
| 探索 | m-explore-ros2 |
| 传感器融合 | robot_localization (EKF) |
| 调度系统 | openTCS-NeNa ( RELEASE2.0.1) |
| UI 框架 | PyQt5 |

---

## 二、系统分层架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        应用层 (Application)                         │
│                                                                     │
│  opentcs_vehicle_node  ─ openTCS 车队调度集成                       │
│  frontier_explorer     ─ 自主探索建图                               │
│  route_graph_loader    ─ GeoJSON 路线图管理                         │
│  material_action_gui   ─ 上下货动作 GUI (PyQt5)                    │
│  node_watchdog         ─ 系统健康监控                               │
│  battery_bridge        ─ 电池状态桥接 (仅 raspberry)                │
│  lifecycle_starter     ─ 生命周期管理 (替代 Nav2 lifecycle_manager)  │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     导航层 (Navigation)                              │
│                                                                     │
│  Nav2 完整导航栈 (navigation_custom.launch.py):                     │
│  ├ planner_server    SmacPlannerHybrid (Reeds-Shepp, min_r=1.0m)   │
│  ├ controller_server RegulatedPurePursuitController                │
│  ├ behavior_server   BackUp + Wait (无 Spin，阿克曼不可原地旋转)    │
│  ├ velocity_smoother max_vel=0.5m/s                                │
│  ├ collision_monitor 实时碰撞检测                                   │
│  ├ bt_navigator      ackermann_nav.xml 行为树                       │
│  ├ smoother_server   路径平滑                                       │
│  └ route_server      GeoJSON 路线图服务                             │
│                                                                     │
│  行为树策略 (ackermann_nav.xml):                                     │
│  ├ ComputePathToPose → FollowPath (各 2 次重试)                    │
│  └ 恢复: BackUp(3.5m) → ClearCostmap → Wait(2s) → BackUp(2m)     │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   定位/建图层 (Localization/SLAM)                    │
│                                                                     │
│  建图模式: slam_toolbox (online_async) → /map + TF map→odom        │
│  定位模式: map_server + AMCL → TF map→odom + /amcl_pose            │
│                                                                     │
│  AMCL 配置: likelihood_field, max_beams=200, 粒子 500~3000         │
│  SLAM 配置: resolution=0.05, max_range=15.9m                       │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    传感器融合层 (Sensor Fusion)                      │
│                                                                     │
│  EKF (robot_localization/ekf_filter_node)                          │
│  ├ 输入: /odom (里程计) + /imu 或 /imu/data_raw (IMU)              │
│  ├ 输出: TF odom → base_frame                                      │
│  ├ 频率: 30Hz, two_d_mode: true                                    │
│  └ 配置: config/ekf.yaml (frame 由 profile 覆盖)                   │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  硬件抽象层 (Hardware Abstraction)                   │
│                  ★ hardware_profile 决定 ★                          │
│                                                                     │
│  ┌─ gazebo ──────────┐  ┌─ rs485 ─────────────┐  ┌─ raspberry ───┐│
│  │ gz_sim            │  │ rplidar_node (S2L)  │  │ rplidar_node  ││
│  │ ros_gz_bridge     │  │ car_base_node       │  │  (C1)         ││
│  │ socat (虚拟串口)  │  │  (里程计+IMU)       │  │ car_base_node ││
│  │ rs485_receiver    │  │ cmd_vel_bridge      │  │  (底盘+传感器) ││
│  │ vehicle_controller│  │ rs485_chassis_bridge│  │ static_tf     ││
│  │ rs485_bridge      │  │ static_tf           │  │ battery_bridge││
│  └───────────────────┘  └─────────────────────┘  └───────────────┘│
│                                                                     │
│  统一输出话题:                                                       │
│    /scan (LaserScan), /odom (Odometry), /imu (Imu)                 │
│    TF: odom → base_frame, base_frame → lidar_frame                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 三、硬件配置详解

### 3.1 配置文件结构

```
config/profiles/
├── gazebo.yaml       # Gazebo 全仿真
├── rs485.yaml        # RS-485 物理底盘
└── raspberry.yaml    # 树莓派小车 (YeahBot)
```

每个 Profile 文件定义：

```yaml
hardware_profile: "raspberry"       # 配置名称
use_sim_time: false                 # 是否使用仿真时钟

chassis:                            # 底盘驱动
  type: "uart_stm32"                # 底盘类型: gazebo / rs485 / uart_stm32
  serial_port: "/dev/ttyAMA0"       # 串口设备
  baudrate: 115200                  # 波特率

sensors:                            # 传感器配置
  lidar: { type, model, frame, serial_port, baudrate }
  imu:   { type, topic }
  odom:  { type, topic }

frames:                             # 坐标系名称
  base_frame: "base_link"           # 底盘坐标系
  odom_frame: "odom"                # 里程计坐标系
  lidar_frame: "lidar_link"         # 激光雷达坐标系

vehicle:                            # 车辆参数
  wheel_base: 0.175                 # 轴距 (m)
  max_steering_angle: 0.785         # 最大转向角 (rad, 约45°)
  max_velocity: 0.3                 # 最大速度 (m/s)
  lidar_height: 0.15                # 雷达安装高度 (m)
  footprint: "[...]"                # 车身轮廓 (m)
```

### 3.2 三种配置参数对比

| 参数 | gazebo | rs485 | raspberry |
|------|--------|-------|-----------|
| **底盘类型** | Gazebo ros2_control | RS-485 协议 MCU | STM32 UART (car_base_node) |
| **底盘串口** | /tmp/chassis_cmd (虚拟) | /dev/ttyUSB1 | /dev/ttyAMA0 |
| **Ackermann 解算** | vehicle_controller (ROS侧) | cmd_vel_bridge (ROS侧) | **STM32 内部** |
| **激光雷达** | Gazebo 仿真 | RPLIDAR S2L (/dev/ttyUSB0) | RPLIDAR C1 (/dev/lidar) |
| **IMU 来源** | Gazebo 仿真 → /imu | car_base_node → /imu/data_raw | car_base_node → /imu/data_raw |
| **里程计来源** | Gazebo 仿真 → /odom | car_base_node → /odom | car_base_node → /odom |
| **base_frame** | body_link | base_link | base_link |
| **雷达 frame** | ackermann_robot/body_link/lidar | laser | lidar_link |
| **use_sim_time** | true | false | false |
| **轴距** | 0.58m | 0.58m | 0.175m |
| **最大转向角** | 30° (0.5236 rad) | 30° (0.5236 rad) | 45° (0.785 rad) |
| **最大速度** | 1.4 m/s | 1.4 m/s | 0.3 m/s |
| **车身尺寸** | 0.9×0.6m | 0.9×0.6m | 0.24×0.2m |

---

## 四、节点清单与功能说明

### 4.1 自定义节点 (lidar_slam_nodes 包)

| 节点 | 类型 | 功能 | Profile |
|------|------|------|---------|
| `cmd_vel_bridge` | LifecycleNode | /cmd_vel (Twist) → /steering_angle + /velocity (Float64)，含 Ackermann 运动学分解 | gazebo, rs485 |
| `rs485_chassis_bridge` | LifecycleNode | 编码 RS-485 帧 (23B)，写入串口，200ms 周期，500ms 超时归零 | gazebo, rs485 |
| `rs485_chassis_receiver` | LifecycleNode | 解码 RS-485 帧，发布 /rs485/steering_angle + /rs485/velocity | gazebo only |
| `frontier_explorer` | LifecycleNode | 前端探索决策，多目标评分（距离+航向+尺寸），黑名单机制，卡住检测 | all |
| `opentcs_vehicle_node` | LifecycleNode | openTCS Sidecar 桥接，10Hz 位姿上报，航向对齐状态机，障碍物检测 | all |
| `route_graph_loader` | LifecycleNode | 加载 GeoJSON 路线图到 route_server | all |
| `lifecycle_starter` | Node | 替代 Nav2 lifecycle_manager，支持超时+重试+两轮启动 | all |
| `node_watchdog` | Node | 监控关键节点/话题健康，发布 /system_health (DiagnosticArray) | all |
| `battery_bridge` | Node | /PowerVoltage (Float32) → /battery_state (BatteryState)，电压→百分比 | raspberry |
| `load_controllers` | Node | 激活 ros2_control 关节控制器 (forward_position/velocity) | gazebo |
| `ackermann_keyboard_teleop` | Node | 键盘遥控，WASD 控制，q/e 调速 | all |
| `wait_for_topic` | Node | 启动门控：等待话题有发布者 | all |
| `wait_for_tf` | Node | 启动门控：等待 TF 变换可用 | all |
| `wait_for_service` | Node | 启动门控：等待 Service 可用 | all |
| `scan_range_filter` | Node | 过滤 Gazebo /scan NaN/Inf，修复 frame_id 前缀 | gazebo (部分) |

### 4.2 C++ 节点 (ackermann_control 包)

| 节点 | 功能 | Profile |
|------|------|---------|
| `vehicle_controller` | Ackermann 几何解算：前轮双角+后轮差速+换向过渡(0.5s)+800ms超时归零 | gazebo |

### 4.3 第三方节点

| 节点/包 | 功能 |
|---------|------|
| `ekf_filter_node` (robot_localization) | 扩展卡尔曼滤波，融合里程计+IMU |
| `slam_toolbox` | 在线异步 SLAM 建图 |
| `nav2_amcl` | 自适应蒙特卡洛定位 |
| `nav2_*` (10个) | Nav2 导航栈核心 |
| `rplidar_node` (rplidar_ros) | RPLIDAR 激光雷达驱动 |
| `car_base_node` (car_base) | STM32 UART 通信，里程计+IMU+关节状态 (ros2_ws) |
| `gz_sim` + `ros_gz_bridge` | Gazebo 仿真器+话题桥接 |

---

## 五、话题与数据流

### 5.1 话题清单

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/scan` | sensor_msgs/LaserScan | rplidar 或 Gazebo | AMCL, Nav2 costmaps, collision_monitor |
| `/odom` | nav_msgs/Odometry | car_base_node 或 Gazebo | EKF |
| `/imu` 或 `/imu/data_raw` | sensor_msgs/Imu | car_base_node 或 Gazebo | EKF |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 controller | car_base_node 或 cmd_vel_bridge |
| `/steering_angle` | std_msgs/Float64 | cmd_vel_bridge | rs485_chassis_bridge, vehicle_controller |
| `/velocity` | std_msgs/Float64 | cmd_vel_bridge | rs485_chassis_bridge, vehicle_controller |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL | opentcs_vehicle_node |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox 或 map_server | AMCL, Nav2, frontier_explorer |
| `/tf` | tf2_msgs/TFMessage | EKF, AMCL, robot_state_publisher | 所有需要坐标变换的节点 |
| `/battery_state` | sensor_msgs/BatteryState | battery_bridge | opentcs_vehicle_node |
| `/system_health` | diagnostic_msgs/DiagnosticArray | node_watchdog | rqt_robot_monitor |
| `/PowerVoltage` | std_msgs/Float32 | car_base_node (STM32) | battery_bridge |
| `/joint_states` | sensor_msgs/JointState | car_base_node 或 Gazebo | robot_state_publisher |

### 5.2 TF 变换树

```
gazebo profile:                      rs485 / raspberry profile:

  map                                  map
   │ (AMCL)                             │ (AMCL)
   ▼                                    ▼
  odom                                 odom
   │ (EKF)                              │ (EKF)
   ▼                                    ▼
  body_link                            base_link
   │ (static TF)                        │ (static TF)
   ▼                                    ▼
  ackermann_robot/body_link/lidar      laser 或 lidar_link
```

### 5.3 控制数据流 (下行: 目标 → 电机)

**gazebo:**
```
Nav2 → /cmd_vel → cmd_vel_bridge → /steering_angle + /velocity
  → rs485_bridge → socat虚拟串口 → rs485_receiver → /rs485/*
  → vehicle_controller → ros2_control → Gazebo 仿真
```

**rs485:**
```
Nav2 → /cmd_vel → cmd_vel_bridge → /steering_angle + /velocity
  → rs485_bridge → /dev/ttyUSB1 → MCU (RS-485 协议) → 电机
```

**raspberry:**
```
Nav2 → /cmd_vel → car_base_node → /dev/ttyAMA0 → STM32 → 电机
(Ackermann 解算在 STM32 内部完成)
```

### 5.4 传感器数据流 (上行: 传感器 → 定位)

**gazebo:** Gazebo → ros_gz_bridge → /scan + /odom + /imu → EKF → AMCL

**rs485 / raspberry:**
```
RPLIDAR → rplidar_node → /scan ─────────────────→ AMCL, Nav2 costmaps
STM32 → car_base_node → /odom (编码器) ──→ EKF → AMCL
                      → /imu/data_raw (IMU) ─→ EKF
                      → /PowerVoltage → battery_bridge
```

---

## 六、启动流程

### 6.1 统一入口: nav_main.launch.py

```
事件驱动启动链 (严格依赖顺序):

[立即启动]
  ├ 硬件层子 launch (根据 hardware_profile 选择)
  │   ├ gazebo: socat, gz_sim, ros_gz_bridge, robot_state_publisher, ...
  │   ├ rs485: rplidar, car_base_node, cmd_vel_bridge, rs485_bridge (3s延迟)
  │   └ raspberry: rplidar_c1, car_base_node, battery_bridge
  ├ rviz2
  └ node_watchdog
       │
       ▼ wait_for_topic(/scan, timeout=120s)
[传感器融合]
  └ ekf_filter_node
       │
       ▼ wait_for_tf(odom→base, timeout=60s)
[定位]
  ├ map_server (加载地图)
  ├ amcl (粒子滤波定位)
  └ lifecycle_starter_localization
       │
       ▼ wait_for_topic(/lifecycle_starter_localization/ready, timeout=120s)
[导航 + 应用]
  ├ navigation_custom.launch.py (Nav2 全栈)
  ├ lifecycle_starter_custom (管理自定义 LifecycleNode)
  ├ cmd_vel_bridge (gazebo/rs485 only)
  └ opentcs_vehicle_node
       │
       ▼ wait_for_service(/route_server/set_route_graph, timeout=60s)
[路线 + 物料]
  ├ route_graph_loader
  └ material_action_gui (5s 延迟)
```

### 6.2 启动命令

```bash
# Gazebo 仿真
ros2 launch nav_main.launch.py hardware_profile:=gazebo

# RS-485 物理底盘
bash scripts/launch/rs485_nav.sh [map_file.yaml]

# 树莓派小车 (在 Pi 上执行)
bash /home/pi/jvs/scripts/launch/rpi_opentcs.sh [map_file.yaml]

# 旧版兼容 (仍可用)
bash scripts/launch/sim_ackermann_rs485.sh
```

### 6.3 场景化启动脚本矩阵 (硬件 × 业务场景)

每个硬件配置 (Gazebo / 树莓派) 都有 5 个标准化启动脚本,覆盖从建图到调度的完整链路:

| 场景 | Gazebo | 树莓派 |
|------|--------|--------|
| **SLAM 建图** | `sim_slam.sh` | `rpi_slam.sh` |
| **自动探索建图** | `sim_explore.sh` | `rpi_explore.sh` |
| **openTCS 导航** | `sim_opentcs_nav.sh [map]` | `rpi_opentcs_nav.sh [map]` |
| **openTCS Kernel** | `sim_opentcs_kernal.sh` | `rpi_opentcs_kernal.sh` |
| **openTCS PlantOverview** | `sim_opentcs_overview.sh` | `rpi_opentcs_overview.sh` |

**推荐使用流程**:

1. 建图阶段: 跑 `*_slam.sh` (手动遥控) 或 `*_explore.sh` (自动探索)
2. 验证建图: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`
3. 导航阶段: 跑 `*_opentcs_nav.sh /path/to/map.yaml`
4. 调度阶段: 另开终端跑 `*_opentcs_kernal.sh` + `*_opentcs_overview.sh`

**RS-485 实车** 当前仅有 `rs485_nav.sh` (等同于 `*_opentcs_nav.sh`),
Gazebo/树莓派的 `_slam`/`_explore` 脚本可作为 RS-485 的模板参考。

---

## 七、通信协议

### 7.1 RS-485 协议 (gazebo / rs485 profile)

```
帧格式 (23 字节固定):
[0xAA][0x55] | [CH1_H][CH1_L] ... [CH10_H][CH10_L] | [0xA5]
  帧头 2B         10 通道 × 2B = 20B (大端序)          帧尾 1B

通道值域: 1000-2000, 中值 1500 = 停止/中位
CH1: 转向角 (1000=左满舵, 1500=中位, 2000=右满舵)
CH2: 速度   (1000=最大倒车, 1500=停止, 2000=最大前进)
CH3-CH10: 保留 (填充 1500)

波特率: 115200, 写入周期: 200ms, 超时: 500ms 归零
```

### 7.2 UART 协议 (raspberry profile - car_base_node ↔ STM32)

```
发送帧 (ROS → STM32, 11 字节):
[0xAA][0x55][0x0B][类型][Vx_H][Vx_L][Vy_H][Vy_L][Vz_H][Vz_L][校验]
类型: 0x50=速度控制, 0x66=蜂鸣器, 0x80=机械臂
数据: short 类型, 放大 1000 倍

接收帧 (STM32 → ROS, 36 字节):
[0xAA]...[0x7D]
含: IMU加速度(6B) + 陀螺仪(6B) + 编码器速度(6B) + 关节角度(12B) + 电池电压(1B)

串口: /dev/ttyAMA0, 波特率 115200
IMU: STM32 内部 Mahony 互补滤波, 50Hz
里程计: 编码器 + IMU 融合, 含经验修正系数
```

---

## 八、坐标系定义

| Frame | 发布者 | 说明 |
|-------|--------|------|
| `map` | — | 全局固定坐标系 (世界) |
| `odom` | EKF | 里程计坐标系 (平滑漂移) |
| `body_link` / `base_link` | EKF | 机器人底盘中心 |
| `ackermann_robot/body_link/lidar` / `laser` / `lidar_link` | static TF | 激光雷达安装位置 |
| `imu_link` | static TF / URDF | IMU 传感器 |
| `front_*_steering_link` | robot_state_publisher | 前轮转向关节 |
| `*_wheel_link` | robot_state_publisher | 车轮 |

---

## 九、机器人参数

### BX-S40 (gazebo / rs485)

| 参数 | 值 |
|------|-----|
| 车身尺寸 | 0.9 × 0.6 × 0.4 m |
| 轮径 | 0.16 m |
| 轴距 | 0.58 m |
| 轮距 | 0.68 m |
| 最大转向角 | 30° (0.5236 rad) |
| 最大速度 | 1.4 m/s |
| 车身轮廓 | [±0.45, ±0.30] m |
| 导航最大速度 | 0.5 m/s |
| 最小转弯半径 | 1.0 m |

### YeahBot (raspberry)

| 参数 | 值 |
|------|-----|
| 轴距 | 0.175 m |
| 最大转向角 | 45° (0.785 rad) |
| 最大速度 | 0.3 m/s |
| 车身轮廓 | [±0.12, ±0.10] m |
| 激光雷达 | RPLIDAR C1, /dev/lidar, 460800 baud |
| 底盘串口 | /dev/ttyAMA0, 115200 baud |
| 电池 | 24V 系统, 20V~24V 范围 |

---

## 十、关键设计决策

### 10.1 为什么用 lifecycle_starter 替代 Nav2 lifecycle_manager?

Nav2 原生 lifecycle_manager 在 Fast-DDS (rmw_fastrtps_cpp) 下偶尔出现 service call 超时，
导致节点无法激活。自定义 `lifecycle_starter` 提供超时+重试机制，确保可靠启动。

### 10.2 为什么 Ackermann 不能原地旋转?

Ackermann 转向类似汽车，需要前进才能转弯。因此：
- 行为树移除了 `Spin` 恢复动作，替换为 `Wait`
- cmd_vel_bridge 在低速+有角速度时强制给予 0.05m/s 蠕行速度
- 规划器使用 SmacPlannerHybrid (Reeds-Shepp)，支持倒车

### 10.3 为什么树莓派不需要 cmd_vel_bridge 和 rs485_bridge?

树莓派小车的 STM32 内部完成 Ackermann 运动学解算。ROS 侧只需发送原始 `/cmd_vel` (Twist)，
STM32 负责将线速度和角速度转换为左右轮差速和前轮转角。这简化了 ROS 侧的控制链。

### 10.4 硬件 Profile 机制

`hardware_profile` 参数在 launch 时传入，通过 `OpaqueFunction` 在运行时加载对应的
`config/profiles/{profile}.yaml` 配置文件，决定：
- 启动哪个硬件子 launch
- EKF 的 frame 名称和 sensor 话题
- lifecycle_starter_custom 管理的节点列表
- 是否启动 cmd_vel_bridge / rs485_bridge

---

## 十一、文件结构

```
lidar-slam/
├── launch/
│   ├── nav_main.launch.py              ★ 统一导航入口
│   ├── hardware/
│   │   ├── gazebo_hardware.launch.py   ★ Gazebo 硬件层
│   │   ├── rs485_hardware.launch.py    ★ RS-485 硬件层
│   │   └── raspberry_hardware.launch.py★ 树莓派硬件层
│   ├── navigation_custom.launch.py     # Nav2 子 launch
│   ├── localization_custom.launch.py   # AMCL 子 launch
│   ├── real_slam.launch.py             # 实车 SLAM (独立)
│   ├── rplidar_s2l.launch.py           # RPLIDAR 驱动
│   ├── sim_ackermann_rs485.launch.py   # 旧版统一入口 (兼容)
│   └── sim_ackermann_*.launch.py       # 旧版仿真 launch
├── config/
│   ├── profiles/
│   │   ├── gazebo.yaml                 ★ Gazebo 硬件参数
│   │   ├── rs485.yaml                  ★ RS-485 硬件参数
│   │   └── raspberry.yaml              ★ 树莓派硬件参数
│   ├── ekf.yaml                        # EKF 传感器融合
│   ├── nav2_params_opentcs.yaml        # Nav2 参数 (openTCS 模式)
│   ├── nav2_params_exploration.yaml    # Nav2 参数 (探索模式)
│   ├── slam_toolbox_*.yaml             # SLAM 参数 (多种)
│   ├── rs485_bridge.yaml               # RS-485 串口配置
│   ├── opentcs_vehicle.yaml            # openTCS 车辆配置
│   ├── watchdog.yaml                   # 系统监控配置
│   └── material_action.yaml            # 物料操作配置
├── scripts/
│   ├── launch/
│   │   ├── ────────── Gazebo 场景 ──────────
│   │   ├── sim_slam.sh              ★ Gazebo SLAM 建图
│   │   ├── sim_explore.sh           ★ Gazebo 自动探索建图
│   │   ├── sim_opentcs_nav.sh       ★ Gazebo openTCS 导航
│   │   ├── sim_opentcs_kernal.sh    ★ openTCS Kernel (Gazebo)
│   │   ├── sim_opentcs_overview.sh  ★ openTCS PlantOverview (Gazebo)
│   │   ├── ────────── 树莓派场景 ──────────
│   │   ├── rpi_slam.sh           ★ 树莓派 SLAM 建图
│   │   ├── rpi_explore.sh        ★ 树莓派自动探索建图
│   │   ├── rpi_opentcs_nav.sh    ★ 树莓派 openTCS 导航
│   │   ├── rpi_opentcs_kernal.sh ★ openTCS Kernel (树莓派)
│   │   ├── rpi_opentcs_overview.sh★ openTCS PlantOverview (树莓派)
│   │   ├── ────────── RS-485 / 旧版 ──────────
│   │   ├── rs485_nav.sh                ★ RS-485 启动脚本
│   │   ├── rpi_opentcs.sh        ★ 树莓派旧版 (兼容)
│   │   ├── sim_ackermann_rs485.sh      # 仿真启动 (当前主场景)
│   │   └── ...其他启动脚本
│   ├── tools/
│   │   ├── cleanup_ros2.sh             # 清理残留进程
│   │   └── diagnose_ackermann.sh       # 诊断工具
│   └── deploy_remote.py                # 远程部署工具
├── src/
│   ├── lidar_slam_nodes/               # 自定义 Python 节点包
│   │   └── lidar_slam_nodes/
│   │       ├── cmd_vel_bridge.py
│   │       ├── rs485_chassis_bridge.py
│   │       ├── rs485_chassis_receiver.py
│   │       ├── rs485_protocol.py
│   │       ├── frontier_explorer.py
│   │       ├── opentcs_vehicle_node.py
│   │       ├── lifecycle_starter.py
│   │       ├── node_watchdog.py
│   │       ├── route_graph_loader.py
│   │       ├── battery_bridge.py
│   │       ├── ackermann_keyboard_teleop.py
│   │       ├── load_controllers.py
│   │       ├── scan_range_filter.py
│   │       ├── wait_for_topic.py
│   │       ├── wait_for_tf.py
│   │       └── wait_for_service.py
│   ├── ackermann_control/              # C++ 车辆控制器 (ros2_control)
│   ├── jvs_agv_material_actions/       # 物料操作 GUI (PyQt5)
│   ├── jvs_agv_material_msgs/          # 物料操作消息接口
│   ├── explore_lite → third-party/     # 自主探索 (符号链接)
│   └── explore_lite_msgs → third-party/
├── third-party/                        # Git 子模块
│   ├── slam_toolbox/                   # SLAM 工具箱 (fork)
│   ├── m-explore-ros2/                 # 自主探索
│   ├── gazebo_ackermann_steering_vehicle/ # Gazebo 车辆模型
│   ├── rplidar_sdk/                    # RPLIDAR SDK
│   ├── openTCS-NeNa/                   # openTCS 车队调度
│   └── aws-robomaker-small-warehouse-world/ # Gazebo 仿真世界
├── behavior_trees/
│   └── ackermann_nav.xml               # Ackermann 定制行为树
├── models/
│   └── ackermann/
│       └── ackermann.xacro             # 机器人 URDF 模型
├── maps/                               # 地图文件 (.pgm + .yaml)
├── worlds/
│   └── factory.sdf                     # Gazebo 仿真世界
└── docs/                               # 项目文档
```

---

## 十二、部署指南

### 12.1 仿真环境 (开发机)

```bash
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 编译
colcon build --symlink-install

# 启动仿真导航
ros2 launch nav_main.launch.py hardware_profile:=gazebo
```

### 12.2 树莓派小车 (10.0.0.205)

```bash
# 首次部署: 从开发机上传代码
conda run -n lidar_slam python3 scripts/deploy_remote.py check  # 检查连接

# 在树莓派上编译
cd /home/pi/jvs
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 3

# 启动导航
bash scripts/launch/rpi_opentcs.sh [地图文件]

# 环境变量已在 ~/.bashrc 中配置:
#   LIDAR_SLAM_ROOT=/home/pi/jvs
#   ROS_DOMAIN_ID=42
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   source /home/pi/jvs/install/setup.bash
```

### 12.3 RS-485 实车

```bash
# 确认串口设备存在
ls /dev/ttyUSB0 /dev/ttyUSB1

# 启动导航
bash scripts/launch/rs485_nav.sh [地图文件]
```

---

## 十三、监控与诊断

### 13.1 系统健康监控 (node_watchdog)

每 5 秒检查一次，关键指标：

| 类别 | 监控对象 | 告警级别 |
|------|---------|---------|
| 关键节点 | ekf, amcl, controller_server, bt_navigator, cmd_vel_bridge, opentcs_vehicle | ERROR |
| 关键话题 | /scan, /odom, /tf, /cmd_vel, /amcl_pose | ERROR |
| 非关键节点 | rviz2, material_action_gui | WARN |

### 13.2 诊断命令

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查话题频率
ros2 topic hz /scan
ros2 topic hz /odom

# 查看节点状态
ros2 lifecycle list /cmd_vel_bridge

# 诊断 Ackermann 系统
bash scripts/tools/diagnose_ackermann.sh
```
