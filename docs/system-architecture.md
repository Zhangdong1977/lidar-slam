# JVS-AGV 阿克曼导航系统架构文档

> ROS2 Jazzy | Ackermann 底盘 | Nav2 导航 | openTCS 车队调度 | 三硬件配置 | 多车调度

---

## 一、系统概述

JVS-AGV 是一套基于 ROS2 Jazzy 的阿克曼（Ackermann）转向 AGV 导航系统，采用**三层解耦架构**，支持**四种硬件配置**一键切换，支持**多车调度**场景。

| 配置 | 底盘 | 传感器 | DDS Domain | 适用场景 |
|------|------|--------|------------|---------|
| **gazebo** | Gazebo 仿真 (ros2_control) | 仿真 (ros_gz_bridge) | 42 | 开发调试、算法验证 |
| **rs485** | RS-485 协议 MCU (BX-S40) | RPLIDAR S2L + 编码器 + IMU | 30 | 实车部署（RS-485 底盘） |
| **raspberry** | STM32 UART (YeahBot) | RPLIDAR C1 + 编码器 + IMU | 30 | 树莓派小车部署 |
| **rplidar_s2l** | 无底盘 | RPLIDAR S2L | 30 | 手持建图（笔记本+雷达） |

### 1.1 核心能力

- **SLAM 建图**：slam_toolbox 在线异步建图
- **自主导航**：Nav2 + SmacPlannerHybrid (Dubin) + RegulatedPurePursuitController
- **自主探索**：frontier_explorer 前端探索 + explore_lite
- **车队调度**：openTCS-NeNa 集成，车辆状态上报、任务下发、路线图管理
- **多车隔离**：DDS Domain ID 隔环境 + ROS2 Namespace 隔车辆
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

## 二、环境隔离与多车架构

### 2.1 DDS Domain ID 隔离

仿真环境 (domain 42) 和生产环境 (domain 30) 通过 DDS Domain ID 彻底隔离，互相无法发现话题。

```
开发端 (domain 42)              生产端 (domain 30)
┌─────────────────┐            ┌─────────────────────┐
│  Gazebo 仿真 × N │            │  Pi 小车 × N        │
│  /gazebo_1/scan  │            │  /c30_1/scan, etc.  │
│  namespace: gazebo_1 │        │  namespace: c30_1   │
└─────────────────┘            └─────────────────────┘
     互不可见 (DDS 层面隔离)
```

- Domain ID 由 profile YAML 的 `domain_id` 字段决定
- Shell 脚本自动从 profile 读取，支持 `--domain-id` CLI 覆盖
- Pi 端 `~/.bashrc` 应设置 `export ROS_DOMAIN_ID=30`

### 2.2 ROS2 Namespace 多车隔离

同一 domain 内，每台小车分配独立 namespace（如 `c30_1`, `c30_2`），通过 `GroupAction + PushRosNamespace` 实现话题/服务自动隔离。

**隔离原理：**
- 所有话题名使用**相对路径**（如 `scan` 而非 `/scan`）
- `PushRosNamespace('c30_1')` 将相对话题解析为 `/c30_1/scan`
- TF 话题通过 remapping `('/tf', 'tf')` 隔离到 `/c30_1/tf`
- Gazebo `robot_description` 按车辆隔离到 `/<namespace>/robot_description`
- Gazebo `/clock` 是全局仿真时钟，只由启动 Gazebo 的 bridge 发布；复用 Gazebo 的车辆使用 `--no-gazebo`
- `namespace=''`（默认）时，`PushRosNamespace('')` 为 no-op → 向后兼容单车

**第三方节点**（`rplidar_node`, `car_base_node`）通过 launch 中的**防御性 remappings** 确保话题被 namespace：

```python
car_base = Node(
    package='car_base', executable='car_base_node',
    remappings=[
        ('/odom', 'odom'),
        ('/imu/data_raw', 'imu/data_raw'),
        ('/cmd_vel', 'cmd_vel'),
        ('/tf', 'tf'), ('/tf_static', 'tf_static'),
    ],
)
```

**Sidecar 话题**（openTCS 通信）保持绝对路径，通过 `vehicle_name` 参数构造前缀：
- `/{vehicle_name}/amcl_pose`、`/{vehicle_name}/goal_pose` 等
- `/{vehicle_name}/route_graph_json` 用于下发 GeoJSON 路线图，避免与 Nav2 `route_graph` MarkerArray 撞名
- 多车时将 `vehicle_name` 设为 namespace 值（如 `c30_1`）

### 2.3 完整架构图

```
开发端 (domain 42)                   生产端 (domain 30)
┌──────────────────────┐     ┌───────────────────┬───────────────────┐
│  Gazebo 仿真          │     │  Pi C30_1         │  Pi C30_2         │
│  namespace: gazebo_1  │     │  namespace: c30_1 │  namespace: c30_2 │
│  /gazebo_1/scan       │     │  /c30_1/scan      │  /c30_2/scan      │
│  /gazebo_1/cmd_vel    │     │  /c30_1/cmd_vel   │  /c30_2/cmd_vel   │
│  /gazebo_1/tf         │     │  /c30_1/tf        │  /c30_2/tf        │
└──────────────────────┘     └───────────────────┴───────────────────┘
                             ┌───────────────────────────────────────┐
                             │  openTCS Sidecar (dev/server)         │
                             │  订阅: /c30_1/amcl_pose, /c30_2/...  │
                             │  发布: /c30_1/goal_pose, /c30_2/...  │
                             └───────────────────────────────────────┘
```

---

## 三、系统分层架构

系统采用**三层解耦架构**，应用层 × 中间层 × 硬件层正交组合，任何场景 × 硬件组合都能一行命令启动。

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         应用层 (Application Layer)                       │
│                         ★ 三个场景入口 ★                                 │
│                                                                         │
│  slam_main.launch.py ── 场景1: 手工建图                                  │
│    ├ ackermann_keyboard_teleop (可选键盘遥控)                             │
│    └ slam_toolbox online_async                                          │
│                                                                         │
│  explore_main.launch.py ─ 场景2: 自动探索建图                            │
│    ├ frontier_explorer (前端探索决策)                                     │
│    └ slam_toolbox + Nav2 (slam 提供定位，无需 AMCL)                      │
│                                                                         │
│  nav_main.launch.py ─── 场景3: 调度集成 (★ 多车 namespace)               │
│    ├ GroupAction(PushRosNamespace) ← namespace 隔离                     │
│    ├ opentcs_vehicle_node  ─ openTCS 车队调度集成                        │
│    ├ route_graph_loader    ─ GeoJSON 路线图管理                          │
│    └ material_action_gui   ─ 上下货动作 GUI (PyQt5)                     │
│                                                                         │
│  ─── 跨层服务 (所有场景共享) ───                                         │
│  node_watchdog ─ 系统健康监控                                            │
│  lifecycle_starter ─ 生命周期管理 (替代 Nav2 lifecycle_manager)           │
│  rviz2 ─ 可视化                                                          │
│  battery_bridge ─ 电池状态桥接 (仅 raspberry)                            │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       中间层 (Middleware Layer)                           │
│                       ★ 复杂子系统 ★                                     │
│                                                                         │
│  navigation.launch.py ── Nav2 完整导航栈 (10 节点):                      │
│  ├ PushRosNamespace(namespace) ← 话题自动隔离                           │
│  ├ planner_server    SmacPlannerHybrid (Dubin, min_r=1.0m)             │
│  ├ controller_server RegulatedPurePursuitController                    │
│  ├ behavior_server   BackUp + Wait (无 Spin，阿克曼不可原地旋转)         │
│  ├ velocity_smoother max_vel=0.5m/s                                    │
│  ├ collision_monitor 实时碰撞检测                                        │
│  ├ bt_navigator      ackermann_nav.xml 行为树                            │
│  ├ smoother_server   路径平滑                                            │
│  └ route_server      GeoJSON 路线图服务                                  │
│                                                                         │
│  localization.launch.py ─ AMCL 定位 (map_server + AMCL):                 │
│  ├ PushRosNamespace(namespace) ← 话题自动隔离                           │
│  └ map_server + AMCL → TF map→odom + amcl_pose                         │
│                                                                         │
│  传感器融合 (内联于应用层):                                               │
│  ├ EKF (robot_localization/ekf_filter_node) — gazebo/rs485/raspberry   │
│  └ rf2o (rf2o_laser_odometry) — rplidar_s2l (无底盘纯激光里程计)        │
│                                                                         │
│  SLAM (内联于应用层):                                                     │
│  └ slam_toolbox (online_async) → map + TF map→odom                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    硬件抽象层 (Hardware Abstraction Layer)                │
│                    ★ hardware_profile 参数决定 ★                         │
│                                                                         │
│  ┌─ gazebo ───────┐  ┌─ rs485 ─────────┐  ┌─ raspberry ──┐  ┌─ rplidar_s2l ┐│
│  │ gz_sim         │  │ rplidar (S2L)   │  │ rplidar (C1) │  │ rplidar (S2L)││
│  │ ros_gz_bridge  │  │ car_base_node   │  │ car_base_node│  │ static_tf    ││
│  │ socat (虚拟串口)│  │  (里程计+IMU)   │  │  (底盘+传感器)│  │ (无底盘)     ││
│  │ rs485_receiver │  │ cmd_vel_bridge  │  │ static_tf    │  └──────────────┘│
│  │ vehicle_ctrl   │  │ rs485_bridge    │  │ battery_bridge│                  │
│  │ rs485_bridge   │  │ static_tf       │  └──────────────┘                  │
│  └────────────────┘  └─────────────────┘                                    │
│                                                                         │
│  统一输出 (相对话题名, namespace 后自动解析):                              │
│  scan, odom, imu, cmd_vel, TF: odom→base, base→lidar                    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 四、硬件配置详解

### 4.1 配置文件结构

```
config/profiles/
├── gazebo.yaml       # Gazebo 全仿真 (domain_id: 42)
├── rs485.yaml        # RS-485 物理底盘 (domain_id: 30)
├── raspberry.yaml    # 树莓派小车 (domain_id: 30)
└── rplidar_s2l.yaml  # 纯激光雷达建图 (domain_id: 30)
```

每个 Profile 文件定义：

```yaml
hardware_profile: "raspberry"       # 配置名称
use_sim_time: false                 # 是否使用仿真时钟
domain_id: 30                       # DDS Domain ID (仿真=42, 生产=30)

sensing:                            # 传感器融合方式
  type: "ekf"                       # ekf: EKF融合 / rf2o: 激光里程计

chassis:                            # 底盘驱动
  type: "uart_stm32"                # 底盘类型: gazebo / rs485 / uart_stm32 / none
  serial_port: "/dev/ttyAMA0"       # 串口设备
  baudrate: 115200                  # 波特率

sensors:                            # 传感器配置 (话题名均为相对路径)
  lidar: { type, model, frame, serial_port, baudrate }
  imu:   { type, topic: "imu/data_raw" }
  odom:  { type, topic: "odom" }

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

### 4.2 四种配置参数对比

| 参数 | gazebo | rs485 | raspberry | rplidar_s2l |
|------|--------|-------|-----------|-------------|
| **domain_id** | 42 | 30 | 30 | 30 |
| **底盘类型** | Gazebo ros2_control | RS-485 协议 MCU | STM32 UART (car_base_node) | 无底盘 |
| **底盘串口** | /tmp/chassis_cmd (虚拟) | /dev/ttyUSB1 | /dev/ttyAMA0 | — |
| **Ackermann 解算** | vehicle_controller (ROS侧) | cmd_vel_bridge (ROS侧) | **STM32 内部** | — |
| **激光雷达** | Gazebo 仿真 | RPLIDAR S2L (/dev/ttyUSB0) | RPLIDAR C1 (/dev/lidar) | RPLIDAR S2L (/dev/ttyUSB0) |
| **IMU 来源** | Gazebo 仿真 → imu | car_base_node → imu/data_raw | car_base_node → imu/data_raw | 无 |
| **里程计来源** | Gazebo 仿真 → odom | car_base_node → odom | car_base_node → odom | 无 (rf2o 激光里程计) |
| **传感器融合** | EKF | EKF | EKF | rf2o |
| **base_frame** | body_link | base_link | base_link | base_link |
| **雷达 frame** | `<namespace>/body_link/lidar` | laser | lidar_link | laser |
| **use_sim_time** | true | false | false | false |
| **轴距** | 0.58m | 0.58m | 0.175m | — |
| **最大转向角** | 30° (0.5236 rad) | 30° (0.5236 rad) | 45° (0.785 rad) | — |
| **最大速度** | 1.4 m/s | 1.4 m/s | 0.3 m/s | — |

---

## 五、节点清单与功能说明

### 5.1 自定义节点 (lidar_slam_nodes 包)

| 节点 | 类型 | 功能 | Profile |
|------|------|------|---------|
| `cmd_vel_bridge` | LifecycleNode | cmd_vel (Twist) → steering_angle + velocity (Float64)，含 Ackermann 运动学分解 | gazebo, rs485 |
| `rs485_chassis_bridge` | LifecycleNode | 编码 RS-485 帧 (23B)，写入串口，200ms 周期，500ms 超时归零 | gazebo, rs485 |
| `rs485_chassis_receiver` | LifecycleNode | 解码 RS-485 帧，发布 rs485/steering_angle + rs485/velocity | gazebo only |
| `frontier_explorer` | LifecycleNode | 前端探索决策，多目标评分（距离+航向+尺寸），黑名单机制，卡住检测 | all |
| `opentcs_vehicle_node` | LifecycleNode | openTCS Sidecar 桥接，10Hz 位姿上报，航向对齐状态机，障碍物检测；所有话题已参数化，支持 namespace | all |
| `route_graph_loader` | LifecycleNode | 订阅 `route_graph_json`，加载 GeoJSON 路线图到 route_server；Marker 发布到 `route_graph/markers` | all |
| `lifecycle_starter` | Node | 替代 Nav2 lifecycle_manager，支持超时+重试+两轮启动 | all |
| `node_watchdog` | Node | 监控关键节点/话题健康，发布 system_health (DiagnosticArray)；话题名由 watchdog.yaml 配置 | all |
| `battery_bridge` | Node | PowerVoltage (Float32) → battery_state (BatteryState)，电压→百分比 | raspberry |
| `load_controllers` | Node | 激活 ros2_control 关节控制器 (forward_position/velocity) | gazebo |
| `ackermann_keyboard_teleop` | Node | 键盘遥控，WASD 控制，q/e 调速 | all |
| `wait_for_topic` | Node | 启动门控：等待话题有发布者 | all |
| `wait_for_tf` | Node | 启动门控：等待 TF 变换可用 | all |
| `wait_for_service` | Node | 启动门控：等待 Service 可用 | all |
| `scan_range_filter` | Node | 过滤 Gazebo scan NaN/Inf，修复 frame_id 前缀 | gazebo (部分) |

### 5.2 C++ 节点 (ackermann_control 包)

| 节点 | 功能 | Profile |
|------|------|---------|
| `vehicle_controller` | Ackermann 几何解算：前轮双角+后轮差速+换向过渡(0.5s)+800ms超时归零 | gazebo |

### 5.3 第三方节点

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

## 六、话题与数据流

### 6.1 话题清单

所有话题使用**相对名称**，namespace 后自动解析为 `/<namespace>/<topic>`。默认 `namespace=''` 时等价于绝对路径。

| 话题 (相对名) | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `scan` | sensor_msgs/LaserScan | rplidar 或 Gazebo | AMCL, Nav2 costmaps, collision_monitor |
| `odom` | nav_msgs/Odometry | car_base_node 或 Gazebo | EKF |
| `imu` 或 `imu/data_raw` | sensor_msgs/Imu | car_base_node 或 Gazebo | EKF |
| `cmd_vel` | geometry_msgs/Twist | Nav2 controller | car_base_node 或 cmd_vel_bridge |
| `steering_angle` | std_msgs/Float64 | cmd_vel_bridge | rs485_chassis_bridge, vehicle_controller |
| `velocity` | std_msgs/Float64 | cmd_vel_bridge | rs485_chassis_bridge, vehicle_controller |
| `amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL | opentcs_vehicle_node |
| `map` | nav_msgs/OccupancyGrid | slam_toolbox 或 map_server | AMCL, Nav2, frontier_explorer |
| `tf` | tf2_msgs/TFMessage | EKF, AMCL, robot_state_publisher | 所有需要坐标变换的节点 |
| `battery_state` | sensor_msgs/BatteryState | battery_bridge | opentcs_vehicle_node |
| `system_health` | diagnostic_msgs/DiagnosticArray | node_watchdog | rqt_robot_monitor |
| `PowerVoltage` | std_msgs/Float32 | car_base_node (STM32) | battery_bridge |
| `joint_states` | sensor_msgs/JointState | car_base_node 或 Gazebo | robot_state_publisher |
| `route_graph_json` | std_msgs/String (JSON) | 外部 (openTCS Sidecar) | route_graph_loader |
| `route_graph` | visualization_msgs/MarkerArray | Nav2 route_server | rviz2 |
| `route_graph/markers` | visualization_msgs/MarkerArray | route_graph_loader | rviz2 |
| `system_health_summary` | std_msgs/String | node_watchdog | 调试用 |

**Sidecar 话题**（绝对路径，运行在 namespace 外部）：

| 话题 (绝对名) | 类型 | 方向 |
|------|------|------|
| `/{vehicle_name}/amcl_pose` | PoseWithCovarianceStamped | 发布 (opentcs_vehicle → Sidecar) |
| `/{vehicle_name}/goal_pose` | PoseStamped | 订阅 (Sidecar → opentcs_vehicle) |
| `/{vehicle_name}/robot_state` | String | 发布 (opentcs_vehicle → Sidecar) |
| `/{vehicle_name}/battery_state` | BatteryState | 发布 (opentcs_vehicle → Sidecar) |
| `/{vehicle_name}/route_graph_json` | String (GeoJSON) | 订阅 (Sidecar → route_graph_loader) |

### 6.2 TF 变换树

TF 话题通过 remapping `('/tf', 'tf')` 隔离。namespace 后每台车有独立的 TF 树。

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
  <namespace>/body_link/lidar          laser 或 lidar_link
```

Frame ID 在各 namespace 的 TF 树内保持不变，通过 TF 话题隔离互不冲突。

### 6.3 控制数据流 (下行: 目标 → 电机)

**gazebo:**
```
Nav2 → cmd_vel → cmd_vel_bridge → steering_angle + velocity
  → rs485_bridge → socat虚拟串口 → rs485_receiver → rs485/*
  → vehicle_controller → ros2_control → Gazebo 仿真
```

**rs485:**
```
Nav2 → cmd_vel → cmd_vel_bridge → steering_angle + velocity
  → rs485_bridge → /dev/ttyUSB1 → MCU (RS-485 协议) → 电机
```

**raspberry:**
```
Nav2 → cmd_vel → car_base_node → /dev/ttyAMA0 → STM32 → 电机
(Ackermann 解算在 STM32 内部完成)
```

### 6.4 传感器数据流 (上行: 传感器 → 定位)

**gazebo:** Gazebo → ros_gz_bridge → scan + odom + imu → EKF → AMCL

**rs485 / raspberry:**
```
RPLIDAR → rplidar_node → scan ───────────────────→ AMCL, Nav2 costmaps
STM32 → car_base_node → odom (编码器) ──→ EKF → AMCL
                      → imu/data_raw (IMU) ─→ EKF
                      → PowerVoltage → battery_bridge
```

---

## 七、启动流程

### 7.1 三场景入口 (应用层)

所有场景共享同一套事件驱动启动链模式：`wait_for_*` 门控 → `OnProcessExit` 触发下一层，无 TimerAction 硬编码延迟。

场景3 (dispatch) 的所有节点被 `GroupAction(PushRosNamespace)` 包裹，支持 namespace 多车隔离。

#### 场景1: 手工建图 (slam_main.launch.py)

```
[立即启动]
  ├ 硬件层子 launch (根据 hardware_profile 选择)
  │   ├ gazebo: socat, gz_sim, ros_gz_bridge, robot_state_publisher, ...
  │   ├ rs485: rplidar, car_base_node, ...
  │   ├ raspberry: rplidar_c1, car_base_node, battery_bridge
  │   └ rplidar_s2l: rplidar_node, static_tf
  ├ rviz2
  └ node_watchdog
       │
       ▼ wait_for_topic(scan, timeout=120s)
[传感器融合] (根据 profile sensing.type 决定)
  ├ EKF (gazebo/rs485/raspberry): ekf_filter_node
  └ rf2o (rplidar_s2l): rf2o_laser_odometry
       │
       ▼ wait_for_tf(odom→base, timeout=60s)
[SLAM]
  └ slam_toolbox online_async → map + TF map→odom
       │
       ▼ 3s 延迟
[Teleop]
  └ ackermann_keyboard_teleop (可选, use_teleop:=True)
```

#### 场景2: 自动探索建图 (explore_main.launch.py)

```
[立即启动]
  ├ 硬件层 + rviz2 + watchdog (同上)
       │
       ▼ wait_for_topic(scan) → EKF
       │
       ▼ wait_for_tf(odom→base)
[SLAM]
  └ slam_toolbox online_async (提供 map→odom TF, 无需 AMCL)
       │
       ▼ wait_for_tf(map→base, timeout=120s)
[Nav2 导航]
  ├ navigation.launch.py (Nav2 全栈, 无 map_server/AMCL)
  ├ cmd_vel_bridge (gazebo/rs485 only)
  └ lifecycle_starter_explore
       │
       ▼ wait_for_service(Nav2 就绪)
[探索]
  └ frontier_explorer → NavigateToPose → Nav2 → cmd_vel
```

#### 场景3: 调度集成 (nav_main.launch.py)

```
GroupAction(PushRosNamespace(namespace))  ← 所有节点在 namespace 内
┌──────────────────────────────────────────────────────────────┐
│ [立即启动]                                                     │
│   ├ 硬件层 (防御性 remappings → 话题自动 namespace)            │
│   ├ rviz2 + watchdog (TF remapping)                          │
│   │                                                           │
│   ▼ wait_for_topic(scan) → EKF                                │
│   │                                                           │
│   ▼ wait_for_tf(odom→base)                                    │
│ [定位]                                                         │
│   └ localization.launch.py (namespace 传递, AMCL 初始位姿修复) │
│       │                                                       │
│       ▼ wait_for_topic(map)                                   │
│ [导航 + 应用]                                                  │
│   ├ navigation.launch.py (namespace 传递, PushRosNamespace)   │
│   ├ lifecycle_starter_custom                                  │
│   ├ cmd_vel_bridge (gazebo/rs485 only)                        │
│   └ opentcs_vehicle_node (TF remapping, 话题参数化)           │
│       │                                                       │
│       ▼ wait_for_service(route_server/set_route_graph)        │
│ [路线 + 物料]                                                  │
│   ├ route_graph_loader (订阅 route_graph_json, 话题参数化)       │
│   └ material_action_gui (5s 延迟)                             │
└──────────────────────────────────────────────────────────────┘
```

### 7.2 统一启动脚本

所有场景通过统一的 shell 脚本入口，`--profile` 参数选择硬件配置，`--domain-id` 覆盖 DDS 域，`--namespace` 设置多车命名空间：

```bash
# ── 场景1: 手工建图 ──────────────────────────────────────────
./scripts/launch/slam.sh                                # gazebo 仿真 (默认)
./scripts/launch/slam.sh --profile rs485                # RS-485 实车
./scripts/launch/slam.sh --profile raspberry            # 树莓派
./scripts/launch/slam.sh --profile rplidar_s2l          # 纯激光雷达 (无底盘)
./scripts/launch/slam.sh --no-teleop                    # 不启动键盘遥控
./scripts/launch/slam.sh --slam-params /path/to/params  # 自定义SLAM参数
./scripts/launch/slam.sh --domain-id 30                 # 覆盖 DDS Domain ID

# ── 场景2: 自动探索建图 ─────────────────────────────────────
./scripts/launch/explore.sh                             # gazebo 仿真 (默认)
./scripts/launch/explore.sh --profile raspberry         # 树莓派

# ── 场景3: 调度集成 ─────────────────────────────────────────
./scripts/launch/dispatch.sh                            # gazebo 仿真 (默认)
./scripts/launch/dispatch.sh --profile rs485            # RS-485 实车
./scripts/launch/dispatch.sh --profile raspberry        # 树莓派
./scripts/launch/dispatch.sh --map /path/to/map.yaml    # 指定地图
./scripts/launch/dispatch.sh --namespace c30_1          # 多车 namespace
./scripts/launch/dispatch.sh --profile raspberry --namespace c30_1 --map maps/my_map.yaml

# ── 辅助工具 ────────────────────────────────────────────────
./scripts/launch/save_map.sh -f maps/my_map             # 保存地图
./scripts/launch/teleop.sh                              # 键盘遥控 (独立终端)
./scripts/launch/teleop.sh --domain-id 30               # 指定 Domain ID
```

### 7.3 场景 × 硬件组合矩阵

| | gazebo (仿真) | rs485 (实车) | raspberry (树莓派) | rplidar_s2l (纯雷达) |
|---|---|---|---|---|
| **手工建图** | `slam.sh` | `slam.sh --profile rs485` | `slam.sh --profile raspberry` | `slam.sh --profile rplidar_s2l` |
| **自动探索** | `explore.sh` | `explore.sh --profile rs485` | `explore.sh --profile raspberry` | — |
| **调度集成** | `dispatch.sh` | `dispatch.sh --profile rs485` | `dispatch.sh --profile raspberry` | — |
| **多车调度** | — | `dispatch.sh --namespace c30_1` | `dispatch.sh --profile raspberry --namespace c30_1` | — |

### 7.4 推荐使用流程

1. **建图阶段**: `./scripts/launch/slam.sh` (手动遥控) 或 `./scripts/launch/explore.sh` (自动探索)
2. **保存地图**: `./scripts/launch/save_map.sh -f maps/my_map`
3. **单车调度**: `./scripts/launch/dispatch.sh --map maps/my_map.yaml`
4. **多车调度**: 每台 Pi 执行 `./scripts/launch/dispatch.sh --profile raspberry --namespace c30_N --map maps/my_map.yaml`
5. **openTCS 调度**: 另开终端启动 openTCS Kernel + PlantOverview

---

## 八、通信协议

### 8.1 RS-485 协议 (gazebo / rs485 profile)

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

### 8.2 UART 协议 (raspberry profile - car_base_node ↔ STM32)

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

## 九、坐标系定义

| Frame | 发布者 | 说明 |
|-------|--------|------|
| `map` | — | 全局固定坐标系 (世界) |
| `odom` | EKF | 里程计坐标系 (平滑漂移) |
| `body_link` / `base_link` | EKF | 机器人底盘中心 |
| `<namespace>/body_link/lidar` / `laser` / `lidar_link` | static TF | 激光雷达安装位置 |
| `imu_link` | static TF / URDF | IMU 传感器 |
| `front_*_steering_link` | robot_state_publisher | 前轮转向关节 |
| `*_wheel_link` | robot_state_publisher | 车轮 |

---

## 十、机器人参数

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

## 十一、关键设计决策

### 11.1 为什么用 lifecycle_starter 替代 Nav2 lifecycle_manager?

Nav2 原生 lifecycle_manager 在 Fast-DDS (rmw_fastrtps_cpp) 下偶尔出现 service call 超时，
导致节点无法激活。自定义 `lifecycle_starter` 提供超时+重试机制，确保可靠启动。

### 11.2 为什么 Ackermann 不能原地旋转?

Ackermann 转向类似汽车，需要前进才能转弯。因此：
- 行为树移除了 `Spin` 恢复动作，替换为 `Wait`
- cmd_vel_bridge 在低速+有角速度时强制给予 0.05m/s 蠕行速度
- 规划器使用 SmacPlannerHybrid (Dubin)，前进弧线运动模型

### 11.3 为什么树莓派不需要 cmd_vel_bridge 和 rs485_bridge?

树莓派小车的 STM32 内部完成 Ackermann 运动学解算。ROS 侧只需发送原始 `cmd_vel` (Twist)，
STM32 负责将线速度和角速度转换为左右轮差速和前轮转角。这简化了 ROS 侧的控制链。

### 11.4 硬件 Profile 机制

`hardware_profile` 参数在 launch 时传入，通过 `OpaqueFunction` 在运行时加载对应的
`config/profiles/{profile}.yaml` 配置文件，决定：
- 启动哪个硬件子 launch
- DDS Domain ID（从 `domain_id` 字段读取）
- 传感器融合方式：`sensing.type: ekf` (EKF) 或 `rf2o` (激光里程计)
- EKF 的 frame 名称和 sensor 话题
- lifecycle_starter_custom 管理的节点列表
- 是否启动 cmd_vel_bridge / rs485_bridge

### 11.5 三层解耦设计

系统按 **应用层 × 中间层 × 硬件层** 正交解耦：

- **应用层** (3个场景入口)：slam_main / explore_main / nav_main
- **中间层** (复杂子系统)：navigation.launch.py / localization.launch.py
- **硬件层** (4种 profile)：gazebo / rs485 / raspberry / rplidar_s2l

同一场景入口通过 `hardware_profile` 参数适配不同硬件，同一硬件可跑不同场景。
跨层服务 (watchdog + lifecycle_starter + rviz2) 在所有场景中共享。

### 11.6 DDS Domain + ROS2 Namespace 混合隔离策略

两个独立的隔离机制：

- **DDS Domain ID**：隔离 DDS 发现范围（仿真 domain 42 vs 生产 domain 30），不同 domain 间完全不可见
- **ROS2 Namespace**：同一 domain 内隔离话题名（多车场景），每台车有独立的话题空间

设计选择：
- 所有 YAML 和节点中的话题名使用**相对路径**，namespace 由 `PushRosNamespace` 在 launch 时注入
- 第三方不可修改的节点（`rplidar_node`, `car_base_node`）通过**防御性 remappings** 将绝对话题转为相对
- TF 话题通过 `('/tf', 'tf')` remapping 隔离到 `/<namespace>/tf`
- Gazebo `robot_description` 使用 `/<namespace>/robot_description`，避免多车 controller_manager 订阅同一 URDF
- Gazebo `/clock` 保持全局单发布者；`--no-gazebo` 的车辆不桥接 `/clock`
- openTCS Sidecar 话题保持**绝对路径**，通过 `vehicle_name` 参数构造前缀
- `namespace=''`（默认）时所有隔离机制为 no-op → 完全向后兼容单车模式

---

## 十二、文件结构

```
lidar-slam/
├── launch/                               # ── Launch 文件 (三层架构) ──
│   │
│   │ ── 应用层 (场景入口) ─────────────────────────────────
│   ├── slam_main.launch.py             ★ 场景1: 手工建图
│   ├── explore_main.launch.py          ★ 场景2: 自动探索建图
│   ├── nav_main.launch.py              ★ 场景3: 调度集成 (GroupAction + namespace)
│   │
│   │ ── 中间层 (复杂子系统) ───────────────────────────────
│   ├── navigation.launch.py            ★ Nav2 导航栈 (10 节点, PushRosNamespace)
│   ├── localization.launch.py          ★ AMCL 定位 (PushRosNamespace)
│   │
│   │ ── 硬件抽象层 ──────────────────────────────────────
│   └── hardware/
│       ├── gazebo_hardware.launch.py   ★ Gazebo 仿真
│       ├── rs485_hardware.launch.py    ★ RS-485 实车 (防御性 remappings)
│       ├── raspberry_hardware.launch.py★ 树莓派小车 (防御性 remappings)
│       └── rplidar_s2l_hardware.launch.py★ 纯激光雷达 (无底盘)
│
├── config/
│   ├── profiles/                        # 硬件 Profile 配置 (含 domain_id)
│   │   ├── gazebo.yaml                 ★ Gazebo (domain_id: 42)
│   │   ├── rs485.yaml                  ★ RS-485 (domain_id: 30)
│   │   ├── raspberry.yaml              ★ 树莓派 (domain_id: 30)
│   │   └── rplidar_s2l.yaml            ★ 纯雷达 (domain_id: 30)
│   ├── ekf.yaml                        # EKF 传感器融合 (相对话题名)
│   ├── nav2_params_opentcs.yaml        # Nav2 参数 (相对话题名)
│   ├── nav2_params_exploration.yaml    # Nav2 参数 (相对话题名)
│   ├── slam_toolbox_*.yaml             # SLAM 参数 (多种)
│   ├── rs485_bridge.yaml               # RS-485 串口配置
│   ├── opentcs_vehicle.yaml            # openTCS 车辆配置 (相对话题名)
│   ├── watchdog.yaml                   # 系统监控配置 (相对话题名)
│   └── material_action.yaml            # 物料操作配置
│
├── scripts/
│   ├── launch/                          # ── 统一启动脚本 ──
│   │   ├── slam.sh                     ★ 手工建图 (--profile, --domain-id)
│   │   ├── explore.sh                  ★ 自动探索建图
│   │   ├── dispatch.sh                 ★ 调度集成 (--profile, --namespace, --domain-id)
│   │   ├── save_map.sh                 ★ 保存地图
│   │   ├── teleop.sh                   ★ 键盘遥控 (--domain-id)
│   │   ├── view_tf_tree.sh             # TF 树查看
│   │   └── rplidar_s2_view.sh          # RPLIDAR 驱动+可视化
│   ├── tools/
│   │   ├── cleanup_ros2.sh             # 清理残留进程
│   │   ├── diagnose_ackermann.sh       # 诊断工具
│   │   ├── map_to_sdf.py               # 地图转 Gazebo SDF
│   │   └── sim_sidecar_goal.py         # 模拟 openTCS Sidecar
│   ├── check/
│   │   ├── sim_check.sh                # 仿真环境预检
│   │   └── diagnose_inflation.py       # Nav2 InflationLayer 诊断
│   └── deploy_remote.py                # 远程部署工具
│
├── src/                                 # ── 源代码 ──
│   ├── lidar_slam_nodes/               # 自定义 Python 节点包 (16 节点)
│   │   └── lidar_slam_nodes/
│   │       ├── cmd_vel_bridge.py
│   │       ├── rs485_chassis_bridge.py
│   │       ├── rs485_chassis_receiver.py
│   │       ├── rs485_protocol.py
│   │       ├── frontier_explorer.py
│   │       ├── opentcs_vehicle_node.py  # 话题已参数化, 支持 namespace
│   │       ├── lifecycle_starter.py
│   │       ├── node_watchdog.py         # 话题名由 watchdog.yaml 配置
│   │       ├── route_graph_loader.py    # 话题已参数化
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
│
├── third-party/                        # Git 子模块
│   ├── slam_toolbox/                   # SLAM 工具箱 (fork)
│   ├── m-explore-ros2/                 # 自主探索
│   ├── gazebo_ackermann_steering_vehicle/ # Gazebo 车辆模型
│   ├── rplidar_sdk/                    # RPLIDAR SDK
│   ├── openTCS-NeNa/                   # openTCS 车队调度
│   └── aws-robomaker-small-warehouse-world/ # Gazebo 仿真世界
│
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

## 十三、部署指南

### 13.1 仿真环境 (开发机)

```bash
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 编译
colcon build --symlink-install

# 手工建图 (仿真, domain 42)
./scripts/launch/slam.sh

# 自动探索 (仿真)
./scripts/launch/explore.sh

# 调度集成 (仿真)
./scripts/launch/dispatch.sh --map maps/my_map.yaml
```

### 13.2 树莓派小车

```bash
# 首次部署: 从开发机上传代码
conda run -n lidar_slam python3 scripts/deploy_remote.py check  # 检查连接

# 在树莓派上编译
cd /home/pi/lidar-slam
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 3

# 手工建图 (树莓派, domain 30)
./scripts/launch/slam.sh --profile raspberry

# 单车调度 (树莓派)
./scripts/launch/dispatch.sh --profile raspberry --map maps/my_map.yaml

# 多车调度 — Pi 1
./scripts/launch/dispatch.sh --profile raspberry --namespace c30_1 --map maps/my_map.yaml

# 多车调度 — Pi 2
./scripts/launch/dispatch.sh --profile raspberry --namespace c30_2 --map maps/my_map.yaml

# 环境变量已在 ~/.bashrc 中配置:
#   LIDAR_SLAM_ROOT=/home/pi/lidar-slam
#   ROS_DOMAIN_ID=30
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   source /home/pi/lidar-slam/install/setup.bash
```

### 13.3 RS-485 实车

```bash
# 确认串口设备存在
ls /dev/ttyUSB0 /dev/ttyUSB1

# 调度集成 (RS-485)
./scripts/launch/dispatch.sh --profile rs485 --map maps/my_map.yaml

# 多车调度
./scripts/launch/dispatch.sh --profile rs485 --namespace c30_1 --map maps/my_map.yaml
```

### 13.4 纯激光雷达建图

```bash
# 手持建图 (笔记本 + RPLIDAR S2L)
./scripts/launch/slam.sh --profile rplidar_s2l

# 保存地图
./scripts/launch/save_map.sh -f maps/handheld_map
```

---

## 十四、监控与诊断

### 14.1 系统健康监控 (node_watchdog)

每 5 秒检查一次，关键指标：

| 类别 | 监控对象 | 告警级别 |
|------|---------|---------|
| 关键节点 | ekf, amcl, controller_server, bt_navigator, cmd_vel_bridge, opentcs_vehicle | ERROR |
| 关键话题 | scan, odom, tf, cmd_vel, amcl_pose | ERROR |
| 非关键节点 | rviz2, material_action_gui | WARN |

### 14.2 诊断命令

```bash
# 查看 TF 树 (namespace 模式)
ros2 run tf2_tools view_frames

# 检查话题频率 (namespace 模式需指定完整路径)
ros2 topic hz /c30_1/scan
ros2 topic hz /c30_1/odom

# 查看节点状态
ros2 lifecycle list /c30_1/cmd_vel_bridge

# 验证 namespace 隔离
ros2 topic list | grep c30_1   # 应看到 /c30_1/scan, /c30_1/odom 等

# 验证 Gazebo 多车无同名异类型 topic
ros2 topic list -t | awk -F'[][]' 'NF>=3 && $2 ~ /,/ {print}'

# 验证 robot_description 和 clock 隔离
ros2 topic info /gazebo_1/robot_description -v
ros2 topic info /clock -v       # 应只有一个 ros_gz_bridge publisher

# 验证 domain 隔离 (domain 42 不应看到 domain 30 的话题)
ros2 topic list                 # 应无 Pi 发布的话题

# 诊断 Ackermann 系统
bash scripts/tools/diagnose_ackermann.sh
```
