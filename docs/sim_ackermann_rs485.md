# sim_ackermann_rs485 — RS-485 底盘协议仿真导航架构

## 功能概述

`sim_ackermann_rs485.launch.py` 是 **带 RS-485 底盘通信协议仿真的 Ackermann 导航 + openTCS 桥接** 启动文件。它在 opentcs 导航场景的基础上，在控制链路中插入一层 RS-485 串口协议仿真，使上位机软件**同时适用于仿真和实物**，切换时只需改串口路径。

核心特点：

1. 通过 **socat 虚拟串口对** 模拟物理 RS-485 总线
2. 控制指令经 **485 帧编解码**（23 字节固定帧，10 通道，AA55/A5 帧头帧尾）
3. 完整保留 opentcs 场景的导航定位 + 车队调度能力
4. **实物迁移**时只改 `serial_port` 参数，去掉 receiver 和 vehicle_controller

**与 opentcs 场景的核心区别**：控制指令不再从 `cmd_vel_bridge` 直达 `vehicle_controller`，而是经过 `rs485_chassis_bridge` → 虚拟串口 → `rs485_chassis_receiver` 的协议仿真链路，复现真实控制卡的通信行为。

## 系统架构

```
时间轴(s)  节点/组件
─────────────────────────────────────────────────────
  0s     socat 虚拟串口对 (/tmp/chassis_cmd ↔ /tmp/chassis_recv)
  0s     Gazebo 仿真器 + ros_gz_bridge + 静态TF(lidar)
  0s     robot_state_publisher (URDF)
  5s     spawn robot + EKF 融合 (odom+imu → TF)
 10s     load_controllers (ros2_control 控制器加载)
 12s     rs485_chassis_receiver + vehicle_controller (remap)
 13s     rs485_chassis_bridge
 15s     Nav2 定位 (map_server + AMCL)
 25s     Nav2 导航 (规划+控制+代价地图)
  0s     cmd_vel_bridge (Twist→阿克曼)
 30s     opentcs_nav2_bridge (openTCS↔Nav2 桥接)
  0s     RViz2 可视化
```

## ROS2 节点列表

| # | 节点名 | 包 | 功能 |
|---|--------|---|------|
| 1 | `gz_sim` | `ros_gz_sim` | Gazebo Harmonic 仿真器，加载 factory.sdf 世界 |
| 2 | `parameter_bridge` | `ros_gz_bridge` | Gazebo↔ROS2 消息桥接（含 /tf） |
| 3 | `robot_state_publisher` | `robot_state_publisher` | 发布 URDF 和关节 TF |
| 4 | `socat` | 系统进程 | 虚拟串口对，模拟 RS-485 物理总线 |
| 5 | `rs485_chassis_bridge` | `lidar_slam_nodes` | 订阅 steering/velocity，编码为 485 帧写入串口 |
| 6 | `rs485_chassis_receiver` | `lidar_slam_nodes` | 从串口读取 485 帧，解码发布到 /rs485/ 话题 |
| 7 | `vehicle_controller` | `ackermann_control` | 阿克曼转向几何解算（订阅 /rs485/ 话题） |
| 8 | `ekf_filter_node` | `robot_localization` | EKF 融合 /odom + /imu → 发布 odom→body_link TF |
| 9 | `map_server` | `nav2_map_server` | 加载已知地图 `auto_exploration_map.yaml` 到 /map |
| 10 | `amcl` | `nav2_amcl` | 自适应蒙特卡洛定位，发布 map→odom TF |
| 11 | `controller_server` | `nav2_controller` | RegulatedPurePursuit 路径跟踪 |
| 12 | `planner_server` | `nav2_planner` | NavfnPlanner (Dijkstra/A*) |
| 13 | `bt_navigator` | `nav2_bt_navigator` | 行为树导航协调器 |
| 14 | `behavior_server` | `nav2_behaviors` | 旋转/后退/等待等恢复行为 |
| 15 | `smoother_server` | `nav2_smoother` | 路径平滑 |
| 16 | `velocity_smoother` | `nav2_velocity_smoother` | 速度平滑 |
| 17 | `collision_monitor` | `nav2_collision_monitor` | 碰撞监控（默认关闭） |
| 18 | `cmd_vel_bridge` | `lidar_slam_nodes` | Nav2 Twist → 阿克曼 steering_angle/velocity |
| 19 | `opentcs_nav2_bridge` | `lidar_slam_nodes` | openTCS ↔ Nav2 协议桥接 |
| 20 | `rviz2` | `rviz2` | 可视化 |

## RS-485 协议规范

### 帧结构（23 字节固定长度）

```
[0xAA][0x55] | [CH1_H][CH1_L] [CH2_H][CH2_L] ... [CH10_H][CH10_L] | [0xA5]
  帧头(2B)            数据区(20B)，每通道2字节大端序                    帧尾(1B)
```

### 通道定义

| 通道 | 值范围 | 功能 | 映射公式 |
|------|--------|------|---------|
| CH1 | 1000=左满, 1500=中, 2000=右满 | 转向 | `ch = 1500 + 500 × (angle / 0.5236)` |
| CH2 | 1000=全后退, 1500=停, 2000=全前进 | 速度 | `ch = 1500 + 500 × (speed / 1.4)` |
| CH3-4 | 1000=断, 2000=通 | 继电器（点动） | 直通 |
| CH5-6 | 1000=断, 2000=通 | 继电器（自锁） | 直通 |
| CH7-8 | 固定 1500 | 预留 | — |
| CH9-10 | 1000-2000 | 模拟量 0-5V | 直通 |

### 通信规则

- 刷新间隔：50~300ms（默认 200ms）
- 超时 500ms 未收到有效帧 → 全通道归 1500（中性/停止）
- 值范围自动限幅到 [1000, 2000]
- 分辨率：转向 0.06°/step，速度 0.0028 m/s/step

### 帧示例

"前进最大 + 左转最大"：`AA 55 03 E8 07 D0 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC A5`

## 话题设计

### 完整控制流

```
openTCS (外部调度)
  └─ /goal_pose (PoseStamped)  ──→  opentcs_nav2_bridge
                                       └─ NavigateToPose action ──→  bt_navigator
                                                                       └─ planner_server
                                                                       └─ controller_server (RPP)
                                                                          └─ /cmd_vel (Twist)
                                                                             └─ cmd_vel_bridge
                                                                                ├─ /steering_angle (Float64)  ──→  rs485_chassis_bridge
                                                                                └─ /velocity (Float64)        ──→  rs485_chassis_bridge
                                                                                                                  │
                                                                                                        编码485帧 (23B)
                                                                                                        pyserial.write()
                                                                                                                  │
                                                                                                            /tmp/chassis_cmd
                                                                                                                  │
                                                                                                          [socat 虚拟串口]
                                                                                                                  │
                                                                                                            /tmp/chassis_recv
                                                                                                                  │
                                                                                                        pyserial.read()
                                                                                                        扫描AA55帧头
                                                                                                        校验A5帧尾
                                                                                                                  │
                                                                                                        rs485_chassis_receiver
                                                                                                                  │
                                                                                                /rs485/steering_angle (Float64)
                                                                                                /rs485/velocity (Float64)
                                                                                                                  │
                                                                                                        vehicle_controller (remap)
                                                                                                                  │
                                                                                                /forward_position_controller/commands
                                                                                                /forward_velocity_controller/commands
                                                                                                                  │
                                                                                                        ros2_control → Gazebo
```

### 传感器数据流

```
Gazebo
  ├─ /scan   (LaserScan)  ──→  AMCL, Nav2 costmap
  ├─ /odom   (Odometry)   ──→  ekf_filter_node, Nav2
  ├─ /imu    (Imu)        ──→  ekf_filter_node
  ├─ /tf     (TFMessage)  ──→  TF 系统（Gazebo 模型位姿）
  └─ /clock  (Clock)      ──→  全局时钟
```

### 位置上报流

```
TF 树 (map → body_link)
  └─ opentcs_nav2_bridge 定时查询 TF
      └─ /amcl_pose (PoseWithCovarianceStamped)  ──→  openTCS
```

### 关键话题汇总

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/cmd_vel` | `Twist` | Nav2 controller | `cmd_vel_bridge` | 导航速度指令 |
| `/steering_angle` | `Float64` | `cmd_vel_bridge` | `rs485_chassis_bridge` | 弧度，±0.5236 |
| `/velocity` | `Float64` | `cmd_vel_bridge` | `rs485_chassis_bridge` | m/s，±1.4 |
| `/rs485/steering_angle` | `Float64` | `rs485_chassis_receiver` | `vehicle_controller` | 经485解码后的转向 |
| `/rs485/velocity` | `Float64` | `rs485_chassis_receiver` | `vehicle_controller` | 经485解码后的速度 |
| `/forward_position_controller/commands` | `Float64MultiArray` | `vehicle_controller` | ros2_control | 左/右转向角 |
| `/forward_velocity_controller/commands` | `Float64MultiArray` | `vehicle_controller` | ros2_control | 左/右轮速 |
| `/scan` | `LaserScan` | ros_gz_bridge | AMCL, Nav2 costmap | 激光扫描 |
| `/odom` | `Odometry` | ros_gz_bridge | ekf_filter_node | 里程计 |
| `/imu` | `Imu` | ros_gz_bridge | ekf_filter_node | 惯性测量 |
| `/map` | `OccupancyGrid` | map_server | Nav2 static_layer, AMCL | 静态地图 |
| `/goal_pose` | `PoseStamped` | openTCS | `opentcs_nav2_bridge` | 调度目标点 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | `opentcs_nav2_bridge` | openTCS | 位姿上报 |

## TF 树

```
map
 └─ odom          (由 AMCL 发布，粒子滤波定位)
     └─ body_link  (由 ekf_filter_node 发布，融合 odom+imu)
         ├─ ackermann_robot/body_link/lidar  (静态 TF, z=0.22)
         ├─ front_left_steering_link  (由 robot_state_publisher 发布)
         │   └─ front_left_wheel_link
         ├─ front_right_steering_link
         │   └─ front_right_wheel_link
         ├─ rear_left_wheel_link
         └─ rear_right_wheel_link
```

## 自定义节点详解

### rs485_chassis_bridge（发送端）

订阅 `/steering_angle` 和 `/velocity`，编码为 RS-485 协议帧写入串口。

| 行为 | 说明 |
|------|------|
| 定时发送 | 每 `refresh_interval_ms`（默认 200ms）编码并发送一帧 |
| 通道映射 | CH1=转向，CH2=速度，CH3-10=1500（默认） |
| 超时保护 | 500ms 未收到新消息 → 发送中性帧（全通道 1500） |
| 值限幅 | 自动 clamp 到 [1000, 2000] |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/tmp/chassis_cmd` | 仿真虚拟串口；实物改为 `/dev/ttyUSB1` |
| `baudrate` | `115200` | 与控制卡一致 |
| `refresh_interval_ms` | `200` | 协议要求 50~300ms |
| `timeout_ms` | `500` | 协议要求 500ms 超时 |
| `max_steering_angle` | `0.5236` | 与 ackermann_params.yaml 一致 |
| `max_velocity` | `1.4` | 与 ackermann_params.yaml 一致 |

### rs485_chassis_receiver（接收端，仅仿真用）

从串口读取 RS-485 协议帧，解码后发布到 `/rs485/` 话题。仿真环境专用，实物底盘由 MCU 替代。

| 行为 | 说明 |
|------|------|
| 读帧 | 100Hz 非阻塞读取，扫描 AA55 帧头，校验 A5 帧尾 |
| 发布 | CH1 → `/rs485/steering_angle`，CH2 → `/rs485/velocity` |
| 超时 | 500ms 无有效帧 → 发布 (0.0, 0.0) |
| 前缀 | `/rs485/` 前缀避免与 bridge 订阅的 `/steering_angle` 形成环路 |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/tmp/chassis_recv` | 仿真虚拟串口 |
| `baudrate` | `115200` | 与控制卡一致 |
| `timeout_ms` | `500` | 无帧超时 |
| `max_steering_angle` | `0.5236` | 通道值→弧度映射 |
| `max_velocity` | `1.4` | 通道值→速度映射 |

### cmd_vel_bridge

将 Nav2 的差速驱动 `Twist` 指令转换为阿克曼转向指令（与 opentcs 场景相同）。

### opentcs_nav2_bridge

openTCS ↔ Nav2 协议桥接（与 opentcs 场景相同）。

## 超时安全机制

系统有三层超时保护，从外到内依次生效：

```
┌─ 第1层: rs485_chassis_bridge (500ms) ──────────────────────┐
│  cmd_vel_bridge 超时 → bridge 发送全 1500 中性帧            │
│                                                             │
│  ┌─ 第2层: rs485_chassis_receiver (500ms) ───────────────┐  │
│  │  无有效帧 → receiver 发布 (0.0, 0.0)                  │  │
│  │                                                       │  │
│  │  ┌─ 第3层: vehicle_controller (800ms) ──────────────┐ │  │
│  │  │  无话题消息 → 车轮速度归零，转向归零              │ │  │
│  │  └──────────────────────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘

正常停车延迟: cmd_vel_bridge停止 → 500ms → bridge发中性帧
                               → 500ms → receiver发零值
                               → 800ms → vehicle_controller零轮速
总计: ~1300ms（三层串行）
```

## 与 opentcs 场景的对比

| 维度 | opentcs（直接控制） | rs485（协议仿真） |
|------|-------------------|------------------|
| **控制链路** | `cmd_vel_bridge → vehicle_controller`（直连） | `cmd_vel_bridge → rs485_bridge → [socat] → rs485_receiver → vehicle_controller` |
| **vehicle_controller 订阅** | `/steering_angle`, `/velocity` | `/rs485/steering_angle`, `/rs485/velocity` |
| **串口通信** | 无 | socat 虚拟串口对 |
| **485 帧编解码** | 无 | bridge 编码，receiver 解码 |
| **超时层数** | 2 层（cmd_vel_bridge + vehicle_controller） | 3 层（+ rs485 协议超时） |
| **ackermann_control 引入方式** | `IncludeLaunchDescription` 整包 | 拆开为独立节点（为了 remap） |
| **停车延迟** | ~1300ms（2层） | ~1800ms（3层串行） |
| **实物迁移** | 不支持 | 改串口路径即可 |
| **其余功能** | 导航+调度 | 导航+调度（完全保留） |

## 实物迁移指南

部署到实物底盘时，需要修改的内容：

### 保留运行的节点

| 节点 | 配置变更 |
|------|---------|
| `rs485_chassis_bridge` | `serial_port` 改为 `/dev/ttyUSB1`（实际 USB-RS485 适配器） |
| `cmd_vel_bridge` | 无变更 |
| `opentcs_nav2_bridge` | 无变更 |
| Nav2 全套 | 无变更 |

### 不再运行的节点

| 节点 | 原因 |
|------|------|
| `socat` | 不需要虚拟串口 |
| `rs485_chassis_receiver` | 实物底盘 MCU 直接处理 485 帧 |
| `vehicle_controller` | 实物底盘 MCU 直接驱动电机 |

### 配置差异

| 参数 | 仿真值 | 实物值 |
|------|--------|--------|
| `rs485_chassis_bridge.serial_port` | `/tmp/chassis_cmd` | `/dev/ttyUSB1` |
| `rs485_chassis_bridge.baudrate` | 115200 | 115200 |
| `use_sim_time` | `True` | `False` |
