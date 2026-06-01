# sim_ackermann_rs485 — RS-485 底盘协议仿真导航架构

## 功能概述

`sim_ackermann_rs485.launch.py` 是 **带 RS-485 底盘通信协议仿真的 Ackermann 导航 + openTCS 桥接** 启动文件。它在 opentcs 导航场景的基础上，在控制链路中插入一层 RS-485 串口协议仿真，使上位机软件**同时适用于仿真和实物**，切换时只需一个启动参数。

核心特点：

1. 通过 **socat 虚拟串口对** 模拟物理 RS-485 总线（仿真模式）
2. 控制指令经 **485 帧编解码**（23 字节固定帧，10 通道，AA55/A5 帧头帧尾）
3. 完整保留 opentcs 场景的导航定位 + 车队调度能力
4. **6 个自定义节点全部为 LifecycleNode**，由自定义 `lifecycle_starter` 统一管理（替代 Nav2 的 `lifecycle_manager`）
5. **事件驱动启动**：基于 `wait_for_topic` / `wait_for_service` / `wait_for_tf` + `lifecycle_starter/ready` 就绪信号，替代固定延时
6. **保活机制**：`respawn=True` + `lifecycle_starter` 超时重试 + `node_watchdog` 健康监控
7. **仿真/实物参数化**：通过 `simulation:=True/False` 一个参数切换，无需改代码

**与 opentcs 场景的核心区别**：控制指令不再从 `cmd_vel_bridge` 直达 `vehicle_controller`，而是经过 `rs485_chassis_bridge` → 虚拟串口 → `rs485_chassis_receiver` 的协议仿真链路，复现真实控制卡的通信行为。

## 启动方式

### 仿真环境（Gazebo）

```bash
./scripts/launch/sim_ackermann_rs485.sh
# 或带参数覆盖：
# ./scripts/launch/sim_ackermann_rs485.sh vehicle_name:=robot_1
```

### 真实小车

```bash
./scripts/launch/real_ackermann_nav.sh
# 等同于：simulation:=False use_sim_time:=False
```

### Launch Arguments

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `simulation` | `True` | True=Gazebo仿真, False=真实硬件 |
| `use_sim_time` | `True` | 全局时钟模式 |
| `use_respawn` | `True` | 节点崩溃自动重启 |
| `vehicle_name` | `ackermann_robot` | openTCS 车辆标识 |
| `namespace` | `''` | 多车命名空间 |

## 系统架构

### 事件驱动启动序列

```
就绪条件                           启动节点
──────────────────────────────────────────────────────────────
T=0 (立即)                         socat (sim) + gz_sim (sim) + bridge (sim)
                                   robot_state_publisher + laser_tf + rviz2
                                   node_watchdog (保活监控)
  │
  ├─ wait_for_topic(/scan) ───────→ spawn_robot (sim) + ekf
  │
  ├─ spawn_robot 退出 (sim) ──────→ wait_for_service(controller_manager)
  │
  ├─ controller_manager 就绪 ────→ load_controllers (sim)
  │
  ├─ load_controllers 退出 ──────→ wait_for_topic(/joint_states)
  │
  ├─ /joint_states 就绪 ────────→ rs485_receiver (sim) + vehicle_controller (sim)
  │                                 └─ 1s 后 → rs485_bridge
  │
  ├─ wait_for_tf(                ──→ map_server + amcl + lifecycle_starter_localization
  │   odom→body_link)
  │
  ├─ lifecycle_starter_localization  → navigation (10个Nav2节点)
  │  /ready 信号                    + lifecycle_starter_custom
  │                                 + cmd_vel_bridge + opentcs_vehicle_node
  │
  └─ wait_for_service(            ──→ route_graph_loader
      route_server)                  + 5s 后 → material_action_gui
```

### 真实小车启动序列

```
就绪条件                           启动节点
──────────────────────────────────────────────────────────────
T=0 (立即)                         robot_state_publisher + laser_tf + rviz2
                                   node_watchdog
  │
  ├─ wait_for_topic(/scan) ───────→ ekf + cmd_vel_bridge
  │                                 └─ 3s 后 → rs485_bridge (直连物理串口 /dev/ttyUSB1)
  │
  ├─ wait_for_tf(                ──→ map_server + amcl + lifecycle_starter_localization
  │   odom→body_link)
  │
  ├─ lifecycle_starter_localization  → navigation + lifecycle_starter_custom
  │  /ready 信号                    + opentcs_vehicle_node
  │
  └─ wait_for_service(            ──→ route_graph_loader
      route_server)
```

## Lifecycle 管理架构

所有自定义节点均为 **LifecycleNode**，由两个自定义 `lifecycle_starter` 和一个 Nav2 `lifecycle_manager` 统一管理状态转换。

### 为什么替换 lifecycle_manager？

Nav2 的 `lifecycle_manager` 使用无超时的阻塞式 service call，在 Fast-DDS + Gazebo CPU 高负载下会无限挂起。自定义 `lifecycle_starter` 替代方案提供：
- **可配置超时**：每次 lifecycle service call 有 30s 超时保护
- **自动重试**：指数退避重试（最多 5 次）
- **两遍启动**：先尝试所有节点，失败的节点在第 2、3 遍重试（处理节点启动时序差异）
- **就绪信号**：通过 `/lifecycle_starter_{name}/ready` topic 通知下游启动链
- **一次性模式**：startup 完成后退出进程，不占用常驻资源

### 三级 Lifecycle 管理

```
lifecycle_starter_localization       (自定义 LifecycleStarter, 一次性)
  ├─ map_server                      (LifecycleNode)
  └─ amcl                            (LifecycleNode)
  └─ 启动完成后发布 /lifecycle_starter_localization/ready 并退出

lifecycle_manager_navigation         (Nav2 内置, 常驻 bond 心跳)
  ├─ controller_server               (LifecycleNode)
  ├─ smoother_server                 (LifecycleNode)
  ├─ planner_server                  (LifecycleNode)
  ├─ route_server                    (LifecycleNode)
  ├─ behavior_server                 (LifecycleNode)
  ├─ velocity_smoother               (LifecycleNode)
  ├─ collision_monitor               (LifecycleNode)
  ├─ bt_navigator                    (LifecycleNode)
  ├─ waypoint_follower               (LifecycleNode)
  └─ docking_server                  (LifecycleNode)

lifecycle_starter_custom             (自定义 LifecycleStarter, 一次性)
  ├─ cmd_vel_bridge                  (LifecycleNode, Python)
  ├─ rs485_chassis_bridge            (LifecycleNode, Python)
  ├─ rs485_chassis_receiver          (LifecycleNode, Python, sim only)
  ├─ opentcs_vehicle_node            (LifecycleNode, Python)
  ├─ route_graph_loader              (LifecycleNode, Python)
  └─ vehicle_controller              (LifecycleNode, C++, sim only)
  └─ 启动完成后发布 /lifecycle_starter_custom/ready 并退出
```

### Lifecycle 状态转换

每个 LifecycleNode 统一遵循：

```
unconfigured ──[on_configure]──→ inactive ──[on_activate]──→ active
                                  ↑                          ↓
                                  └──[on_deactivate]─────────┘
```

| 回调 | 职责 | 典型操作 |
|------|------|---------|
| `on_configure` | 初始化配置 | 声明参数、读取配置、计算派生值、初始化状态变量 |
| `on_activate` | 启动数据流 | 创建 publishers/subscriptions/timers、打开串口 |
| `on_deactivate` | 停止数据流 | 销毁 timers、关闭串口 |
| `on_cleanup` | 释放资源 | 重置内部状态 |
| `on_shutdown` | 最终清理 | — |

### LifecycleStarter 配置

两个 lifecycle_starter 使用自定义参数（定义在 launch file 中）：

| 参数 | localization | custom | 说明 |
|------|-------------|--------|------|
| `node_names` | map_server, amcl | cmd_vel_bridge, rs485_bridge, rs485_receiver, opentcs_vehicle_node, route_graph_loader, vehicle_controller | 管理的节点列表 |
| `configure_timeout` | 30.0 | 30.0 | 单次 configure 超时（秒） |
| `activate_timeout` | 30.0 | 30.0 | 单次 activate 超时（秒） |
| `max_retries` | 5 | 5 | 每次转换的最大重试次数 |
| `retry_delay` | 2.0 | 2.0 | 重试基础延时（指数退避） |
| `startup_delay` | 2.0 | 5.0 | 启动前等待 DDS 稳定时间 |
| `max_passes` | 3 | 3 | 两遍启动的最大遍数 |
| `monitor_period` | 0.0（禁用） | 0.0（禁用） | 健康监控周期（禁用以避免 rclpy.spin 嵌套崩溃） |

### LifecycleManager (navigation) 配置

`lifecycle_manager_navigation` 仍使用 Nav2 原生参数（定义在 `config/nav2_params_opentcs.yaml`）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `autostart` | `true` | 启动时自动 bring up 所有管理节点 |
| `bond_timeout` | `10.0` | 10 秒心跳超时检测 |
| `bond_heartbeat_period` | `0.1` | 100ms 心跳间隔 |
| `attempt_respawn_reconnection` | `true` | respawn 后自动重连 bond |

## 保活与容错机制

### 三层防护

```
┌─ 第 1 层: Launch respawn ─────────────────────────────────┐
│  use_respawn=True, respawn_delay=2.0                       │
│  节点进程崩溃 → 2 秒后自动重启新进程                        │
│                                                            │
│  ┌─ 第 2 层: Lifecycle 管理 ──────────────────────────┐   │
│  │  Navigation 节点: lifecycle_manager bond 心跳       │   │
│  │    每 100ms 发送 heartbeat, 10s 无响应 → 离线      │   │
│  │    respawn 后 attempt_respawn_reconnection 自动重连 │   │
│  │  Localization/Custom 节点: lifecycle_starter 启动时 │   │
│  │    超时重试（30s × 5次 × 3遍），启动后退出          │   │
│  │    崩溃恢复依赖 launch respawn 重启进程后手动/自动  │   │
│  │    重新 configure→activate（或重启 lifecycle_starter）│  │
│  │                                                     │   │
│  │  ┌─ 第 3 层: node_watchdog 监控 ─────────────────┐│   │
│  │  │  每 5 秒检查一次所有节点和关键 topic           ││   │
│  │  │  发布 /system_health (DiagnosticArray)        ││   │
│  │  │  发布 /system_health_summary (JSON String)    ││   │
│  │  │  状态: OK / WARN / ERROR / STALE              ││   │
│  │  └───────────────────────────────────────────────┘│   │
│  └─────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────┘
```

### 崩溃恢复流程

**Navigation 节点（由 lifecycle_manager 管理，自动恢复）：**

```
Nav2 节点崩溃
  → launch respawn (2s) 重启进程
  → lifecycle_manager_navigation 检测到 bond 断裂
  → 新进程启动后 lifecycle_manager 重新连接
  → lifecycle_manager 执行 configure → activate
  → node_watchdog 检测到节点恢复 → 状态恢复 OK
```

**Localization/Custom 节点（由 lifecycle_starter 管理，一次性启动）：**

```
自定义节点崩溃
  → launch respawn (2s) 重启进程（进程重启，但处于 unconfigured 状态）
  → 需要手动运行 lifecycle_starter 重新 configure→activate
  → 或重启整个 launch 文件
  → node_watchdog 检测到节点 MISSING → 状态变为 ERROR
```

### Watchdog 监控规则

配置在 `config/watchdog.yaml`：

| 级别 | 节点 | 话题 |
|------|------|------|
| **CRITICAL** (ERROR) | ekf_filter_node, amcl, controller_server, bt_navigator, cmd_vel_bridge, opentcs_vehicle_node, lifecycle_manager_navigation | /scan, /odom, /tf, /cmd_vel, /amcl_pose |
| **NON-CRITICAL** (WARN) | rviz2, material_action_gui | /route_graph/markers |

> **注意**：`lifecycle_starter_localization` 和 `lifecycle_starter_custom` 是一次性节点，启动完成后即退出，不应列入 watchdog 监控。`lifecycle_manager_localization` 和 `lifecycle_manager_custom` 已被替换为 lifecycle_starter，但 watchdog 配置中仍保留旧名称以保持兼容性。

### 启动就绪检测工具

| 工具 | 文件 | 用途 |
|------|------|------|
| `wait_for_topic` | `lidar_slam_nodes/wait_for_topic.py` | 阻塞直到 topic 有 publisher，exit(0) |
| `wait_for_service` | `lidar_slam_nodes/wait_for_service.py` | 阻塞直到 service 可用，exit(0) |
| `wait_for_tf` | `lidar_slam_nodes/wait_for_tf.py` | 阻塞直到 TF 链可用，exit(0) |

这三个工具配合 launch file 的 `RegisterEventHandler(OnProcessExit(...))` 实现事件驱动启动。

## ROS2 节点列表

### 仿真模式节点（simulation=True）

| # | 节点名 | 包 | 类型 | 功能 |
|---|--------|---|------|------|
| 1 | `gz_sim` | `ros_gz_sim` | 进程 | Gazebo Harmonic 仿真器 |
| 2 | `parameter_bridge` | `ros_gz_bridge` | Node | Gazebo↔ROS2 消息桥接 |
| 3 | `socat` | 系统进程 | 进程 | 虚拟串口对 |
| 4 | `robot_state_publisher` | `robot_state_publisher` | Node | URDF + 关节 TF |
| 5 | `ekf_filter_node` | `robot_localization` | Node | EKF 融合 odom+imu |
| 6 | `map_server` | `nav2_map_server` | LifecycleNode | 加载地图 |
| 7 | `amcl` | `nav2_amcl` | LifecycleNode | 自适应蒙特卡洛定位 |
| 8 | `controller_server` | `nav2_controller` | LifecycleNode | RPP 路径跟踪 |
| 9 | `planner_server` | `nav2_planner` | LifecycleNode | SmacPlannerHybrid (Reeds-Shepp) |
| 10 | `bt_navigator` | `nav2_bt_navigator` | LifecycleNode | 行为树导航协调 |
| 11 | `behavior_server` | `nav2_behaviors` | LifecycleNode | 恢复行为 |
| 12 | `smoother_server` | `nav2_smoother` | LifecycleNode | 路径平滑 |
| 13 | `velocity_smoother` | `nav2_velocity_smoother` | LifecycleNode | 速度平滑 |
| 14 | `collision_monitor` | `nav2_collision_monitor` | LifecycleNode | 碰撞监控 |
| 15 | `route_server` | `nav2_route` | LifecycleNode | 路线图路由 |
| 16 | `waypoint_follower` | `nav2_waypoint_follower` | LifecycleNode | 航点跟踪 |
| 17 | `docking_server` | `opennav_docking` | LifecycleNode | 对接服务 |
| 18 | `lifecycle_starter_localization` | `lidar_slam_nodes` | Node (一次性) | 启动 map_server + amcl 后退出 |
| 19 | `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Node | 管理 10 个 Nav2 节点（bond 心跳） |
| 20 | `lifecycle_starter_custom` | `lidar_slam_nodes` | Node (一次性) | 启动 6 个自定义节点后退出 |
| 21 | `cmd_vel_bridge` | `lidar_slam_nodes` | **LifecycleNode** | Twist→阿克曼转换 |
| 22 | `rs485_chassis_bridge` | `lidar_slam_nodes` | **LifecycleNode** | 485 帧编码→串口 |
| 23 | `rs485_chassis_receiver` | `lidar_slam_nodes` | **LifecycleNode** | 串口→485 帧解码 |
| 24 | `vehicle_controller` | `ackermann_control` | **LifecycleNode** | 阿克曼几何解算 |
| 25 | `opentcs_vehicle_node` | `lidar_slam_nodes` | **LifecycleNode** | openTCS↔Nav2 桥接 |
| 26 | `route_graph_loader` | `lidar_slam_nodes` | **LifecycleNode** | GeoJSON→route_server |
| 27 | `material_action_gui` | `jvs_agv_material_actions` | Node | JVS-VGA 物料操作 GUI |
| 28 | `node_watchdog` | `lidar_slam_nodes` | Node | 系统健康监控 |
| 29 | `rviz2` | `rviz2` | Node | 可视化 |

### 真实小车节点（simulation=False）

移除仿真专用节点，保留以下：

| 节点 | 说明 |
|------|------|
| robot_state_publisher, laser_tf, ekf_filter_node | 基础感知与 TF |
| map_server, amcl (lifecycle) | 定位 |
| lifecycle_starter_localization (一次性) | 启动 map_server + amcl 后退出 |
| Nav2 全套 (lifecycle) | 导航 |
| lifecycle_manager_navigation (常驻) | 管理 10 个 Nav2 节点 |
| lifecycle_starter_custom (一次性) | 启动自定义节点后退出 |
| cmd_vel_bridge, opentcs_vehicle_node | 控制转换与调度桥接 |
| rs485_chassis_bridge | 直连物理 RS-485 适配器 (`/dev/ttyUSB1`) |
| route_graph_loader | GeoJSON 路线图加载 |
| node_watchdog | 健康监控 |

**不运行的节点**：socat, gz_sim, ros_gz_bridge, spawn_robot, load_controllers, rs485_chassis_receiver, vehicle_controller（MCU 替代）。

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

## 话题设计

### 完整控制流（仿真）

```
openTCS (外部调度)
  └─ /goal_pose (PoseStamped)  ──→  opentcs_vehicle_node
                                       └─ NavigateToPose action ──→  bt_navigator
                                                                       └─ planner_server (SmacHybrid)
                                                                       └─ controller_server (RPP)
                                                                          └─ /cmd_vel (Twist)
                                                                             └─ cmd_vel_bridge (LifecycleNode)
                                                                                ├─ /steering_angle (Float64)  ──→  rs485_chassis_bridge (LifecycleNode)
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
                                                                                                            rs485_chassis_receiver (LifecycleNode)
                                                                                                                      │
                                                                                                      /rs485/steering_angle
                                                                                                      /rs485/velocity
                                                                                                                      │
                                                                                                            vehicle_controller (LifecycleNode, remap)
                                                                                                                      │
                                                                                                      /forward_position_controller/commands
                                                                                                      /forward_velocity_controller/commands
                                                                                                                      │
                                                                                                            ros2_control → Gazebo
```

### 完整控制流（真实硬件）

```
openTCS
  └─ /goal_pose ──→  ...  ──→  /cmd_vel ──→  cmd_vel_bridge
                                                ├─ /steering_angle ──→  rs485_chassis_bridge
                                                └─ /velocity         ──→  (LifecycleNode)
                                                                          │
                                                                    编码485帧
                                                                    pyserial.write()
                                                                          │
                                                                    /dev/ttyUSB1 (物理 RS-485)
                                                                          │
                                                                    MCU 控制卡 → 电机
```

### 传感器数据流

```
Gazebo (sim) / 真实传感器 (real)
  ├─ /scan   (LaserScan)  ──→  AMCL, Nav2 costmap
  ├─ /odom   (Odometry)   ──→  ekf_filter_node, Nav2
  ├─ /imu    (Imu)        ──→  ekf_filter_node
  └─ /clock  (Clock)      ──→  全局时钟 (sim only)
```

### 关键话题汇总

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/cmd_vel` | `Twist` | Nav2 controller | `cmd_vel_bridge` | 导航速度指令 |
| `/steering_angle` | `Float64` | `cmd_vel_bridge` | `rs485_chassis_bridge` | 弧度，±0.5236 |
| `/velocity` | `Float64` | `cmd_vel_bridge` | `rs485_chassis_bridge` | m/s，±1.4 |
| `/rs485/steering_angle` | `Float64` | `rs485_chassis_receiver` | `vehicle_controller` | 经485解码后的转向 |
| `/rs485/velocity` | `Float64` | `rs485_chassis_receiver` | `vehicle_controller` | 经485解码后的速度 |
| `/system_health` | `DiagnosticArray` | `node_watchdog` | rqt_robot_monitor | 系统健康诊断 |
| `/system_health_summary` | `String (JSON)` | `node_watchdog` | 监控脚本 | JSON 格式健康摘要 |

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

## 自定义 LifecycleNode 详解

### rs485_chassis_bridge（LifecycleNode, 发送端）

订阅 `/steering_angle` 和 `/velocity`，编码为 RS-485 协议帧写入串口。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明参数、读取配置、初始化超时状态 |
| `on_activate` | 打开串口、创建 subscriptions + timer |
| `on_deactivate` | 关闭串口、销毁 timer |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/tmp/chassis_cmd` | 仿真虚拟串口；实物改为 `/dev/ttyUSB1` |
| `baudrate` | `115200` | 与控制卡一致 |
| `refresh_interval_ms` | `200` | 协议要求 50~300ms |
| `timeout_ms` | `500` | 协议要求 500ms 超时 |
| `max_steering_angle` | `0.5236` | 与 ackermann_params.yaml 一致 |
| `max_velocity` | `1.4` | 与 ackermann_params.yaml 一致 |

### rs485_chassis_receiver（LifecycleNode, 接收端, sim only）

从串口读取 RS-485 协议帧，解码后发布到 `/rs485/` 话题。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明参数、读取配置、初始化 buffer |
| `on_activate` | 打开串口、创建 publishers + 100Hz timer |
| `on_deactivate` | 关闭串口、销毁 timer |

### cmd_vel_bridge（LifecycleNode）

将 Nav2 的差速驱动 `Twist` 指令转换为阿克曼转向指令。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明 wheel_base 等参数、初始化状态 |
| `on_activate` | 创建 publishers + subscription + timer |
| `on_deactivate` | 销毁 timer |

### vehicle_controller（LifecycleNode, C++, sim only）

阿克曼转向几何解算，订阅 `/rs485/` 话题（remap），输出轮速和转向角指令。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明参数、计算 track_width 和 wheel_base |
| `on_activate` | 创建 publishers (LifecyclePublisher) + subscriptions + timer |
| `on_deactivate` | 销毁 timer、deactivate publishers |

### opentcs_vehicle_node（LifecycleNode）

openTCS ↔ Nav2 协议桥接。最复杂的节点（~970 行），包含目标管理、航向对齐状态机、电池仿真、安全监控等。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明所有参数、初始化状态变量（目标追踪、对齐状态机、电池、安全等） |
| `on_activate` | 创建所有 publishers/subscriptions/timers、ActionClient、TF listener |
| `on_deactivate` | 销毁所有 timers |

### route_graph_loader（LifecycleNode）

加载 Sidecar 发布的 GeoJSON 路线图到 Nav2 route_server。

| Lifecycle 阶段 | 操作 |
|----------------|------|
| `on_configure` | 声明参数、初始化加载状态 |
| `on_activate` | 创建 subscription (TRANSIENT_LOCAL QoS) + service client + publisher + timer |
| `on_deactivate` | 销毁 timer |

## 超时安全机制

系统有四层超时保护，从外到内依次生效：

```
┌─ 第1层: cmd_vel_bridge timeout ────────────────────────────┐
│  cmd_vel 超时 → 发送 (0.0, 0.0)                             │
│                                                              │
│  ┌─ 第2层: rs485_chassis_bridge (500ms) ─────────────────┐  │
│  │  超时 → 发送全 1500 中性帧                             │  │
│  │                                                        │  │
│  │  ┌─ 第3层: rs485_chassis_receiver (500ms) ──────────┐ │  │
│  │  │  无有效帧 → 发布 (0.0, 0.0)                       │ │  │
│  │  │                                                   │ │  │
│  │  │  ┌─ 第4层: vehicle_controller (800ms) ─────────┐ │ │  │
│  │  │  │  无话题消息 → 车轮速度归零，转向归零          │ │ │  │
│  │  │  └─────────────────────────────────────────────┘ │ │  │
│  │  └──────────────────────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 实物迁移指南

部署到实物底盘只需一条命令切换：

```bash
./scripts/launch/real_ackermann_nav.sh
# 内部传递 simulation:=False use_sim_time:=False
```

### 配置差异

| 参数 | 仿真值 | 实物值 |
|------|--------|--------|
| `simulation` | `True` | `False` |
| `use_sim_time` | `True` | `False` |
| `rs485_chassis_bridge.serial_port` | `/tmp/chassis_cmd` | `/dev/ttyUSB1` |

### 自动移除的节点（simulation=False）

| 节点 | 原因 |
|------|------|
| `socat` | 不需要虚拟串口 |
| `gz_sim` + `ros_gz_bridge` | 不需要 Gazebo |
| `spawn_robot` + `load_controllers` | 不需要 ros2_control（MCU 替代） |
| `rs485_chassis_receiver` | MCU 直接处理 485 帧 |
| `vehicle_controller` | MCU 直接驱动电机 |

### 保留运行的节点

| 节点 | 说明 |
|------|------|
| `rs485_chassis_bridge` | 直连物理 RS-485 适配器 |
| `cmd_vel_bridge` | 无变更 |
| `opentcs_vehicle_node` | 无变更 |
| `route_graph_loader` | 无变更 |
| Nav2 全套 | 无变更 |
| `lifecycle_manager_navigation` | 管理 10 个 Nav2 节点（bond 心跳） |
| `lifecycle_starter_localization` | 一次性启动 map_server + amcl |
| `lifecycle_starter_custom` | 一次性启动自定义节点 |
| `node_watchdog` | 健康监控 |

## 文件索引

### 启动文件

| 文件 | 用途 |
|------|------|
| `launch/sim_ackermann_rs485.launch.py` | 统一 launch（事件驱动 + sim/real + lifecycle_starter） |
| `launch/localization_custom.launch.py` | Nav2 定位（旧版，当前主 launch 未引用） |
| `launch/navigation_custom.launch.py` | Nav2 导航（lifecycle_manager_navigation 配置） |
| `scripts/launch/sim_ackermann_rs485.sh` | 仿真启动脚本 |
| `scripts/launch/real_ackermann_nav.sh` | 真实小车启动脚本 |

### 配置文件

| 文件 | 用途 |
|------|------|
| `config/nav2_params_opentcs.yaml` | Nav2 全参数 + 3 个 lifecycle_manager 配置 |
| `config/rs485_bridge.yaml` | RS-485 串口参数 |
| `config/ekf.yaml` | EKF 传感器融合参数 |
| `config/opentcs_vehicle.yaml` | openTCS 车辆状态参数 |
| `config/material_action.yaml` | 物料操作参数 |
| `config/watchdog.yaml` | Watchdog 监控规则 |

### 自定义节点源码

| 文件 | 类型 | Lifecycle |
|------|------|-----------|
| `src/lidar_slam_nodes/lidar_slam_nodes/cmd_vel_bridge.py` | Python | ✅ |
| `src/lidar_slam_nodes/lidar_slam_nodes/rs485_chassis_bridge.py` | Python | ✅ |
| `src/lidar_slam_nodes/lidar_slam_nodes/rs485_chassis_receiver.py` | Python | ✅ |
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py` | Python | ✅ |
| `src/lidar_slam_nodes/lidar_slam_nodes/route_graph_loader.py` | Python | ✅ |
| `src/ackermann_control/src/vehicle_controller.cpp` | C++ | ✅ |
| `src/lidar_slam_nodes/lidar_slam_nodes/lifecycle_starter.py` | Python | — (一次性启动器) |
| `src/lidar_slam_nodes/lidar_slam_nodes/node_watchdog.py` | Python | — |
| `src/lidar_slam_nodes/lidar_slam_nodes/wait_for_topic.py` | Python | — |
| `src/lidar_slam_nodes/lidar_slam_nodes/wait_for_service.py` | Python | — |
| `src/lidar_slam_nodes/lidar_slam_nodes/wait_for_tf.py` | Python | — |
| `src/lidar_slam_nodes/lidar_slam_nodes/load_controllers.py` | Python | — (一次性) |
