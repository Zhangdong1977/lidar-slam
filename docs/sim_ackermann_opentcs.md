# sim_ackermann_opentcs — 导航 + openTCS 桥接架构

## 功能概述

`sim_ackermann_opentcs.sh` 是一键启动 **Ackermann 阿克曼转向机器人仿真导航 + openTCS 车队管理桥接** 的入口脚本。它完成以下工作：

1. 设置 X11 显示环境（GUI 需要）
2. 激活 conda 环境 + ROS2 Jazzy 环境（使用 FastDDS）
3. 清理残留的 Gazebo/ROS2 僵尸进程
4. 调用 `sim_ackermann_opentcs.launch.py`，启动导航定位管线及 openTCS 桥接
5. 日志直接输出到终端（未重定向到文件）

**与 explore 场景的核心区别**：explore 场景用于自主探索建图（slam_toolbox 在线建图），opentcs 场景用于**已知地图上的定点导航**（AMCL 定位 + 已有地图），并通过 openTCS 桥接节点接收外部调度系统的导航指令。

## 系统架构

```
时间轴(s)  节点/组件
─────────────────────────────────────────────────────
  0s     Gazebo 仿真器 + ros_gz_bridge + 静态TF(lidar)
  2s     ackermann_control (URDF+控制器+生成机器人)
  5s     EKF 融合 (odom+imu → TF)
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
| 4 | `vehicle_controller` | `ackermann_control` | 阿克曼转向几何解算（中心角→左右轮） |
| 5 | `ekf_filter_node` | `robot_localization` | EKF 融合 /odom + /imu → 发布 odom→body_link TF |
| 6 | `map_server` | `nav2_map_server` | 加载已知地图 `auto_exploration_map.yaml` 到 /map |
| 7 | `amcl` | `nav2_amcl` | 自适应蒙特卡洛定位，发布 map→odom TF |
| 8 | `controller_server` | `nav2_controller` | RegulatedPurePursuit 路径跟踪 |
| 9 | `planner_server` | `nav2_planner` | NavfnPlanner (Dijkstra/A*) |
| 10 | `bt_navigator` | `nav2_bt_navigator` | 行为树导航协调器 |
| 11 | `behavior_server` | `nav2_behaviors` | 旋转/后退/等待等恢复行为 |
| 12 | `smoother_server` | `nav2_smoother` | 路径平滑 |
| 13 | `velocity_smoother` | `nav2_velocity_smoother` | 速度平滑 |
| 14 | `collision_monitor` | `nav2_collision_monitor` | 碰撞监控（默认关闭） |
| 15 | `cmd_vel_bridge` | `lidar_slam_nodes` | Nav2 Twist → 阿克曼 steering_angle/velocity |
| 16 | `opentcs_nav2_bridge` | `lidar_slam_nodes` | openTCS ↔ Nav2 协议桥接 |
| 17 | `rviz2` | `rviz2` | 可视化 |

## 话题设计

### 传感器数据流

```
Gazebo
  ├─ /scan   (LaserScan)  ──→  AMCL, Nav2 costmap
  ├─ /odom   (Odometry)   ──→  ekf_filter_node, Nav2
  ├─ /imu    (Imu)        ──→  ekf_filter_node
  ├─ /tf     (TFMessage)  ──→  TF 系统（Gazebo 模型位姿）
  └─ /clock  (Clock)      ──→  全局时钟
```

### 导航控制流

```
openTCS (外部调度)
  └─ /goal_pose (PoseStamped)  ──→  opentcs_nav2_bridge
                                       └─ NavigateToPose action ──→  bt_navigator
                                                                       └─ planner_server (NavfnPlanner)
                                                                       └─ controller_server (RPP)
                                                                          └─ /cmd_vel (Twist)
                                                                             └─ cmd_vel_bridge
                                                                                ├─ /steering_angle (Float64)  ──→  vehicle_controller
                                                                                └─ /velocity (Float64)        ──→  vehicle_controller
                                                                                    └─ Gazebo ros2_control
```

### 位置上报流

```
TF 树 (map → body_link)
  └─ opentcs_nav2_bridge 定时查询 TF
      └─ /amcl_pose (PoseWithCovarianceStamped)  ──→  openTCS
```

### 关键话题汇总

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/scan` | `LaserScan` | ros_gz_bridge (来自 Gazebo) | AMCL, Nav2 costmap |
| `/odom` | `Odometry` | ros_gz_bridge (来自 Gazebo) | ekf_filter_node, Nav2 |
| `/imu` | `Imu` | ros_gz_bridge (来自 Gazebo) | ekf_filter_node |
| `/tf` | `TFMessage` | ros_gz_bridge (来自 Gazebo) | TF 系统 |
| `/map` | `OccupancyGrid` | map_server | Nav2 static_layer, AMCL |
| `/goal_pose` | `PoseStamped` | openTCS | `opentcs_nav2_bridge` |
| `/amcl_pose` | `PoseWithCovarianceStamped` | `opentcs_nav2_bridge` | openTCS |
| `/cmd_vel` | `Twist` | Nav2 controller | `cmd_vel_bridge` |
| `/steering_angle` | `Float64` | `cmd_vel_bridge` | `vehicle_controller` |
| `/velocity` | `Float64` | `cmd_vel_bridge` | `vehicle_controller` |

## TF 树

```
map
 └─ odom          (由 AMCL 发布，粒子滤波定位)
     └─ body_link  (由 ekf_filter_node 发布，融合 odom+imu)
         ├─ body_link/lidar           (静态 TF, z=0.22)
         ├─ front_left_steering_link  (由 robot_state_publisher 发布)
         │   └─ front_left_wheel_link
         ├─ front_right_steering_link
         │   └─ front_right_wheel_link
         ├─ rear_left_wheel_link
         └─ rear_right_wheel_link
```

**TF 链路说明**：

- **`map → odom`**：由 **AMCL** 粒子滤波器通过激光扫描与已知地图匹配计算发布（与 explore 场景的 slam_toolbox 不同）
- **`odom → body_link`**：由 `robot_localization` 的 EKF 融合轮式里程计（/odom）和 IMU（/imu）发布，频率 100Hz
- **`body_link → body_link/lidar`**：静态变换，LiDAR 安装在机体上方 22cm
- **`body_link → 轮/转向关节`**：由 `robot_state_publisher` 根据 URDF 关节状态发布

## 自定义节点详解

### cmd_vel_bridge

将 Nav2 的差速驱动 `Twist` 指令转换为阿克曼转向指令：

- `steering_angle = atan(wheel_base × ω / v)`
- `velocity = v`
- 20Hz 定时发布，0.5s 超时自动归零
- 纯旋转时提供最小蠕行速度（阿克曼车需要前进才能转向）

### opentcs_nav2_bridge

**openTCS ↔ Nav2 协议桥接节点**，实现两个方向的数据转换：

**指令方向（openTCS → Nav2）**：
- 订阅 `/goal_pose`（`PoseStamped`），来自 openTCS 调度系统
- 转换为 Nav2 的 `NavigateToPose` action goal 发送
- 支持目标抢占：新目标到达时自动取消前一个未完成的目标
- 等待 Nav2 action server 就绪（最长 2s）

**位置方向（TF → openTCS）**：
- 定时查询 `map → body_link` TF 变换（默认 10Hz）
- 转换为 `PoseWithCovarianceStamped` 消息发布到 `/amcl_pose`
- 协方差为固定值（0.01），非概率性 AMCL 估计

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_pose_topic` | `/goal_pose` | openTCS 目标点话题 |
| `amcl_pose_topic` | `/amcl_pose` | 机器人位姿上报话题 |
| `nav_action_name` | `/navigate_to_pose` | Nav2 action 名称 |
| `target_frame` | `map` | TF 目标帧 |
| `source_frame` | `body_link` | TF 源帧 |
| `pose_publish_rate` | `10.0` | 位姿上报频率 (Hz) |

## Nav2 关键配置特点

与 explore 场景相比，opentcs 场景的导航参数有显著调整：

| 配置项 | explore 场景 | opentcs 场景 | 说明 |
|--------|-------------|-------------|------|
| **定位方式** | slam_toolbox (在线SLAM) | AMCL (粒子滤波) | opentcs 在已知地图上定位 |
| **地图来源** | slam_toolbox 实时构建 | map_server 加载静态地图 | `auto_exploration_map.yaml` |
| **规划器** | SmacPlannerHybrid (Reeds-Shepp) | NavfnPlanner (Dijkstra) | 已知地图无需考虑运动学约束 |
| **规划频率** | 5 Hz | 20 Hz | 已知地图规划更频繁 |
| **控制器速度** | 0.25 m/s | 0.5 m/s | 导航场景允许更快速度 |
| **前瞻距离** | 0.5~3.0 m | 0.4~1.2 m | 导航路径更确定，前瞻更短 |
| **目标容差** | xy=0.35m, yaw=360° | xy=0.25m, yaw=0.25rad | 导航需要精确到位和朝向 |
| **全局代价地图 obstacle** | raytrace 8.0m | raytrace 3.0m | 已知地图障碍检测范围更保守 |
| **transform_tolerance** | 1.0s (全局) | 10.0s (全局) | 容忍更大 TF 延迟 |
| **collision_monitor** | 启用 | 关闭 (`enabled: False`) | 导航场景暂不启用碰撞监控 |
| **/scan 话题** | 经过 scan_range_filter 过滤 | 直接从 Gazebo 桥接 | 无需 NaN 过滤 |
| **/tf 桥接** | 无 | 有 | 传递 Gazebo 模型位姿 |

## 与 explore 场景的完整对比

| 维度 | explore (自主探索建图) | opentcs (导航+调度) |
|------|----------------------|-------------------|
| 目的 | 未知环境自主探索并建图 | 已知地图上定点导航 |
| 定位 | slam_toolbox（同时定位与建图） | AMCL（基于已知地图定位） |
| 地图 | 实时构建 | 加载 `auto_exploration_map.yaml` |
| 探索 | frontier_explorer（前沿探索） | 无（由 openTCS 调度） |
| 目标来源 | 自主选择前沿点 | openTCS 下发 `/goal_pose` |
| 位姿上报 | 无 | `/amcl_pose` → openTCS |
| 规划器 | SmacPlannerHybrid (运动学约束) | NavfnPlanner (快速 Dijkstra) |
| DDS | 默认 | FastDDS (`rmw_fastrtps_cpp`) |
| 日志 | 重定向到文件 | 直接输出终端 |
