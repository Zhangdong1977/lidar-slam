# dispatch.sh --profile gazebo 命名空间参考表

> 本文档记录 `dispatch.sh --profile gazebo --namespace gazebo_1` 场景下所有节点、话题、Action 的完整带命名空间名称。
> 最后验证日期：2026-06-04（双车 `gazebo_1` + `gazebo_2`：TF、controller_manager、robot_description、route_graph、clock 命名空间隔离通过）

---

## 1. 启动命令

```bash
bash scripts/launch/dispatch.sh --discovery-address <sidecar_ip>  # 默认 gazebo 仿真，namespace=gazebo_1
bash scripts/launch/dispatch.sh --namespace gazebo_2 --skip-cleanup --no-gazebo --discovery-address <sidecar_ip>  # 第二辆车，复用已有 Gazebo
bash scripts/launch/dispatch.sh --no-rviz --discovery-address <sidecar_ip>  # 不启动 RViz
```

---

## 2. 启动链

```
T=0   [硬件层] socat → Gazebo → ros_gz_bridge → robot_state_publisher → static_tf → spawn_robot
          ↓ spawn_robot 退出
      wait_for_cm → load_controllers → wait_for_joints → rs485_chain (receiver + vehicle_controller + bridge)
T=0   [跨层] node_watchdog 启动
T=0   [感知链] wait_scan (等待 /gazebo_1/scan)
          ↓ wait_scan 退出
      wait_ekf_tf + ekf_filter_node 启动
          ↓ wait_ekf_tf 退出
      [定位层] localization.launch.py (map_server + amcl + lifecycle_manager_localization) + wait_localization_ready
          ↓ wait_localization_ready 退出
      [导航层] navigation.launch.py (10 个 Nav2 节点) + lifecycle_starter_custom + cmd_vel_bridge + opentcs_vehicle_node + wait_route
          ↓ wait_route 退出
      [应用层] route_graph_loader 启动
T+5s  [应用层] material_action_server 延迟启动
```

---

## 3. 节点列表

### 3.1 硬件层（gazebo_hardware.launch.py）

| # | 节点全名 | 包 | 可执行文件 | 说明 |
|---|---|---|---|---|
| 1 | `/gazebo_1/controller_manager` | gz_ros2_control | (Gazebo 插件) | ros2_control 控制器管理器，已命名空间化 |
| 2 | `/gazebo_1/gz_ros_control` | gz_ros2_control | (Gazebo 插件) | Gazebo ROS2 控制节点 |
| 3 | `/gazebo_1/forward_position_controller` | forward_command_controller | (控制器) | 转向关节位置控制 |
| 4 | `/gazebo_1/forward_velocity_controller` | forward_command_controller | (控制器) | 后轮速度控制 |
| 5 | `/gazebo_1/joint_state_broadcaster` | joint_state_broadcaster | (控制器) | 关节状态广播 |
| 6 | `/gazebo_1/ros_gz_bridge` | ros_gz_bridge | parameter_bridge | Gazebo↔ROS2 桥接 |
| 7 | `/gazebo_1/robot_state_publisher` | robot_state_publisher | robot_state_publisher | URDF→TF 发布 |
| 8 | `/gazebo_1/static_transform_publisher_<random>` | tf2_ros | static_transform_publisher | body_link→lidar 静态 TF |
| 9 | `/gazebo_1/rs485_chassis_receiver` | lidar_slam_nodes | rs485_chassis_receiver | 虚拟串口读取 |
| 10 | `/gazebo_1/vehicle_controller` | ackermann_control | vehicle_controller | 阿克曼转向控制 |
| 11 | `/gazebo_1/rs485_chassis_bridge` | lidar_slam_nodes | rs485_chassis_bridge | 虚拟串口写入（延迟 1s） |

> **辅助节点（已退出）:** `/load_controllers_client`（根 NS）, `/gazebo_1/wait_for_cm`, `/gazebo_1/wait_for_joints`

### 3.2 感知层（nav_main.launch.py）

| # | 节点全名 | 包 | 说明 |
|---|---|---|---|
| 12 | `/gazebo_1/ekf_filter_node` | robot_localization | EKF 里程计+IMU 融合 |

### 3.3 定位层（localization.launch.py）

| # | 节点全名 | 包 | 说明 |
|---|---|---|---|
| 13 | `/gazebo_1/map_server` | nav2_map_server | 静态地图发布 |
| 14 | `/gazebo_1/amcl` | nav2_amcl | 自适应蒙特卡洛定位 |
| 15 | `/gazebo_1/lifecycle_manager_localization` | nav2_lifecycle_manager | map_server + amcl 生命周期管理 |

### 3.4 导航层（navigation.launch.py）

| # | 节点全名 | 包 | 说明 |
|---|---|---|---|
| 16 | `/gazebo_1/controller_server` | nav2_controller | 路径跟踪控制器 |
| 17 | `/gazebo_1/smoother_server` | nav2_smoother | 路径平滑 |
| 18 | `/gazebo_1/planner_server` | nav2_planner | 全局路径规划 |
| 19 | `/gazebo_1/route_server` | nav2_route | 路线图路由 |
| 20 | `/gazebo_1/behavior_server` | nav2_behaviors | 恢复行为 (spin/backup/wait) |
| 21 | `/gazebo_1/bt_navigator` | nav2_bt_navigator | 行为树导航协调器 |
| 22 | `/gazebo_1/bt_navigator_navigate_to_pose_rclcpp_node` | nav2_bt_navigator | navigate_to_pose action 子节点 |
| 23 | `/gazebo_1/bt_navigator_navigate_through_poses_rclcpp_node` | nav2_bt_navigator | navigate_through_poses action 子节点 |
| 24 | `/gazebo_1/waypoint_follower` | nav2_waypoint_follower | 航点跟踪 |
| 25 | `/gazebo_1/velocity_smoother` | nav2_velocity_smoother | 速度平滑 |
| 26 | `/gazebo_1/collision_monitor` | nav2_collision_monitor | 碰撞监控 |
| 27 | `/gazebo_1/docking_server` | opennav_docking | 自动泊车 |
| 28 | `/gazebo_1/lifecycle_manager_navigation` | nav2_lifecycle_manager | Nav2 节点生命周期管理 |
| 29 | `/gazebo_1/local_costmap/local_costmap` | nav2_costmap_2d | 局部代价地图子节点 |
| 30 | `/gazebo_1/global_costmap/global_costmap` | nav2_costmap_2d | 全局代价地图子节点 |

### 3.5 应用层（nav_main.launch.py）

| # | 节点全名 | 包 | 说明 |
|---|---|---|---|
| 31 | `/gazebo_1/lifecycle_starter_custom` | lidar_slam_nodes | 应用层 LifecycleNode 激活 |
| 32 | `/gazebo_1/cmd_vel_bridge` | lidar_slam_nodes | cmd_vel 桥接（Gazebo/RS-485） |
| 33 | `/gazebo_1/opentcs_vehicle_node` | lidar_slam_nodes | openTCS 车辆通信节点 |
| 34 | `/gazebo_1/route_graph_loader` | lidar_slam_nodes | 路线图加载器 |
| 35 | `/gazebo_1/material_action_server` | jvs_agv_material_actions | 物料动作服务端 |

### 3.6 跨层服务

| # | 节点全名 | 包 | 说明 |
|---|---|---|---|
| 36 | `/gazebo_1/node_watchdog` | lidar_slam_nodes | 节点健康监控 |

### 3.7 外部节点

| # | 节点全名 | 说明 |
|---|---|---|
| 37 | `/jvs_opentcs_ros2_sidecar` | openTCS Java sidecar（外部进程，不在本仓库内） |

---

## 4. 话题列表

### 4.1 全局话题

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | `/gazebo_1/ros_gz_bridge` | 仿真时钟；只由启动 Gazebo 的实例发布 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | lifecycle_manager、EKF、controller_manager 等 | 全局诊断聚合 |
| `/parameter_events` | rcl_interfaces/ParameterEvent | ROS2 节点 | 参数事件 |
| `/rosout` | rcl_interfaces/Log | ROS2 节点 | 日志 |

> 已验证：双车运行时没有全局 `/robot_description`、`/tf`、`/tf_static`、`/route_graph`、`/goal_pose`。这些话题均使用 namespace 隔离。

### 4.2 传感器数据（Gazebo 桥接）

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/gazebo_1/scan` | sensor_msgs/LaserScan | ros_gz_bridge | amcl, collision_monitor 等 |
| `/gazebo_1/odom` | nav_msgs/Odometry | ros_gz_bridge | ekf_filter_node |
| `/gazebo_1/imu` | sensor_msgs/Imu | ros_gz_bridge | ekf_filter_node |
| `/gazebo_1/robot_description` | std_msgs/String | robot_state_publisher | `/gazebo_1/controller_manager` |

### 4.3 TF（命名空间化）

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/gazebo_1/tf` | tf2_msgs/TFMessage | robot_state_publisher, ekf, amcl 等 | 动态 TF |
| `/gazebo_1/tf_static` | tf2_msgs/TFMessage | robot_state_publisher, static_transform_publisher | 静态 TF |

### 4.4 关节与控制

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/joint_states` | sensor_msgs/JointState | joint_state_broadcaster 发布 |
| `/gazebo_1/dynamic_joint_states` | control_msgs/DynamicJointState | ros2_control 内部 |
| `/gazebo_1/forward_position_controller/commands` | std_msgs/Float64MultiArray | 转向指令 |
| `/gazebo_1/forward_velocity_controller/commands` | std_msgs/Float64MultiArray | 速度指令 |
| `/gazebo_1/steering_angle` | std_msgs/Float64 | 转向角反馈 |
| `/gazebo_1/velocity` | std_msgs/Float64 | 速度反馈 |

### 4.5 EKF 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/odometry/filtered` | nav_msgs/Odometry | EKF 融合后的里程计 |

### 4.6 cmd_vel 数据流

```
controller_server ──cmd_vel_nav──> velocity_smoother
behavior_server   ──cmd_vel_nav──>       │
                                      cmd_vel_smoothed
                                       ↓
                                collision_monitor
                                       ↓
                                   cmd_vel ──> cmd_vel_bridge ──> vehicle_controller
```

| 话题 | 类型 | 发布者 → 订阅者 |
|---|---|---|
| `/gazebo_1/cmd_vel_nav` | geometry_msgs/Twist | controller_server/behavior_server → velocity_smoother |
| `/gazebo_1/cmd_vel_smoothed` | geometry_msgs/Twist | velocity_smoother → collision_monitor |
| `/gazebo_1/cmd_vel` | geometry_msgs/Twist | collision_monitor → cmd_vel_bridge |
| `/gazebo_1/cmd_vel_teleop` | geometry_msgs/Twist | 键盘/手柄遥控输入 |

### 4.7 定位话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/map` | nav_msgs/OccupancyGrid | map_server 发布的静态地图 |
| `/gazebo_1/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL 定位结果 |
| `/gazebo_1/particle_cloud` | geometry_msgs/ParticleCloud | AMCL 粒子云 |
| `/gazebo_1/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 初始位姿设置（rviz） |
| `/gazebo_1/set_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL 位姿重置 |

### 4.8 导航规划话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/plan` | nav_msgs/Path | 全局规划路径 |
| `/gazebo_1/plan_smoothed` | nav_msgs/Path | 平滑后的路径 |
| `/gazebo_1/received_global_plan` | nav_msgs/Path | 接收到的全局路径 |
| `/gazebo_1/lookahead_point` | geometry_msgs/PointStamped | 控制器前瞻点 |
| `/gazebo_1/curvature_lookahead_point` | geometry_msgs/PointStamped | 曲率前瞻点 |
| `/gazebo_1/speed_limit` | nav2_msgs/SpeedLimit | 速度限制 |

### 4.9 代价地图话题

| 话题前缀 | 说明 |
|---|---|
| `/gazebo_1/local_costmap/costmap*` | 局部代价地图（costmap, raw, updates） |
| `/gazebo_1/local_costmap/obstacle_layer*` | 局部障碍物层 |
| `/gazebo_1/local_costmap/published_footprint` | 局部足迹 |
| `/gazebo_1/global_costmap/costmap*` | 全局代价地图 |
| `/gazebo_1/global_costmap/static_layer*` | 全局静态层 |
| `/gazebo_1/global_costmap/obstacle_layer*` | 全局障碍物层 |
| `/gazebo_1/downsampled_costmap*` | 降采样代价地图 |

### 4.10 碰撞监控

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/collision_monitor_state` | nav2_msgs/CollisionMonitorState | 碰撞监控状态 |
| `/gazebo_1/collision_monitor/collision_points_marker` | visualization_msgs/MarkerArray | 碰撞点可视化 |

### 4.11 应用层话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/goal_pose` | geometry_msgs/PoseStamped | opentcs_vehicle 发布的导航目标 |
| `/gazebo_1/robot_state` | std_msgs/String | 车辆运行状态 JSON（29+ 字段） |
| `/gazebo_1/battery_state` | sensor_msgs/BatteryState | 电池状态（仿真） |
| `/gazebo_1/route_graph_json` | std_msgs/String | 路线图 GeoJSON 输入；外部 sidecar 发布，route_graph_loader 订阅 |
| `/gazebo_1/route_graph` | visualization_msgs/MarkerArray | Nav2 route_server 发布的路线图可视化 |
| `/gazebo_1/route_graph/markers` | visualization_msgs/MarkerArray | route_graph_loader 发布的路线图可视化 |
| `/gazebo_1/system_health` | diagnostic_msgs/DiagnosticArray | watchdog 系统健康 |
| `/gazebo_1/system_health_summary` | std_msgs/String | 系统健康摘要 |
| `/gazebo_1/preempt_teleop` | std_msgs/Empty | 遥控抢占信号 |
| `/gazebo_1/is_rotating_to_heading` | std_msgs/Bool | 旋转到目标航向标志 |

### 4.12 openTCS Sidecar 话题（外部组件，使用 vehicle_name）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/{vehicle_name}/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | sidecar 订阅的定位数据 |
| `/{vehicle_name}/goal_pose` | geometry_msgs/PoseStamped | sidecar 下发的导航目标 |
| `/{vehicle_name}/robot_state` | std_msgs/String | sidecar 订阅的车辆状态 |
| `/{vehicle_name}/battery_state` | sensor_msgs/BatteryState | sidecar 订阅的电池状态 |
| `/{vehicle_name}/initialpose` | geometry_msgs/PoseWithCovarianceStamped | sidecar 下发的初始位姿 |
| `/{vehicle_name}/route_graph_json` | std_msgs/String | sidecar 下发的 GeoJSON 路线图 |

> `dispatch.sh` 默认将 `vehicle_name` 设置为运行时 namespace，因此 `--namespace gazebo_1` 时 sidecar 话题为 `/gazebo_1/...`。

### 4.13 生命周期/transition_event 话题（内部使用，省略详细列表）

所有 LifecycleNode 会发布 `{node_name}/transition_event` 话题。以下节点有此类话题：
`amcl, behavior_server, bt_navigator, cmd_vel_bridge, collision_monitor, controller_server, docking_server, map_server, opentcs_vehicle_node, planner_server, route_graph_loader, route_server, rs485_chassis_bridge, rs485_chassis_receiver, smoother_server, vehicle_controller, velocity_smoother, waypoint_follower, local_costmap/local_costmap, global_costmap/global_costmap, joint_state_broadcaster`

---

## 5. Action 列表

### 5.1 Nav2 导航 Action

| Action 全名 | 类型 | Server 节点 |
|---|---|---|
| `/gazebo_1/navigate_to_pose` | nav2_msgs/action/NavigateToPose | bt_navigator |
| `/gazebo_1/navigate_through_poses` | nav2_msgs/action/NavigateThroughPoses | bt_navigator |
| `/gazebo_1/follow_path` | nav2_msgs/action/FollowPath | controller_server |
| `/gazebo_1/compute_path_to_pose` | nav2_msgs/action/ComputePathToPose | planner_server |
| `/gazebo_1/compute_path_through_poses` | nav2_msgs/action/ComputePathThroughPoses | planner_server |
| `/gazebo_1/smooth_path` | nav2_msgs/action/SmoothPath | smoother_server |
| `/gazebo_1/follow_waypoints` | nav2_msgs/action/FollowWaypoints | waypoint_follower |
| `/gazebo_1/assisted_teleop` | nav2_msgs/action/AssistedTeleop | behavior_server |
| `/gazebo_1/spin` | nav2_msgs/action/Spin | behavior_server |
| `/gazebo_1/backup` | nav2_msgs/action/BackUp | behavior_server |
| `/gazebo_1/drive_on_heading` | nav2_msgs/action/DriveOnHeading | behavior_server |
| `/gazebo_1/wait` | nav2_msgs/action/Wait | behavior_server |

### 5.2 路线规划 Action

| Action 全名 | 类型 | Server 节点 |
|---|---|---|
| `/gazebo_1/compute_route` | nav2_msgs/action/ComputeRoute | route_server |
| `/gazebo_1/compute_and_track_route` | nav2_msgs/action/ComputeAndTrackRoute | route_server |

### 5.3 泊车 Action

| Action 全名 | 类型 | Server 节点 |
|---|---|---|
| `/gazebo_1/dock_robot` | nav2_msgs/action/DockRobot | docking_server |
| `/gazebo_1/undock_robot` | nav2_msgs/action/UndockRobot | docking_server |

### 5.4 物料 Action

| Action 全名 | 类型 | Server 节点 |
|---|---|---|
| `/gazebo_1/load_materials` | jvs_agv_material_msgs/action/LoadMaterials | material_action_server |
| `/gazebo_1/unload_materials` | jvs_agv_material_msgs/action/UnloadMaterials | material_action_server |

### 5.5 外部 Action

| Action 全名 | 类型 | 说明 |
|---|---|---|
| `/{vehicle_name}/navigate_to_pose` | — | openTCS sidecar 桥接（外部） |
| `/gazebo_1/follow_gps_waypoints` | nav2_msgs/action/FollowGPSWaypoints | Nav2 内置，本场景未使用 |

---

## 6. TF 树

```
map ──(AMCL)──> odom ──(EKF)──> body_link ──(static TF)──> gazebo_1/body_link/lidar
                                  │
                                  ├── front_left_steering_joint ──> front_left_steering_link
                                  │     └── front_left_wheel_joint ──> front_left_wheel_link
                                  ├── front_right_steering_joint ──> front_right_steering_link
                                  │     └── front_right_wheel_joint ──> front_right_wheel_link
                                  ├── rear_left_wheel_joint ──> rear_left_wheel_link
                                  └── rear_right_wheel_joint ──> rear_right_wheel_link
```

---

## 7. 关键 Service

| Service 全名 | 类型 | 说明 |
|---|---|---|
| `/gazebo_1/lifecycle_manager_localization/manage_nodes` | lifecycle_manager_msgs/ManageLifecycleNodes | 定位层生命周期管理 |
| `/gazebo_1/lifecycle_manager_navigation/manage_nodes` | lifecycle_manager_msgs/ManageLifecycleNodes | 导航层生命周期管理 |
| `/gazebo_1/lifecycle_starter_custom/...` | — | 应用层 LifecycleNode 激活 |
| `/gazebo_1/controller_manager/load_controller` | controller_manager_msgs/LoadController | 加载控制器 |
| `/gazebo_1/controller_manager/switch_controller` | controller_manager_msgs/SwitchController | 切换控制器 |
| `/gazebo_1/controller_manager/list_controllers` | controller_manager_msgs/ListControllers | 列出控制器 |
| `/gazebo_1/route_server/set_route_graph` | nav2_msgs/SetRouteGraph | 设置路线图 |
| `/gazebo_1/reinitialize_global_localization` | std_srvs/Empty | AMCL 全局重定位 |
| `/gazebo_1/collision_monitor/toggle` | std_srvs/Empty | 碰撞监控开关 |

---

## 8. 多车扩展

使用不同 namespace 启动多辆车：

```bash
# 车辆 1
bash scripts/launch/dispatch.sh --namespace gazebo_1 --discovery-address <sidecar_ip>

# 车辆 2（另一个终端）
bash scripts/launch/dispatch.sh --namespace gazebo_2 --skip-cleanup --no-gazebo --discovery-address <sidecar_ip>
```

所有节点、话题、Action 会自动使用对应 namespace 前缀。

- `/clock` 为全局仿真时钟，仅由启动 Gazebo 的 `ros_gz_bridge` 发布；第二辆车使用 `--no-gazebo` 时不会重复桥接 `/clock`。
- `/robot_description` 已按车辆隔离为 `/<namespace>/robot_description`，每个 `controller_manager` 只订阅本车 URDF。
- GeoJSON 路线图输入使用 `/<namespace>/route_graph_json`；`/<namespace>/route_graph` 保留给 Nav2 `route_server` 的 MarkerArray。

> 外部 sidecar (`jvs_opentcs_ros2_sidecar`) 使用 `vehicle_name` 前缀；`dispatch.sh` 默认令 `vehicle_name == namespace`。
