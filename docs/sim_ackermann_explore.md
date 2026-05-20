# sim_ackermann_explore — 自主探索建图架构

## 功能概述

`sim_ackermann_explore.sh` 是一键启动 **Ackermann 阿克曼转向机器人仿真自主探索建图** 的入口脚本。它完成以下工作：

1. 设置 X11 显示环境（GUI 需要）
2. 激活 conda 环境 + ROS2 Jazzy 环境
3. 清理残留的 Gazebo/ROS2 僵尸进程
4. 调用 `sim_ackermann_explore.launch.py`，启动完整的仿真探索管线
5. 所有输出重定向到 `log/explore_*.log` 日志文件

## 系统架构

整个管线分为 10 个阶段，按时间顺序依次启动：

```
时间轴(s)  节点/组件
─────────────────────────────────────────────────────
  0s     Gazebo 仿真器 + ros_gz_bridge + 静态TF(lidar)
  2s     ackermann_control (URDF+控制器+生成机器人)
  2s     scan_range_filter (激光扫描过滤)
  5s     EKF 融合 (odom+imu → TF)
  5s     slam_toolbox (在线SLAM)
 25s     Nav2 导航栈 (规划+控制+代价地图)
  0s     cmd_vel_bridge (Twist→阿克曼)
  0s     RViz2 可视化
 30s     wait_for_tf (等待TF树就绪)
 TF就绪   frontier_explorer (自主探索)
```

**设计思想**：`wait_for_tf` 作为同步屏障，确保 `map→odom→body_link` 整条 TF 链路建立后，才启动探索节点，避免探索器在定位未就绪时发出无效导航目标。

## ROS2 节点列表

| # | 节点名 | 包 | 功能 |
|---|--------|---|------|
| 1 | `gz_sim` | `ros_gz_sim` | Gazebo Harmonic 仿真器，加载 factory.sdf 世界 |
| 2 | `parameter_bridge` | `ros_gz_bridge` | Gazebo↔ROS2 消息桥接 |
| 3 | `scan_range_filter` | `lidar_slam_nodes` | 将 LiDAR 的 NaN/inf 替换为有效值，清洗 frame_id |
| 4 | `robot_state_publisher` | `robot_state_publisher` | 发布 URDF 和关节 TF |
| 5 | `vehicle_controller` | `ackermann_control` | 阿克曼转向几何解算（中心角→左右轮） |
| 6 | `ekf_filter_node` | `robot_localization` | EKF 融合 /odom + /imu → 发布 odom→body_link TF |
| 7 | `slam_toolbox` | `slam_toolbox` | 在线异步 SLAM，发布 /map 和 map→odom TF |
| 8 | `controller_server` | `nav2_controller` | RegulatedPurePursuit 路径跟踪 |
| 9 | `planner_server` | `nav2_planner` | SmacPlannerHybrid (Reeds-Shepp 运动模型) |
| 10 | `bt_navigator` | `nav2_bt_navigator` | 行为树导航协调器 |
| 11 | `behavior_server` | `nav2_behaviors` | 旋转/后退/等待等恢复行为 |
| 12 | `smoother_server` | `nav2_smoother` | 路径平滑 |
| 13 | `velocity_smoother` | `nav2_velocity_smoother` | 速度平滑 |
| 14 | `collision_monitor` | `nav2_collision_monitor` | 碰撞监控 |
| 15 | `cmd_vel_bridge` | `lidar_slam_nodes` | Nav2 Twist → 阿克曼 steering_angle/velocity |
| 16 | `wait_for_tf` | `lidar_slam_nodes` | 等待 TF 就绪后退出（同步屏障） |
| 17 | `frontier_explorer` | `lidar_slam_nodes` | 基于前沿的自主探索（阿克曼航向感知） |
| 18 | `rviz2` | `rviz2` | 可视化 |

## 话题设计

### 传感器数据流

```
Gazebo
  ├─ /scan_raw   (LaserScan)  ──→  scan_range_filter  ──→  /scan  (LaserScan)
  ├─ /odom       (Odometry)   ──→  ekf_filter_node
  ├─ /imu        (Imu)        ──→  ekf_filter_node
  └─ /clock      (Clock)      ──→  全局时钟
```

### 导航控制流

```
frontier_explorer
  └─ NavigateToPose action ──→  bt_navigator
                                   └─ planner_server (SmacPlannerHybrid)
                                   └─ controller_server (RegulatedPurePursuit)
                                      └─ /cmd_vel (Twist)
                                         └─ cmd_vel_bridge
                                            ├─ /steering_angle (Float64)  ──→  vehicle_controller
                                            └─ /velocity (Float64)        ──→  vehicle_controller
                                                └─ /forward_position_controller/commands
                                                └─ /forward_velocity_controller/commands
                                                    └─ Gazebo ros2_control
```

### 关键话题汇总

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/scan_raw` | `LaserScan` | Gazebo | `scan_range_filter` |
| `/scan` | `LaserScan` | `scan_range_filter` | slam_toolbox, Nav2 costmap |
| `/odom` | `Odometry` | Gazebo | `ekf_filter_node`, Nav2 |
| `/imu` | `Imu` | Gazebo | `ekf_filter_node` |
| `/map` | `OccupancyGrid` | slam_toolbox | `frontier_explorer`, Nav2 static_layer |
| `/cmd_vel` | `Twist` | Nav2 controller | `cmd_vel_bridge` |
| `/steering_angle` | `Float64` | `cmd_vel_bridge` | `vehicle_controller` |
| `/velocity` | `Float64` | `cmd_vel_bridge` | `vehicle_controller` |
| `/frontier_markers` | `MarkerArray` | `frontier_explorer` | RViz2 |

## TF 树

```
map
 └─ odom          (由 slam_toolbox 发布，scan-matching 定位)
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

- **`map → odom`**：由 `slam_toolbox` 通过扫描匹配计算，频率 50Hz（`transform_publish_period: 0.02`）
- **`odom → body_link`**：由 `robot_localization` 的 EKF 融合轮式里程计（/odom）和 IMU（/imu）发布，频率 100Hz
- **`body_link → body_link/lidar`**：静态变换，LiDAR 安装在机体上方 22cm
- **`body_link → 轮/转向关节`**：由 `robot_state_publisher` 根据 URDF 关节状态发布

## 自定义节点详解

### scan_range_filter

将 Gazebo 产出的原始激光数据中的 NaN 替换为 `inf`，并清洗 Gazebo 命名空间前缀（`ackermann_robot/body_link/lidar` → `body_link/lidar`），使 slam_toolbox 能正确追踪自由空间。

### cmd_vel_bridge

将 Nav2 的差速驱动 `Twist` 指令转换为阿克曼转向指令：

- `steering_angle = atan(wheel_base × ω / v)`
- `velocity = v`
- 20Hz 定时发布，0.5s 超时自动归零
- 纯旋转时提供最小蠕行速度（阿克曼车需要前进才能转向）

### wait_for_tf

同步屏障节点。阻塞等待 `body_link → map` 变换可用（最长 120s），就绪后以 exit code 0 退出，触发 `frontier_explorer` 启动。

### frontier_explorer

核心探索节点，状态机驱动（IDLE → SELECTING → NAVIGATING → COMPLETED）：

- 检测前沿（自由格子邻接未知格子）→ BFS 聚类 → 评分（大小/距离/航向）
- 阿克曼可行性过滤（最大航向差 2.5rad，目标距离 1~35m）
- 卡住检测（超时 180s + 位移 0.5m 双重判定）
- 降级策略：放宽航向约束 → 远距离大目标点 → 重定位至前沿质心
- 完成后自动保存地图到 `maps/auto_exploration_map`

## Nav2 关键配置特点

- **规划器**：`SmacPlannerHybrid`，Reeds-Shepp 运动模型，最小转弯半径 1.0m，支持倒车（`allow_reverse_expansion: true`）
- **控制器**：`RegulatedPurePursuit`，前瞻距离 0.5~3.0m，支持倒车
- **代价地图**：全局 5cm 分辨率，静态层+障碍层+膨胀层；局部 6×6m 滚动窗口
- **足迹**：矩形 `0.9m × 0.6m`，四角坐标定义
- **不使用 AMCL**：SLAM 阶段由 slam_toolbox 直接提供定位（`map→odom` TF），无需粒子滤波定位
