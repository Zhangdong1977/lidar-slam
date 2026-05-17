# 自动探索建图参数调整记录

> 调整日期：2026-05-16  
> 基于日志：`log/explore_2026-05-15_19-22-02.log`  
> 问题摘要：探索过程中 279 次碰撞告警、45 次规划器超迭代、36 次 costmap 超时、34 次卡住，成功率仅 31%，最终地图仅 419×518 像素（约 21m×26m）

---

## 一、config/nav2_params_exploration.yaml

### 1.1 bt_navigator

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `bt_navigator.ros__parameters` | `default_server_timeout` | `20` | `60` | 规划器在大型地图中需要更长时间计算路径，20s 不足以等待规划完成 |

### 1.2 controller_server

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters` | `controller_frequency` | `10.0` | `20.0` | 提高控制频率，减少控制延迟，使机器人对障碍物响应更快 |
| `controller_server.ros__parameters` | `min_x_velocity_threshold` | `0.001` | `0.01` | 过低的速度阈值导致机器人实际未移动时仍认为在运动，延迟 stuck 检测 |
| `controller_server.ros__parameters` | `min_theta_velocity_threshold` | `0.001` | `0.01` | 同上，旋转速度阈值也需要提高以避免误判 |
| `controller_server.ros__parameters` | `transform_tolerance` | `0.5` | `1.0` | 日志中 costmap 反复从 (0,0) 规划表明 TF 变换查询频繁超时，增大容忍度 |

### 1.3 controller_server.progress_checker

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters.progress_checker` | `movement_time_allowance` | `20.0` | `30.0` | 降低的期望速度和更大的地图意味着机器人需要更长时间才能显示进展 |

### 1.4 controller_server.FollowPath (RegulatedPurePursuitController)

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters.FollowPath` | `desired_linear_vel` | `1.0` | `0.8` | 降低期望线速度，减少碰撞风险（日志中 279 次碰撞告警） |
| `controller_server.ros__parameters.FollowPath` | `lookahead_dist` | `1.8` | `2.0` | 增大前视距离，让机器人更早规划转向，减少急转弯 |
| `controller_server.ros__parameters.FollowPath` | `min_lookahead_dist` | `0.8` | `1.0` | 增大最小前视距离，避免在靠近目标时前视过短导致碰撞 |
| `controller_server.ros__parameters.FollowPath` | `max_lookahead_dist` | `3.0` | `4.0` | 在地图开阔区域允许更远的前视，生成更平滑的路径 |
| `controller_server.ros__parameters.FollowPath` | `lookahead_time` | `1.5` | `2.0` | 配合前视距离调整，使速度缩放后的前视距离更合理 |
| `controller_server.ros__parameters.FollowPath` | `transform_tolerance` | `0.5` | `1.0` | 与 controller_server 级别保持一致的 TF 容忍度 |
| `controller_server.ros__parameters.FollowPath` | `min_approach_linear_velocity` | `0.3` | `0.2` | 降低接近目标时的最小速度，减少接近阶段碰撞 |
| `controller_server.ros__parameters.FollowPath` | `approach_velocity_scaling_dist` | `0.6` | `0.8` | 增大减速距离，使速度过渡更平滑 |
| `controller_server.ros__parameters.FollowPath` | `max_allowed_time_to_collision_up_to_carrot` | `1.5` | `3.0` | **关键修复**：日志中 279 次 "collision ahead" 的根本原因，碰撞预测时间过短导致大量误报 |
| `controller_server.ros__parameters.FollowPath` | `regulated_linear_scaling_min_radius` | `0.5` | `0.6` | 增大最小转弯半径阈值，使速度缩放更保守 |
| `controller_server.ros__parameters.FollowPath` | `regulated_linear_scaling_min_speed` | `0.15` | `0.1` | 允许更低的巡航速度（配合 desired_linear_vel 降低） |
| `controller_server.ros__parameters.FollowPath` | `max_robot_pose_search_dist` | `5.0` | `10.0` | 在大型地图中需要更大搜索范围来找到路径上的最近位姿 |

### 1.5 local_costmap

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `local_costmap.local_costmap.ros__parameters` | `update_frequency` | `3.0` | `5.0` | 提高局部代价地图更新频率，减少碰撞和 stuck 事件 |
| `local_costmap.local_costmap.ros__parameters` | `width` | `4` | `6` | 增大局部窗口从 4m×4m 到 6m×6m，给控制器更好的局部环境感知 |
| `local_costmap.local_costmap.ros__parameters` | `height` | `4` | `6` | 同上 |
| `local_costmap.local_costmap.ros__parameters.obstacle_layer.scan` | `raytrace_max_range` | `3.5` | `4.0` | 配合窗口增大，障碍物感知范围扩展 |
| `local_costmap.local_costmap.ros__parameters.obstacle_layer.scan` | `obstacle_max_range` | `3.5` | `4.0` | 同上 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `cost_scaling_factor` | `0.8` | `1.5` | **关键修复**：增大代价衰减因子，使障碍物代价随距离更快下降，减少 "collision ahead" 误报 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `1.0` | `0.55` | **关键修复**：缩小膨胀半径从 1.0m→0.55m（车体半长+余量=0.45+0.1），减少过度填充，仓库过道宽约 1.5m，1.0m 膨胀会导致整个过道被标记为障碍物 |

### 1.6 global_costmap

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `global_costmap.global_costmap.ros__parameters` | `update_frequency` | `0.2` | `1.0` | **关键修复**：原 5 秒更新一次太慢，日志中 costmap 反复从 (0,0) 规划说明 TF 变换在更新周期内丢失，提升到 1Hz |
| `global_costmap.global_costmap.ros__parameters` | `transform_tolerance` | `0.5` | `1.0` | TF 查找容忍度提高，减少 "Costmap timed out" 错误（日志中出现 36 次） |
| `global_costmap.global_costmap.ros__parameters.inflation_layer` | `cost_scaling_factor` | `0.8` | `1.5` | 与局部代价地图保持一致 |
| `global_costmap.global_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `1.2` | `0.55` | 与局部代价地图保持一致，避免全局路径被过度膨胀的障碍物阻断 |

### 1.7 planner_server

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `planner_server.ros__parameters` | `costmap_update_timeout` | `2.0` | `10.0` | **关键修复**：规划器等待 costmap 更新的超时，原值 2s 太短，配合 global_costmap 1Hz 更新需要更长时间 |
| `planner_server.ros__parameters.GridBased` | `max_iterations` | `100000` | `250000` | **关键修复**：日志中 45 次 "exceeded maximum iterations"，在大地图（721×687）上需要更多搜索迭代 |
| `planner_server.ros__parameters.GridBased` | `max_on_approach_iterations` | `1000` | `2000` | 接近目标时的局部优化迭代数翻倍，提高到达率 |
| `planner_server.ros__parameters.GridBased` | `max_planning_time` | `5.0` | `15.0` | **关键修复**：5s 不足以在 700+ 格的地图上规划 Reeds-Shepp 路径，增大到 15s |

### 1.8 behavior_server

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `behavior_server.ros__parameters` | `transform_tolerance` | `0.1` | `0.5` | 与其他组件保持一致的 TF 容忍度 |

### 1.9 velocity_smoother

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `velocity_smoother.ros__parameters` | `velocity_timeout` | `1.0` | `2.0` | 速度命令超时延长，避免规划延迟期间误判超时 |

### 1.10 collision_monitor

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `collision_monitor.ros__parameters` | `transform_tolerance` | `0.2` | `0.5` | 与其他组件保持一致 |
| `collision_monitor.ros__parameters.FootprintApproach` | `time_before_collision` | `1.2` | `2.0` | **关键修复**：原值过于敏感，1.2s 预测碰撞导致大量误触发停车，延长到 2s 减少误报 |

---

## 二、config/ekf.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `ekf_filter_node.ros__parameters` | `frequency` | `50.0` | `100.0` | 提高 EKF 更新频率，使 odom→body_link TF 发布更密集，减少 costmap 变换查询超时 |
| `ekf_filter_node.ros__parameters` | `odom0_queue_size` | `10` | `20` | 增大里程计数据缓冲，避免高频 EKF 下的数据丢失 |
| `ekf_filter_node.ros__parameters` | `imu0_queue_size` | `10` | `20` | 增大 IMU 数据缓冲 |

---

## 三、config/slam_toolbox_ackermann.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `slam_toolbox.ros__parameters` | `transform_publish_period` | `0.05` | `0.02` | TF 发布从 20Hz→50Hz，确保 map→odom 变换在 global_costmap 需要时始终可用 |
| `slam_toolbox.ros__parameters` | `map_update_interval` | `2.0` | `5.0` | 降低地图发布频率，减少 global_costmap static_layer 频繁重建，提升规划稳定性 |
| `slam_toolbox.ros__parameters` | `transform_timeout` | `2.0` | `5.0` | 增大 TF 超时容忍度，配合其他组件的 transform_tolerance 调整 |
| `slam_toolbox.ros__parameters` | `queue_size` | `200` | `500` | **关键修复**：日志第 114 行 `"Message Filter dropping message ... queue is full"` 说明扫描队列不足，增大到 500 |
| `slam_toolbox.ros__parameters` | `minimum_travel_distance` | `0.3` | `0.5` | 增大最小移动距离阈值，减少高密度区域的冗余关键帧，提升后端优化速度 |
| `slam_toolbox.ros__parameters` | `minimum_travel_heading` | `0.2` | `0.3` | 同上，减少旋转时的冗余关键帧 |
| `slam_toolbox.ros__parameters` | `scan_buffer_size` | `15` | `25` | 增大扫描缓冲，提升扫描匹配的鲁棒性 |

---

## 调整总结

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞误报 | 279 次 "collision ahead" | 4 项 | `max_allowed_time_to_collision_up_to_carrot` ↑, `inflation_radius` ↓, `cost_scaling_factor` ↑, `time_before_collision` ↑ |
| TF/costmap 超时 | 36 次 "Costmap timed out" + 从 (0,0) 规划 | 7 项 | global_costmap `update_frequency` ↑, `transform_tolerance` ↑, `costmap_update_timeout` ↑, EKF `frequency` ↑, slam `transform_publish_period` ↓ |
| 规划器超迭代 | 45 次 "exceeded maximum iterations" | 4 项 | `max_iterations` ↑, `max_planning_time` ↑, `max_on_approach_iterations` ↑, `costmap_update_timeout` ↑ |
| 机器人卡住 | 34 次 "Robot stuck" + 成功率 31% | 间接修复 | 以上所有碰撞和规划修复综合作用，减少不必要的停车 |
| 消息丢失 | "queue is full" 丢弃激光扫描 | 1 项 | slam_toolbox `queue_size` ↑ |
