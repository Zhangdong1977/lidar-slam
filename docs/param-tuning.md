# 自动探索建图参数调整记录

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
| `controller_server.ros__parameters.FollowPath` | `desired_linear_vel` | `0.8` | `0.4` | (第一轮调整后实际运行值——中间调整未记录详细日志，当前文件值为 0.4) |
| `controller_server.ros__parameters.FollowPath` | `desired_linear_vel` | `0.4` | `0.25` | **修复撞击**：降低期望线速度，在狭窄仓库过道中留出更长的制动距离 |
| `controller_server.ros__parameters.FollowPath` | `use_collision_detection` | `False` | `True` | **修复撞击**：启用 RPP 控制器内置的 lookahead 弧线碰撞检测，使机器人在即将碰撞时主动减速/停止 |
| `controller_server.ros__parameters.FollowPath` | `use_cost_regulated_linear_velocity_scaling` | `False` | `True` | **修复撞击**：启用基于代价地图的速度缩放，机器人在高代价区域自动降速 |

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
| `local_costmap.local_costmap.ros__parameters.obstacle_layer.scan` | `obstacle_min_range` | `0.6` | `0.0` | **修复撞击**：原值过滤了距激光 0.6m 内的障碍物，导致近场障碍物在局部代价地图中完全不可见。激光高于车身 0.02m 无自检风险 |

### 1.6 global_costmap

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `global_costmap.global_costmap.ros__parameters` | `update_frequency` | `0.2` | `1.0` | **关键修复**：原 5 秒更新一次太慢，日志中 costmap 反复从 (0,0) 规划说明 TF 变换在更新周期内丢失，提升到 1Hz |
| `global_costmap.global_costmap.ros__parameters` | `transform_tolerance` | `0.5` | `1.0` | TF 查找容忍度提高，减少 "Costmap timed out" 错误（日志中出现 36 次） |
| `global_costmap.global_costmap.ros__parameters.inflation_layer` | `cost_scaling_factor` | `0.8` | `1.5` | 与局部代价地图保持一致 |
| `global_costmap.global_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `1.2` | `0.55` | 与局部代价地图保持一致，避免全局路径被过度膨胀的障碍物阻断 |
| `global_costmap.global_costmap.ros__parameters.obstacle_layer.scan` | `obstacle_min_range` | `1.0` | `0.3` | **修复撞击**：原值过滤了距激光 1.0m 内的障碍物，全局规划器因此看不到近场障碍物，规划出穿越障碍物的路径。降至 0.3m 保留少量过滤防止激光噪声在静态层中产生假障碍物 |

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
| `collision_monitor.ros__parameters.FootprintApproach` | `enabled` | `False` | `True` | **修复撞击**：启用基于 robot footprint 的预测性多边形碰撞检测，速度叠加后提前判断碰撞 |

### 1.11 controller_server.goal_checker（第三轮追加）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters.general_goal_checker` | `xy_goal_tolerance` | `0.1` | `0.5` | frontier 目标点位于已知/未知边界（靠近障碍物），0.1m 容忍要求机器人逼近障碍物。增大到 0.5m 使机器人在安全距离内即认为到达目标 |

### 1.12 planner_server.GridBased（第三轮追加）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `planner_server.ros__parameters.GridBased` | `cost_penalty` | `8.0` | `15.0` | 增大代价惩罚因子，使 SmacPlannerHybrid 更积极地避开代价地图中的障碍物膨胀区，规划出的整条路径（含终点）离障碍物更远 |

### 1.13 controller_server.progress_checker（第四轮追加）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters.progress_checker` | `required_movement_radius` | `0.15` | `0.3` | Ackermann 调头时位移极小（后轮小范围后退+前轮修正），0.15m 过于严格，增大到 0.3m 避免误判无进展 |
| `controller_server.ros__parameters.progress_checker` | `movement_time_allowance` | `30.0` | `60.0` | Ackermann 倒车调头需要更长时间，30s 不够完成完整调头动作序列 |

---

## 二、config/ekf.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `ekf_filter_node.ros__parameters` | `frequency` | `50.0` | `100.0` → `30.0` | 100Hz 导致 rviz2 Message Filter 队列溢出持续丢消息，30Hz 足够 nav2 使用 |
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

## 调整总结（第一、二轮）

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞误报 | 279 次 "collision ahead" | 4 项 | `max_allowed_time_to_collision_up_to_carrot` ↑, `inflation_radius` ↓, `cost_scaling_factor` ↑, `time_before_collision` ↑ |
| TF/costmap 超时 | 36 次 "Costmap timed out" + 从 (0,0) 规划 | 7 项 | global_costmap `update_frequency` ↑, `transform_tolerance` ↑, `costmap_update_timeout` ↑, EKF `frequency` ↑, slam `transform_publish_period` ↓ |
| 规划器超迭代 | 45 次 "exceeded maximum iterations" | 4 项 | `max_iterations` ↑, `max_planning_time` ↑, `max_on_approach_iterations` ↑, `costmap_update_timeout` ↑ |
| 机器人卡住 | 34 次 "Robot stuck" + 成功率 31% | 间接修复 | 以上所有碰撞和规划修复综合作用，减少不必要的停车 |
| 消息丢失 | "queue is full" 丢弃激光扫描 | 1 项 | slam_toolbox `queue_size` ↑ |

### 第二轮 (2026-05-18)：修复激光雷达障碍物检测失效

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 近场障碍物被忽略 | 机器人频繁撞击障碍物，导航路径穿越墙壁/货架 | 2 项 | local `obstacle_min_range` 0.6→0.0, global `obstacle_min_range` 1.0→0.3 |
| 控制器无碰撞检测 | 机器人不减速冲向障碍物 | 2 项 | `use_collision_detection` ↑, `use_cost_regulated_linear_velocity_scaling` ↑ |
| 速度过高 | 狭窄过道制动距离不足 | 1 项 | `desired_linear_vel` 0.4→0.25 |
| 碰撞监视不完整 | FootprintApproach 预测关闭 | 1 项 | `FootprintApproach.enabled` ↑ |

---

## 四、behavior_trees/ackermann_nav.xml（第四轮 2026-05-18）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `NavigateWithReplanning.RateController` | `hz` | `0.3` (3.33s) | `0.1` (10s) | **核心修复**：3.33s 重规划频率导致 Ackermann 调头时控制器状态被不断重置，后轮刚后退即被打断，陷入原地振荡。降到 10s 给足执行时间 |
| `RecoveryActions.RoundRobin` | `BackUp backup_dist` | `1.5` / `0.5` | `2.5` / `1.0` | Ackermann 被障碍物逼停后需更大后退距离腾出转弯空间，原值不足以让车体摆脱困境 |

### 第四轮 (2026-05-18)：修复 Ackermann 调头振荡、频繁重规划

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 路径重规划过频 | "Passing new path" 每 3.35s 触发，控制器状态不断重置 | 1 项 | `RateController.hz` 0.3→0.1 |
| 调头后退距离不足 | 后轮小范围后退，无法腾出转弯空间 | 2 项 | `backup_dist` 1.5→2.5, 0.5→1.0 |
| 进展检测过严 | progress timeout 触发前机器人实际在调头而非卡住 | 2 项 | `required_movement_radius` ↑, `movement_time_allowance` ↑ |

**根因分析**：日志显示 "Passing new path to controller" 每 ~3.35s 触发一次，由行为树 `<RateController hz="0.3">` 控制。每次新路径到达都会重置 RPP 控制器的内部跟踪状态。Ackermann 车辆倒车调头需要连贯的 steering+throttle 序列，被频繁打断后表现为：后轮仅轻微后退、前轮反复修正、机器人原地调整方向。

---

## 五、config/nav2_params_exploration.yaml（第五轮 2026-05-18）：修复规划器超迭代

### 5.1 planner_server.GridBased

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `planner_server.ros__parameters.GridBased` | `downsample_costmap` | `false` | `true` | **核心修复**：开启代价地图降采样，搜索空间缩小 4 倍，使 SmacPlannerHybrid 能在大地图上找到路径 |
| `planner_server.ros__parameters.GridBased` | `downsampling_factor` | `1` | `2` | 降采样倍数，2 倍降采样 + 144 角度分箱 → 状态数从 4.3 亿降至 1.1 亿 |
| `planner_server.ros__parameters.GridBased` | `max_iterations` | `1000000` | `2000000` | 翻倍搜索预算，配合降采样后在同等时间内探索更大比例的状态空间 |
| `planner_server.ros__parameters.GridBased` | `analytic_expansion_ratio` | `3.5` | `4.0` | 增大分析扩展比例，在开阔区域跳过多余的图搜索节点 |
| `planner_server.ros__parameters.GridBased` | `analytic_expansion_max_length` | `3.0` | `5.0` | 增大分析扩展最大长度，在稀疏障碍物区域快速跳跃到目标方向 |

### 第五轮总结：修复规划器超迭代

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 规划器超迭代 | "exceeded maximum iterations" 在 1507×1996 代价地图上反复失败 | 5 项 | `downsample_costmap` 开启, `downsampling_factor` 2, `max_iterations` 翻倍, `analytic_expansion` 增大 |

**根因分析**：代价地图扩张至 1507×1996 像素（约 75m×100m），SmacPlannerHybrid 使用 REEDS_SHEPP 运动模型 + 144 角度分箱，总状态数约 4.3 亿。100 万次迭代仅搜索了 0.23% 的状态空间。开启 2 倍降采样后状态数降至 1.1 亿，配合 200 万次迭代可探索约 1.8%。同时增大分析扩展参数使规划器在开阔区域快速跳过大片空白格子。

---

## 六、config/nav2_params_exploration.yaml（第六轮 2026-05-18）：修复碰撞检测瘫痪

### 6.1 local_costmap.inflation_layer

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `1.5` | `0.6` | **核心修复**：1.5m 膨胀半径在 6×6m 局部代价地图中造成大面积致命代价区，RPP 碰撞检测（前视 0.75m）持续命中致命单元，导致机器人 0.4s 内触发 "Controller patience exceeded"，完全无法移动。0.6m = 车体半长 0.45 + 安全余量 0.15，足够安全 |

### 第六轮总结：修复碰撞检测瘫痪

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞检测瘫痪 | "collision ahead!" 每 50ms 触发一次，0.4s 后 "Controller patience exceeded"，机器人完全不动 | 1 项 | local `inflation_radius` 1.5→0.6 |

**根因分析**：碰撞检测本身正常工作——`max_allowed_time_to_collision_up_to_carrot: 3.0`（前视 0.75m）在检测到致命代价时正确触发警告。问题在于局部代价地图的 `inflation_radius: 1.5` 导致障碍物周围 1.5m 内布满膨胀代价，而 6×6m 窗口内几乎所有区域都被致命代价覆盖。0.6m 的膨胀半径（车体半长 0.45m + 0.15m 余量）在保证安全的同时，避免过度填充狭窄过道。

---

## 七、behavior_trees + Nav2 参数（第七轮 2026-05-18）：修复 Start Occupied 死锁

> 基于日志：`log/explore_2026-05-18_21-43-34.log`

### 根因分析

机器人自主探索时与障碍物碰撞后永久卡死，循环报错 `Start occupied`（SmacPlannerHybrid 错误码 205）。问题分两层：

1. **如何进入障碍物**：控制器检测到碰撞 → `failure_tolerance: 5.0` 允许 5 次连续失败 → 每次失败清除 local costmap → 机器人逐渐向前蠕动 → 最终物理上进入障碍物内部
2. **为何无法恢复（核心 Bug）**：SmacPlannerHybrid 返回错误码 205（START_OCCUPIED），但 BT 的恢复门 `WouldAPlannerRecoveryHelp` 只检查 200/207/208 三个错误码，**不包含 205**。Fallback 门返回 FAILURE，整个恢复分支（包含 BackUp 倒车）被跳过

### 7.1 behavior_trees/ackermann_nav.xml

| 参数位置 | 变更 | 调整原因 |
|----------|------|----------|
| 恢复门 Fallback | 添加 `AreErrorCodesPresent error_code="{compute_path_error_code}" error_codes_to_check="205;206"` | **核心修复**：START_OCCUPIED(205) 和 GOAL_OCCUPIED(206) 错误码被 WouldAPlannerRecoveryHelp 忽略，导致 BackUp 倒车恢复永远不执行 |

### 7.2 config/nav2_params_exploration.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters` | `failure_tolerance` | `5.0` | `2.0` | 减少允许连续失败次数，阻止机器人蠕动进入障碍物 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `0.6` | `0.8` | 增大本地膨胀半径，提早避开障碍物 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `cost_scaling_factor` | `2.0` | `3.0` | 配合更大膨胀半径，使代价衰减更陡峭 |

---

## 八、Nav2 参数调整（第十轮 2026-05-19）：修复 Nav2 瞬间到达

### 8.1 config/nav2_params_exploration.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `planner_server.GridBased.tolerance` | 2.0 | 0.75 | 规划器容差过大导致近距离目标被瞬间"到达"，降至 0.75m 迫使规划器创建实际路径 |
| `controller_server.general_goal_checker.xy_goal_tolerance` | 0.5 | 0.35 | 更严格的目标到达判定，配合缩小的规划容差 |

### 第十轮总结

| 问题类别 | 核心改动 |
|----------|----------|
| Nav2 瞬间到达 | planner tolerance 2.0→0.75, xy_goal_tolerance 0.5→0.35 |

---

## 九、Nav2 参数调整（第十二轮 2026-05-19）：减少自动探索碰撞

> 基于日志：`log/explore_2026-05-19_11-16-55.log`

### 9.1 config/nav2_params_exploration.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `local_costmap.inflation_layer` | `inflation_radius` | `0.8` | `1.0` | 增大膨胀半径，梯度保护带从 0.5m 增至 0.70m（内切圆 0.30m 不变）。第 6 轮已证 1.5 会瘫痪 6x6m 窗口 |
| `local_costmap.inflation_layer` | `cost_scaling_factor` | `3.0` | `2.0` | 减缓代价衰减速度，csf=3.0 在 0.5m 处代价仅 59，csf=2.0 提升至约 92 |
| `global_costmap.obstacle_layer.scan` | `obstacle_min_range` | `0.3` | `0.0` | 0.3m 以内障碍物被过滤，与 ackermann 配置一致 |
| `controller_server.FollowPath` | `transform_tolerance` | `2.0` | `1.0` | 2 秒前的 TF 位姿偏差可达 0.3-0.5m |
| `controller_server.FollowPath` | `approach_velocity_scaling_dist` | `0.5` | `0.6` | 增大减速起始距离，配合更大的 inflation_radius |
| `controller_server.FollowPath` | `max_allowed_time_to_collision_up_to_carrot` | `3.0` | `1.5` | 碰撞预测窗口从 0.75m 缩小到 0.375m |
| `velocity_smoother` | `max_velocity` | `[1.0, 0.0, 1.0]` | `[0.5, 0.0, 1.0]` | 峰值速度制动距离从 0.72m 降至 0.12m |
| `velocity_smoother` | `max_accel` | `[1.5, 0.0, 2.5]` | `[1.0, 0.0, 2.0]` | 减少轮胎打滑和里程计漂移 |
| `velocity_smoother` | `max_decel` | `[-1.5, 0.0, -2.5]` | `[-1.0, 0.0, -2.0]` | 与 max_accel 对称 |
| `velocity_smoother` | `velocity_timeout` | `2.0` | `1.0` | 惯性滑行从 2 秒降至 1 秒 |
| `collision_monitor.FootprintApproach` | `time_before_collision` | `2.0` | `1.5` | 配合降速后的 max_velocity=0.5 |

### 第十二轮总结

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞检测频繁 | 562 次 "detected collision ahead" | 11 项参数 | inflation_radius ↑, cost_scaling_factor ↓, max_velocity ↓, obstacle_min_range 调整 |
| 导航频繁中止 | 47 次 abort | 间接改善 | 控制器参数收紧 |
| 规划器超迭代 | 35 次超迭代 | 间接改善 | global obstacle_min_range 归零 |
| 后退恢复失败 | 13 次 backup failed | 间接改善 | 碰撞减少后触发次数降低 |

---

## 十、behavior_trees + Nav2 参数（第十三轮 2026-05-19）：优化 Ackermann 脱困/调头行为

> 问题：脱困和调头时前后移动幅度过小，频繁前进后退振荡

### 根因分析

Ackermann 小车遇到死胡同或需要调头时，RPP 控制器（`allow_reversing=true`）在航向偏差超过 90° 时发出倒车+转向命令，但由于倒车速度过低（-0.35 m/s），移动一点后 lookahead 点更新导致航向偏差回落到 90° 以下，RPP 切换回前进方向 → 小车又朝障碍物移动 → collision_detection 触发减速/停止 → 回到倒车 → **无限振荡**。

同时 `regulated_linear_scaling_min_speed=0.05` 和 `min_approach_linear_velocity=0.08` 导致在障碍物附近以极低速度蠕行（实际≈不动），`cost_scaling_factor=2.0` 使代价衰减不够快，大面积高代价区域持续降速。

### 10.1 config/nav2_params_exploration.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `velocity_smoother.ros__parameters` | `min_velocity` | `[-0.35, 0.0, -1.0]` | `[-0.5, 0.0, -1.0]` | 提高倒车最大速度，让小车在调头时有足够速度完成弧线动作，避免被RPP频繁切换方向 |
| `controller_server.FollowPath` | `min_approach_linear_velocity` | `0.08` | `0.15` | 提高最低速度，避免在障碍物附近以 0.08 m/s 蠕行（实际≈不动），至少保持可感知的移动 |
| `controller_server.FollowPath` | `regulated_linear_scaling_min_speed` | `0.05` | `0.15` | 提高最小巡航速度，costmap 高代价区域不再降到 0.05 m/s，保持有效移动能力 |
| `local_costmap.inflation_layer` | `cost_scaling_factor` | `2.0` | `4.0` | 加速代价衰减，远离障碍物的区域代价更低，减少不必要的减速区域面积 |
| `global_costmap.inflation_layer` | `cost_scaling_factor` | `2.0` | `4.0` | 同上，全局代价地图也加速衰减 |
| `planner_server.GridBased` | `reverse_penalty` | `2.0` | `1.5` | 降低倒车惩罚，鼓励规划出连贯的倒车调头路径而非多次方向切换 |

### 10.2 config/nav2_params_ackermann.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `velocity_smoother.ros__parameters` | `min_velocity` | `[-0.35, 0.0, -1.0]` | `[-0.5, 0.0, -1.0]` | 与探索模式保持一致 |
| `controller_server.FollowPath` | `min_approach_linear_velocity` | `0.05` | `0.15` | 与探索模式保持一致 |
| `controller_server.FollowPath` | `regulated_linear_scaling_min_speed` | `0.05` | `0.15` | 与探索模式保持一致 |
| `local_costmap.inflation_layer` | `cost_scaling_factor` | `1.0` | `2.0` | 加速代价衰减，减少减速区域 |
| `global_costmap.inflation_layer` | `cost_scaling_factor` | `1.5` | `3.0` | 同上 |

### 10.3 behavior_trees/ackermann_nav.xml

| 参数位置 | 变更 | 调整原因 |
|----------|------|----------|
| `RecoveryActions.RoundRobin` 顺序 | 清地图→等3s→退2.5m→退1.0m 改为 **退3.5m→清地图→等2s→退2.0m** | 脱困时最需要的是立即后退，不应先浪费时间清地图和等待。后退前置可第一时间脱离障碍物 |
| `BackUp` 第一个 | `backup_dist=2.5, backup_speed=0.25` → `backup_dist=3.5, backup_speed=0.35` | 增大后退距离和速度，配合提高的 min_velocity(-0.5)，一次后退足够远离障碍物 |
| `BackUp` 第二个 | `backup_dist=1.0, backup_speed=0.2` → `backup_dist=2.0, backup_speed=0.3` | 同上 |
| `Wait` | `wait_duration=3` → `wait_duration=2` | 减少无效等待时间 |

### 第十三轮总结：优化 Ackermann 脱困/调头行为

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 调头前后振荡 | 小车在死胡同频繁前进后退，移动幅度极小 | 6 参数 + BT | velocity_smoother 倒车速度 ↑, 最小速度 ↑, cost_scaling_factor ↑, reverse_penalty ↓, BackUp 前置+增大距离 |
| 障碍物附近蠕行 | 0.05 m/s 速度下实际未移动 | 2 参数 | min_approach_velocity ↑, regulated_min_speed ↑ |
| BT恢复效率低 | 脱困时先清地图等3s才后退 | BT重构 | BackUp 移到 RoundRobin 第一位 |

---

## 十一、openTCS 集成参数（2026-05-19）

### 概述

新增 `opentcs_nav2_bridge` 桥接节点，将 openTCS-NeNa 调度系统的 topic 接口与 Nav2 NavigateToPose action 对接。

### 11.1 桥接节点参数 (`opentcs_nav2_bridge`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_pose_topic` | `/goal_pose` | openTCS 发布的导航目标 topic (PoseStamped) |
| `amcl_pose_topic` | `/amcl_pose` | 发布给 openTCS 的机器人位置 topic (PoseWithCovarianceStamped) |
| `nav_action_name` | `/navigate_to_pose` | Nav2 导航 action 名称 |
| `target_frame` | `map` | TF 目标帧（用于读取机器人位置） |
| `source_frame` | `body_link` | TF 源帧 |
| `pose_publish_rate` | `10.0` | 位置发布频率 Hz，10Hz 足以满足调度系统跟踪需求 |

### 11.2 DDS 兼容性配置

| 环境变量 | 值 | 说明 |
|----------|-----|------|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 与 openTCS-NeNa IHMC Fast-RTPS 对齐，确保 DDS 发现兼容 |
| `ROS_DOMAIN_ID` | `42` | 当前环境默认值 42，openTCS-NeNa 默认=30，需在 Kernel Control Center 中改为 42 |

### 11.3 工厂世界关键坐标参考

充电站位置（来自 factory.sdf）：
| 名称 | ROS2 (x, y) m |
|------|---------------|
| charger_1 | (38, 38) |
| charger_2 | (41, 38) |
| charger_3 | (44, 38) |
| charger_4 | (38, 44) |
| charger_5 | (41, 44) |
| charger_6 | (44, 44) |

### 11.4 相关文件

| 文件 | 说明 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_nav2_bridge.py` | 桥接节点 |
| `launch/sim_ackermann_opentcs.launch.py` | 集成 launch 文件（基于 sim_ackermann_nav + 桥接节点） |
| `scripts/launch/sim_ackermann_opentcs.sh` | 启动脚本 |

---

## 十二、Nav2 参数修复（第十五轮 2026-05-19）：消除虚假碰撞

### 12.1 参数修复

**文件**: `config/nav2_params_exploration.yaml`

| 参数 | 位置 | 旧值 | 新值 | 原因 |
|------|------|------|------|------|
| `obstacle_min_range` | local costmap | 0.0 | 0.5 | LiDAR 硬件最小量程 0.15m，设为 0 允许车身边缘回波/噪声被标记为障碍物，经 inflation 在 footprint 内产生 lethal cost，RPP 检查整个 footprint 不区分前后导致误报 |
| `obstacle_min_range` | global costmap | 0.0 | 0.5 | 同上 |
| `minimum_turning_radius` | SmacPlannerHybrid | 0.35 | 1.0 | 实际最小转弯半径 = wheel_base/tan(max_steer) = 0.58/tan(30°) = 1.004m，偏小 65% 导致规划路径包含机器人无法执行的急转弯 |

---

## 十三、替换探索器为 Ackermann 感知版本（第十六轮 2026-05-19）

### 根因分析

Ackermann 机器人在自动探索时频繁倒退或侧向移动。根因：

1. **旧探索器不考虑机器人朝向**：`orientation_scale: 0.0` 完全忽略机器人当前朝向，目标选择只看距离和面积
2. **目标朝向始终为 identity (w=1.0)**：旧探索器发给 Nav2 的目标姿态永远是朝东（heading=0°），无论机器人面朝哪里
3. **SmacPlannerHybrid 允许倒车**：`REEDS_SHEPP` + `allow_reverse_expansion: true` + `reverse_penalty: 1.5`（过低），规划器对身后目标生成倒车路径
4. **RPP 控制器执行倒车**：`allow_reversing: true` 使控制器执行倒车段

替换为自定义 `frontier_explorer.py`（含朝向感知评分和 Ackermann 可行性过滤）。

### 13.1 启动文件修改

**文件**: `launch/sim_ackermann_explore.launch.py`

| 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| 探索节点 | `explore_lite` | `frontier_explorer` | 使用 Ackermann 感知的自定义探索器，包含朝向评分和可行性过滤 |
| 参数文件 | `config/explore_lite_params.yaml` | `config/frontier_explorer_params.yaml` | 新建专用参数配置 |
| `map_saver_watcher` | 包含在 launch 中 | 移除 | `frontier_explorer.py` 内部已有 `save_map()` 方法，不需要外部 watcher |

### 13.2 新建参数配置

**文件**: `config/frontier_explorer_params.yaml`

```yaml
frontier_explorer:
  ros__parameters:
    frontier_min_size: 20
    size_weight: 1.0
    distance_weight: 0.5
    heading_weight: 2.0        # 朝向成本权重，惩罚非前方目标
    max_goal_distance: 15.0
    min_goal_distance: 1.0
    max_heading_diff: 3.0       # ~172°, 拒绝几乎正后方的目标
    explore_rate: 0.5
    completion_check_count: 10
    planning_retry_count: 5
    max_global_retries: 1
    min_free_cells: 500
    stuck_timeout: 10.0
    stuck_distance: 0.15
    goal_obstacle_clearance: 0.6
    initial_warmup_seconds: 10.0
    nav2_wait_timeout: 120.0
```

### 13.3 Nav2 规划参数优化

**文件**: `config/nav2_params_exploration.yaml`

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `planner_server.GridBased.reverse_penalty` | 1.5 | 5.0 | 大幅惩罚倒车路径，配合朝向感知的探索器减少不必要的倒车 |

### 第十六轮总结

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 机器人频繁倒退/侧向 | 旧探索器忽略朝向，目标朝向固定为东方 | 替换为 frontier_explorer（朝向评分 + Ackermann 可行性过滤） |
| 倒车路径成本低 | reverse_penalty=1.5 倒车仅比前进贵 50% | reverse_penalty 提升至 5.0 |

---

## 十四、frontier_explorer 参数优化（第十七轮 2026-05-19）：修复位置级卡死

> 基于日志：`log/explore_2026-05-19_17-28-41.log`

### 根因分析

机器人成功完成 37 个目标后卡在 (30.91, -26.12)，连续 8 次尝试西侧前沿全部失败（移动仅 0-3mm）。三层叠加：

1. **`max_heading_diff` 过宽**：3.0 弧度 = 171.9°，允许 160°~177° 的目标通过。Ackermann 无法高效到达身后 170°+ 的目标
2. **黑名单距离过小**：`failed_centroids` 用 `min_goal_distance`(1.0m) 过滤，但卡住期间各前沿相距 7-13m，黑名单完全无效
3. **无位置级卡住检测**：在同一位置反复尝试不同前沿，没有机制识别"这个位置已经无法继续"

### 14.1 frontier_explorer.py 源码修改

| 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_heading_diff` 默认值 | 3.0 | 2.5 | 3.0(172°) 允许几乎正后方的目标，2.5(143°) 拒绝需要大角度掉头的目标 |
| 新增 `blacklist_radius` 参数 | — | 5.0m | 黑名单匹配距离从 1.0m 增大到 5.0m，同一区域不同前沿被归为同一不可达区域 |
| 新增 `stuck_position_count` 参数 | — | 3 | 连续 3 个目标在同一位置失败时触发位置级卡住检测 |
| 新增 `_update_stuck_position()` 方法 | — | 追踪连续失败的物理位置 | 位移 >2m 视为不同位置并重置计数 |
| `explore_step()` 添加位置级检查 | — | 达到阈值后清黑名单重试 | 给被误判的前沿一次重新评估机会 |
| `result_callback()` SUCCEEDED 重置 | — | 清零 `stuck_position` 和计数 | 成功导航后重置位置级追踪 |

### 14.2 config/frontier_explorer_params.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_heading_diff` | 3.0 | 2.5 | ~143°，拒绝需要大角度掉头的目标 |
| `blacklist_radius` | (新增) | 5.0 | 黑名单匹配半径，覆盖同一不可达区域的不同前沿 |
| `stuck_position_count` | (新增) | 3 | 连续失败触发阈值 |

### 第十七轮总结

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 极端角度目标 | 160°~177° 目标通过 Ackermann 过滤 | `max_heading_diff` 3.0→2.5 |
| 黑名单无效 | 各前沿相距 7-13m，1.0m 黑名单无过滤 | `blacklist_radius` 5.0m |
| 位置级循环 | 同一位置 8 次尝试不同前沿全部失败 | `_update_stuck_position` + `stuck_position_count=3` |

---

## 十五、frontier_explorer 参数优化（第十八轮 2026-05-19）：增大 stuck_timeout 让 Nav2 恢复生效

### 根因分析

frontier_explorer 的 `stuck_timeout=10s` 在 Nav2 完成恢复前就取消了目标。时间线冲突：

| 组件 | 超时 | 动作 |
|------|------|------|
| frontier_explorer `stuck_timeout` | 10 秒 | 取消目标，换下一个前沿 |
| Nav2 `progress_checker.movement_time_allowance` | 60 秒 | 报告无进展 |
| Nav2 BT RecoveryNode | 6 轮 × ~20s | BackUp + ClearCostmap + Wait |

frontier_explorer 10 秒就取消 → Nav2 的 BackUp 恢复动作**从未执行** → 机器人永远无法通过后退脱困。

### 15.1 config/frontier_explorer_params.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `stuck_timeout` | 10.0 | 180.0 | 给 Nav2 足够时间完成完整恢复周期：progress_checker 60s + BT 6 轮恢复 × ~20s |
| `stuck_distance` | 0.15 | 0.5 | 配合更长的超时，0.5m 表示机器人确实在移动（而非噪声） |

---

## 十六、frontier_explorer 源码修改（第十九轮 2026-05-19）：朝向过滤器导致角落处过早宣布完成

### 根因分析

机器人在地图东北角 (48.27, 47.99) 完成最后一个目标后，面向角落墙壁。所有未探索前沿都在机器人身后（朝向差 >143°）。`max_heading_diff: 2.5`（≈143°）作为**硬过滤器**，把所有身后前沿全部剔除 → `find_frontiers()` 返回空 → 连续 10 次无前沿 → 错误宣布"探索完成"。

实际上地图仍有一半区域未探索。

### 16.1 两轮过滤策略

**代码改动**（`find_frontiers()` 方法）：

旧逻辑：`is_ackermann_feasible()` 硬过滤所有朝向差 > `max_heading_diff` 的目标。

新逻辑：分两个桶收集候选目标：
1. **严格桶**（`strict_goals`）：朝向差 ≤ `max_heading_diff`（优先）
2. **宽松桶**（`relaxed_goals`）：朝向差 > `max_heading_diff`，重算分数（去掉朝向惩罚）

优先返回严格桶；如果严格桶为空，回退到宽松桶，并打印日志 `"No heading-feasible frontiers, relaxing heading constraint for N behind-robot frontiers"`。

**效果**：
- 正常情况下（前方有前沿）→ 行为不变，优先选择前方目标
- 在角落/死胡同（前方无前沿）→ 放宽朝向约束，接受身后目标，让 SmacPlannerHybrid REEDS_SHEPP 规划掉头路径
- 朝向仍通过 `heading_weight` 作为**评分偏好**，不会退回到无朝向感知状态

---

## 十七、frontier_explorer 参数优化（第二十轮 2026-05-19）：修复探索在角落后过早宣布完成

> 基于日志：`log/explore_2026-05-19_19-10-41.log`

### 根因分析

机器人到达 (-47.80, -28.12) 后连续 10 次检测不到前沿，宣布"EXPLORATION COMPLETE"。但地图有 **25.7% 未知区域**（53,955 个前沿格子），远未完成。

用保存的地图模拟 `find_frontiers()` 过滤逻辑：
- 15m 范围内有 50 个有效前沿集群，但全部在机器人身后（航向差 138°~180°）
- 这些集群通过 `relaxed_goals` 桶时可能因 SLAM 实时更新差异被过滤
- **根本问题**：机器人"一条路走到黑"到达角落，`max_goal_distance=15m` 对 ~60×70m 场景太小，无法导航回未探索区域

### 17.1 frontier_explorer.py 源码修改

| 变更 | 说明 |
|------|------|
| 新增 `_long_range_fallback()` 方法 | 当 `find_frontiers()` 第一阶段（距离+航向约束）返回空时，搜索全地图最大前沿集群，在 `max_goal_distance` 处生成中间航点 |
| 新增 `_unknown_ratio()` 方法 | 计算当前地图未知区域比例 |
| `explore_step()` 覆盖率检查 | 宣布完成前检查 `_unknown_ratio() > max_unknown_ratio`，若未达标则重置计数器并清空黑名单 |

**远距离回退机制**：
1. 正常搜索无结果 → 触发 `_long_range_fallback()`
2. 搜索全地图，找最大前沿集群（size ≥ `long_range_min_cluster_size`）
3. 计算集群质心方向，在 `max_goal_distance` 处生成中间航点
4. 航点必须通过 snap_to_free + obstacle_clearance 检查
5. 日志：`"Long-range fallback: cluster at (x, y) [size=N, dist=Dm], waypoint (wx, wy)"`

**覆盖率保护**：
- 即使连续 30 次无前沿，若未知区域 > 15% 仍不宣布完成
- 重置计数器并清空黑名单，给前沿重新评估机会

### 17.2 config/frontier_explorer_params.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_goal_distance` | 15.0 | 20.0 | 扩大搜索半径，仓库场景前沿间距通常 10-30m |
| `completion_check_count` | 10 | 30 | 10 次=20 秒太短，SLAM 0.2Hz 更新频率下可能漏前沿。30 次=60 秒给 SLAM 足够时间稳定 |
| `long_range_enabled` | (新增) | true | 启用远距离前沿回退机制 |
| `long_range_min_cluster_size` | (新增) | 50 | 远距离搜索最小集群阈值，过滤零散前沿 |
| `max_unknown_ratio` | (新增) | 0.15 | 地图未知区域超过 15% 时不允许宣布完成 |

### 第二十轮总结

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 角落后无前沿可到达 | 25.7% 未知区域却宣布完成 | `_long_range_fallback()` 生成中间航点导航回未探索区域 |
| 完成判定过快 | 连续 10 次（20 秒）即宣布完成 | `completion_check_count` 10→30, 新增覆盖率保护 `max_unknown_ratio=0.15` |
| 搜索半径过小 | 15m 对 60×70m 场景不够 | `max_goal_distance` 15→20m |

---

## 十八、frontier_explorer 参数优化（第二十一轮 2026-05-19）：修复探索完成后仍遗留大片未知区域

> 基于日志：`log/explore_2026-05-19_19-43-11.log`

### 根因分析

探索任务运行 ~53 分钟后宣布"EXPLORATION COMPLETE"，52/53 个目标成功到达。但最终地图仍有 351,943 个未知单元格（8.8%），更关键的是**地图中还有 366 个 ≥20 格的前沿簇、61,968 个前沿单元格**未被探测到。

**根因链条**：

1. **机器人停在角落 (-47.71, -45.02)**：距地图对侧前沿超过 20m
2. **`max_goal_distance: 20.0` 过滤掉了所有远距离前沿**：前沿目标必须在 20m 以内
3. **`_long_range_fallback` 只尝试 1 个簇**：选最大簇生成中间航点，失败即放弃
4. **`_unknown_ratio()` 用整个 100m×100m 网格计算**：90.7% 都是空地（free），8.8% 未知低于 15% 阈值，误判为"覆盖完成"。实际上 289,975 个未知格子被墙壁包围（不可达），62,000 个是可达但被遗漏的
5. **`long_range_min_cluster_size: 50` 过滤了中型簇**：366 个簇中很多是 20-50 格的中型簇

### 18.1 参数调整 (`config/frontier_explorer_params.yaml`)

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_goal_distance` | 20.0 | 35.0 | 100m×100m 地图中 20m 搜索半径仅覆盖 12.5%，35m 覆盖 38% |
| `long_range_min_cluster_size` | 50 | 20 | 与 `frontier_min_size` 对齐，不遗漏 20-50 格的中型前沿簇 |
| `max_unknown_ratio` | 0.15 | 0.05 | 收紧覆盖门槛。旧值 0.15 在 4M 格地图中对应 60 万未知格子，过于宽松 |
| `completion_check_count` | 30 | 20 | 配合 flood-fill 覆盖检查，减少无意义等待 |
| `stuck_timeout` | 180.0 | 120.0 | 3 分钟过长，减少卡住等待时间 |

### 18.2 `_unknown_ratio()` 改为 flood-fill 可达区域计算

旧方法对整个地图网格（含大量外围空地）计算未知比例，导致 8.8% 看起来"足够好"。

新方法：从机器人位置 BFS flood-fill，只统计可到达的自由空间和与之相邻的未知单元格。这样排除了被墙壁包围的不可达未知区域和远离仓库的开阔空间。

### 18.3 `_long_range_fallback()` 尝试多个簇

旧逻辑只选最大的 1 个簇，航点生成失败即放弃。新逻辑按簇大小降序排列，循环尝试前 3 个簇，任一成功即返回。同时修复了旧代码中 `return` 语句后的死代码。

### 18.4 新增 `_reposition_to_frontiers()` 方法

当覆盖不足且无前沿可用时，计算所有前沿簇的加权几何中心（权重 = 簇大小），导航到该位置重新扫描。这驱使机器人从角落移动到未探索区域的中心。

在 `result_callback()` 中添加 `_repositioning` 标志处理，重新定位完成后打印日志重新搜索前沿。

### 第二十一轮总结

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 未知区域比例误判 | 8.8% 未知（含大量外围空地）低于 15% 阈值 | `_unknown_ratio()` 改为 flood-fill 可达区域计算 |
| 远距离前沿不可达 | 机器人在角落，前沿超过 20m | `max_goal_distance` 20→35m |
| 长距离回退太弱 | 只试 1 个簇，失败即放弃 | `_long_range_fallback` 尝试前 3 个簇 |
| 无法回溯未探索区域 | 无前沿时直接宣布完成 | 新增 `_reposition_to_frontiers()` 导航到前沿中心 |

---

## 十九、frontier_explorer 参数修复（2026-05-20）：修复位置卡死恢复无效

> 基于日志：`log/explore_2026-05-19_22-16-06.log`

### 根因分析

机器人完成 29/35 个目标后在 (-21.45, 8.57) 永久卡死。位置级卡住检测触发后只做"清黑名单+重试"，但机器人**物理上无法移动**而非前沿被过滤。`_reposition_to_frontiers()` 仅在无前沿可发现时触发，位置卡死时仍有前沿存在所以不会触发。`max_global_retries=1` 只允许 1 次重试就放弃。

### 19.1 frontier_explorer.py 源码修改

| 变更 | 说明 |
|------|------|
| 位置级卡住检测移到 `find_frontiers()` 之前 | 卡住时跳过前沿搜索，直接尝试重新定位 |
| 清黑名单后调用 `_reposition_to_frontiers()` | 导航到前沿簇加权中心，从新位置重新搜索 |
| 重试耗尽时检查 `_unknown_ratio()` | 覆盖率不足则重置计数器并尝试重新定位，而非直接宣布完成 |

### 19.2 config/frontier_explorer_params.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `stuck_timeout` | 120.0 | 180.0 | Nav2 完整恢复周期（progress_checker 60s + BT 6轮恢复 × ~20s）需要 >120s |
| `max_global_retries` | 1 | 3 | 1 次重试太少，位置卡死需要多次尝试不同方向的重新定位 |

### 第十九轮总结

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 位置卡死恢复无效 | 4 次卡在 (-21.45, 8.57)，清黑名单后重试仍然卡住 | 位置卡死时触发 `_reposition_to_frontiers()` |
| 重试耗尽过早放弃 | `max_global_retries=1` 仅 1 次重试就宣布完成 | 重试耗尽时检查覆盖率，不足则重置并重新定位 |
| stuck_timeout 不足 | Nav2 恢复未完成就被取消 | `stuck_timeout` 120→180s |

---

## 二十、Gazebo 模型路径修复（2026-05-20）：factory.sdf 模型加载失败

### 根因分析

`sim_ackermann_opentcs.sh` 启动后 Gazebo 立即退出，日志显示所有 `aws_robomaker_warehouse_*` 模型无法找到（Error Code 14）。`GZ_SIM_RESOURCE_PATH` 仅指向 `/home/hello/lidar-slam/models`（仅含 `ackermann/`），但 factory.sdf 引用的仓库货架/桌子/杂物模型实际位于 `third-party/aws-robomaker-small-warehouse-world/models/`。Gazebo 作为 required 节点，退出后导致整个 launch 系统关闭。

### 20.1 launch/sim_ackermann_opentcs.launch.py

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `GZ_SIM_RESOURCE_PATH` | `models/` | `models/:third-party/aws-robomaker-small-warehouse-world/models/` | 路径缺少 AWS 仓库模型目录，Gazebo 无法解析 `model://aws_robomaker_warehouse_*` URI |

### 20.2 改为 SLAM 模式

| 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| 定位方式 | `localization_launch.py`（map_server + AMCL）+ 静态地图 `ackermann_map.yaml`（9.9m×9.9m） | `slam_toolbox online_async` | 静态地图仅 198×198 像素（~10m），远小于 factory.sdf 的 ~100m×100m 场景，需实时建图 |
| bridge scan topic | `/scan` | `/scan_raw` + `scan_range_filter` | SLAM 模式需要过滤 inf/NaN 扫描数据 |
| bridge /tf | 包含 `/tf@tf2_msgs/msg/TFMessage` | 移除 | slam_toolbox 负责发布 map→odom TF，不需要桥接 Gazebo 的 /tf |
| slam_toolbox 启动延迟 | N/A | 5s（与 EKF 同步） | slam_toolbox 需要先收到 odom 和 scan 数据 |

---

## 十三、openTCS launch 桥接修复（2026-05-20）

### 13.1 /scan 话题名称不匹配 + ROS2 侧重映射

| 文件 | 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|------|
| `sim_ackermann_opentcs.launch.py` | bridge scan topic | `/scan` | `/scan_raw`（GZ 侧） | URDF gpu_lidar 发布到 `/scan_raw` |
| `sim_ackermann_opentcs.launch.py` | bridge remappings | 无 | `('/scan_raw', '/scan')` | ROS2 侧重映射到 `/scan`，让 AMCL/costmap 能收到数据 |
| `sim_ackermann_nav.launch.py` | 同上 | 同上 | 同上 | 同上 |

### 13.2 TF frame_id 不匹配

| 文件 | 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|------|
| `sim_ackermann_opentcs.launch.py` | static TF child frame | `body_link/lidar` | `ackermann_robot/body_link/lidar` | Gazebo 会给模型内的 frame 加上 `<model_name>/` 前缀，导致 scan 数据的 frame_id 与 TF 树不匹配 |
| `sim_ackermann_nav.launch.py` | 同上 | 同上 | 同上 | 同上 |

**故障链**：scan frame_id 为 `ackermann_robot/body_link/lidar` → TF 树只有 `body_link/lidar` → AMCL 无法将 scan 转换到 map frame → `map` frame 不存在 → Nav2 bringup 超时

---

## 二一、VehicleController 方向切换协调逻辑（2026-05-22）：修复驱动轮与方向轮配合不良

### 根因分析

Ackermann 小车调头时驱动轮和方向轮配合不好。后轮速度由 `forward_velocity_controller` 控制，可以瞬间反转；但前轮转向从 +30° 转到 -30° 需要 `2 × 0.5236 / 1.5708 ≈ 0.67 秒`（受 `max_steering_angular_velocity` 限制）。在方向切换的过渡期，后轮已按新方向行驶但前轮还在转动中，导致轨迹偏差和侧滑。

### 21.1 src/ackermann_control/include/ackermann_control/vehicle_controller.hpp

| 变更 | 说明 |
|------|------|
| 新增 `prev_velocity_` 成员 | 记录上一次速度值，用于检测方向切换 |
| 新增 `direction_change_time_` 成员 | 记录方向切换发生的时间戳 |
| 新增 `is_transitioning_` 成员 | 标记当前是否处于过渡期 |
| 新增 `transition_duration_` 成员 | 过渡持续时间（秒），从 ROS 参数读取 |

### 21.2 src/ackermann_control/src/vehicle_controller.cpp

| 变更 | 说明 |
|------|------|
| 构造函数声明 `transition_duration` 参数 | 默认 0.5 秒，可通过 YAML 配置 |
| `velocity_callback` 添加方向切换检测 | 当 `prev_velocity_ * new_velocity < 0` 且两者绝对值 > 0.01 时触发过渡 |
| `velocity_callback` 添加过渡期速度缩放 | 过渡期内后轮速度按 `elapsed / transition_duration_` 线性缩放（0→1） |

### 21.3 src/ackermann_control/config/ackermann_params.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `transition_duration` | (新增) | 0.5 | 方向切换过渡时间（秒），0 表示禁用。0.5s 对应前轮约 75% 响应时间（满打满转需 0.67s） |

---

## 二二、杜绝驱动轮在前前进（2026-05-23）：禁止正常导航时倒车

### 根因分析

Ackermann 小车在自动导航时以倒车姿态（驱动轮/后轮在前）长距离驶向目标。根因：RPP 控制器 `allow_reversing: true` 使得当 carrot 点在正后方时，控制器选择倒车前往而非前进掉头。同时 SmacPlannerHybrid 使用 `REEDS_SHEPP` 运动模型允许规划倒车路径段。

目标：**完全杜绝正常导航时的倒车姿态**，脱困恢复（BackUp）仍保留倒车能力。

### 22.1 config/nav2_params_exploration.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.FollowPath` | `allow_reversing` | `true` | `false` | **核心修复**：RPP 控制器不再选择倒车跟踪路径，当目标在后方时执行前进弧线掉头 |
| `planner_server.GridBased` | `motion_model_for_search` | `"REEDS_SHEPP"` | `"DUBIN"` | DUBIN 曲线只包含前进弧线，从规划层杜绝倒车路径 |
| `planner_server.GridBased` | `allow_reverse_expansion` | `true` | (删除) | DUBIN 模型不支持倒车扩展，移除该参数 |
| `planner_server.GridBased` | `reverse_penalty` | `5.0` | (删除) | DUBIN 模型无此参数 |

### 22.2 config/nav2_params_opentcs.yaml

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.FollowPath` | `allow_reversing` | `true` | `false` | 与探索模式一致，禁止倒车跟踪 |

### 工作机制

```
正常导航：规划器(DUBIN) → 只生成前进路径 → RPP(不倒车) → 前进掉头 → 永远正向行驶
脱困恢复：BT BackUp → 直接发 cmd_vel(负速) → cmd_vel_bridge → vehicle_controller → 可倒车
手动遥控：keyboard/joystick → 直接发 /velocity(负速) → vehicle_controller → 可倒车
```

### 不修改的文件

- `behavior_trees/ackermann_nav.xml`：BackUp 脱困行为保留
- `velocity_smoother` 的 `min_velocity: [-0.5,...]`：BackUp 负速度需要通过
- `cmd_vel_bridge.py`：透传负速度支持脱困
- `vehicle_controller.cpp`：底层保留倒车能力
- 遥控节点：手动遥控倒车不受限

---

## 二三、RS-485 底盘通信协议仿真（2026-05-23）

### 概述

新增 RS-485 底盘通信协议仿真层，在 Gazebo 仿真中复现实物控制卡的 485 串口通信链路。通过 socat 虚拟串口对模拟物理 RS-485 总线，使上位机软件同时适用于仿真和实物。

### 23.1 协议映射

| 485 通道 | 值范围 | 物理含义 | 转换公式 |
|----------|--------|----------|---------|
| CH1 转向 | 1000=左满, 1500=中, 2000=右满 | 弧度 [-0.5236, +0.5236] | `ch = 1500 + 500 × (angle / 0.5236)` |
| CH2 速度 | 1000=全后退, 1500=停, 2000=全前进 | m/s [-1.4, +1.4] | `ch = 1500 + 500 × (speed / 1.4)` |
| CH3-6 继电器 | 1000=断, 2000=通 | 二值 | 直通 |
| CH7-8 预留 | 1500 | - | 固定1500 |
| CH9-10 模拟量 | 1000-2000 | 0-5V | 直通 |

### 23.2 config/rs485_bridge.yaml

| 节点 | 参数 | 值 | 说明 |
|------|------|-----|------|
| `rs485_chassis_bridge` | `serial_port` | `/tmp/chassis_cmd` | 仿真用虚拟串口；实物改为 `/dev/ttyUSB1` |
| `rs485_chassis_bridge` | `baudrate` | 115200 | 与控制卡一致 |
| `rs485_chassis_bridge` | `refresh_interval_ms` | 200 | 协议要求 50-300ms |
| `rs485_chassis_bridge` | `timeout_ms` | 500 | 协议要求 500ms 超时 |
| `rs485_chassis_bridge` | `max_steering_angle` | 0.5236 | 与 ackermann_params.yaml 一致 |
| `rs485_chassis_bridge` | `max_velocity` | 1.4 | 与 ackermann_params.yaml 一致 |
| `rs485_chassis_receiver` | `serial_port` | `/tmp/chassis_recv` | 仿真用虚拟串口 |
| `rs485_chassis_receiver` | `baudrate` | 115200 | 与控制卡一致 |
| `rs485_chassis_receiver` | `timeout_ms` | 500 | 500ms 无帧则发布零值 |

### 23.3 数据流

```
Nav2 → cmd_vel_bridge → /steering_angle, /velocity → rs485_chassis_bridge
  → 编码485帧 → /tmp/chassis_cmd → [socat] → /tmp/chassis_recv
  → rs485_chassis_receiver → 解码485帧 → /rs485/steering_angle, /rs485/velocity
  → vehicle_controller (remap) → ros2_control → Gazebo
```

### 23.4 新增文件

| 文件 | 说明 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/rs485_protocol.py` | 纯协议库（帧编解码、通道-物理量映射） |
| `src/lidar_slam_nodes/lidar_slam_nodes/rs485_chassis_bridge.py` | 发送端节点（Float64→485帧→串口） |
| `src/lidar_slam_nodes/lidar_slam_nodes/rs485_chassis_receiver.py` | 接收端节点（串口→485帧→Float64） |
| `config/rs485_bridge.yaml` | RS-485 桥接参数配置 |
| `launch/sim_ackermann_rs485.launch.py` | 带 RS-485 协议仿真的导航 launch 文件 |

---

## 24. jvs-opentcs 车辆状态接口（2026/05/24）

### 24.1 背景

依据 jvs-opentcs ROS2 车辆状态与控制接口补充规范，新增 `opentcs_vehicle_node` 替代原 `opentcs_nav2_bridge`（仅 RS-485 场景），向 Sidecar 提供完整车辆运行状态。

### 24.2 新增节点

**opentcs_vehicle_node** (`src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py`)

| Topic | 方向 | 类型 | 频率 | 说明 |
|-------|------|------|------|------|
| `/goal_pose` | 订阅 | PoseStamped | - | Sidecar 下发导航目标 |
| `/odom` | 订阅 | Odometry | - | 速度提取 |
| `/amcl_pose` | 订阅 | PoseWithCovarianceStamped | - | AMCL 协方差（定位质量） |
| `/amcl_pose` | 发布 | PoseWithCovarianceStamped | 10Hz | TF→位姿，Sidecar 消费 |
| `/robot_state` | 发布 | std_msgs/String (JSON) | 1Hz | 29+字段完整状态 |
| `/battery_state` | 发布 | sensor_msgs/BatteryState | 1Hz | 仿真电量 |

### 24.3 新增参数 (`config/opentcs_vehicle.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vehicle_name` | "ackermann_robot" | 车辆名称，需与 JVS 一致 |
| `access_identity` | "ackermann_robot" | 机器人访问标识 |
| `namespace` | "" | ROS2 namespace |
| `domain_id` | 42 | ROS_DOMAIN_ID |
| `base_frame` | "body_link" | 机器人基座 frame |
| `map_frame` | "map" | 地图 frame |
| `pose_publish_rate` | 10.0 | 位姿发布频率 (Hz) |
| `status_sample_ms` | 1000 | 状态发布间隔 (ms) |
| `heartbeat_timeout_ms` | 30000 | 心跳超时 (ms) |
| `cancel_timeout_ms` | 5000 | 取消超时 (ms) |
| `max_speed` | 1.4 | 最大速度 (m/s) |
| `battery_sim_enabled` | true | 是否启用电池仿真 |
| `battery_sim_start_percent` | 95.0 | 仿真电池初始电量 (%) |
| `battery_sim_drain_rate` | 5.0 | 仿真电池衰减速率 (%/小时) |

### 24.4 状态机

- `state`: IDLE ⇄ WORKING ⇄ ERROR
- `dispatchStatus`: UNKNOWN → ACCEPTED → EXECUTING → SUCCEEDED/CANCELED/ABORTED
- `localizationStatus`: INITIALIZING → OK / DEGRADED / LOST（基于 AMCL 协方差）

### 24.5 修改文件

| 文件 | 改动 |
|------|------|
| `src/lidar_slam_nodes/setup.py` | 新增 opentcs_vehicle_node entry_point |
| `launch/sim_ackermann_rs485.launch.py` | 替换 opentcs_bridge → opentcs_vehicle |

注：`opentcs_nav2_bridge.py` 保留，`sim_ackermann_opentcs.launch.py` 不受影响。

---

## 25. openTCS Sidecar ROS2 侧补充开发（2026/05/25）

### 25.1 背景

依据 `docs/jvs-opentcs-ros2-sidecar-ros2-side-needs-2026-05-25.md` 需求文档，对 `opentcs_vehicle_node` 进行功能补充，解决安全信号为占位值、orderId 无法关联、电池仅仿真等问题。

### 25.2 修改文件

| 文件 | 改动 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py` | 全部功能变更 |
| `config/opentcs_vehicle.yaml` | 新增 12 个参数 |
| `config/nav2_params_opentcs.yaml` | 启用 collision_monitor |
| `launch/sim_ackermann_rs485.launch.py` | 添加 vehicle_name/namespace launch 参数 |

### 25.3 P0: orderId 关联

| 变更 | 说明 |
|------|------|
| `_goal_cb()` 解析 frame_id | 格式 `map/orderId=TO-xxx`，解析后还原 frame_id 为 `map` |
| `_result_cb()` 清空 orderId | goal 结束后清除 `_current_order_id` |
| 新参数 `goal_order_id_parse` | 默认 true，可关闭 frame_id 解析 |

### 25.4 P1: 安全信号 + 障碍物检测

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `emergency_stop_topic` | "" | 急停 topic（std_msgs/Bool），空=不订阅 |
| `safety_stop_topic` | "" | 安全停车 topic（std_msgs/Bool），空=不订阅 |
| `obstacle_detection_mode` | "collision_monitor" | `collision_monitor`/`scan`/`disabled` |
| `obstacle_scan_threshold` | 0.5 | scan 模式障碍物距离阈值 (m) |
| `obstacle_scan_angle_window` | 1.047 | scan 模式前方角度窗口 (rad, ~60°) |

**nav2_params_opentcs.yaml 修改**：
- `collision_monitor.FootprintApproach.enabled`: `False` → `True`
- `lifecycle_manager_navigation.node_names`: 添加 `'collision_monitor'`

### 25.5 P1: 实车电池接入

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `battery_real_topic` | "/battery_state_real" | 实车 BMS 电池 topic |

当 `battery_sim_enabled: false` 时订阅该 topic，1Hz 定时转发真实数据。

### 25.6 P2: 可配置订阅 + namespace

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `odom_topic` | "/odom" | 原硬编码 `/odom`，现可配置 |
| `amcl_subscribe_topic` | "/amcl_pose" | 原硬编码 `/amcl_pose`，现可配置 |

launch 文件新增 `vehicle_name` 和 `namespace` 参数，通过 ROS2 原生 namespace 机制支持多车。

### 25.7 P2: 位置字段

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `position_report_topic` | "" | 位置报告 topic（std_msgs/String JSON），空=不订阅 |

robot_state JSON 新增 `currentPosition`、`lastNodeId`、`nextPosition` 字段。

### 25.8 P3: 地图 checksum

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_yaml_file` | "" | 地图 YAML 文件路径，非空时计算 PGM SHA256 前 16 位 |

robot_state JSON 新增 `mapChecksum` 字段。

### 25.9 补充修复

| 变更 | 说明 |
|------|------|
| `maxSpeed` 加入 JSON | 原 `max_speed` 参数声明但未使用，现输出到 robot_state |
| heartbeat 超时检测 | `_goal_cb` 记录最后 goal 时间，`_check_localization` 每 2s 检查超时并 log warn |

### 25.10 config/opentcs_vehicle.yaml 完整新增参数

```yaml
# P0
goal_order_id_parse: true

# P1 安全
emergency_stop_topic: ""
safety_stop_topic: ""
obstacle_detection_mode: "collision_monitor"
obstacle_scan_threshold: 0.5
obstacle_scan_angle_window: 1.047

# P1 电池
battery_real_topic: "/battery_state_real"

# P2 可配置订阅
odom_topic: "/odom"
amcl_subscribe_topic: "/amcl_pose"

# P2 位置
position_report_topic: ""

# P3 地图
map_yaml_file: ""

# 自动恢复
error_auto_recover_ms: 10000   # ERROR状态持续10秒后自动恢复IDLE
```

---

## 九、Nav2 lifecycle_manager 启动失败修复 (2026-05-26)

### 9.1 问题

`sim_ackermann_rs485.launch.py` 启动后，在 RViz 中设置目标点小车不动。
原因：Nav2 导航生命周期节点未被 lifecycle_manager 激活。

### 9.2 根因

1. **Fast-DDS RMW 响应超时**：Gazebo 高负载（GUI 占 212% CPU）下，controller_server 的 `change_state` 服务响应在 RMW 层超时，导致 lifecycle_manager 放弃后续节点启动
2. **service_call_timeout 未生效**：标准 `navigation_launch.py` 不把 `params_file` 传给 lifecycle_manager 节点，所以 yaml 中的 `service_call_timeout: 30000` 从未加载
3. **route_server 无配置**：Nav2 Jazzy 的 `navigation_launch.py` 新增了 `route_server` 和 `docking_server`，但 nav2_params 中缺少 `route_server` 配置段

### 9.3 修改内容

| 文件 | 修改 | 原因 |
|------|------|------|
| `launch/navigation_custom.launch.py` | 新建自定义 navigation launch | lifecycle_manager 接收 configured_params，使 service_call_timeout 生效 |
| `launch/sim_ackermann_rs485.launch.py` | 用 navigation_custom 替换标准 navigation_launch | 使用自定义 launch |
| `launch/sim_ackermann_rs485.launch.py` | navigation 启动延迟 25s → 30s | 给 Gazebo 更多稳定时间 |
| `config/nav2_params_opentcs.yaml` | service_call_timeout: 30000 → 60000 | 增加超时容忍度 |
| `config/nav2_params_opentcs.yaml` | lifecycle_manager node_names 加入 route_server/docking_server | 与 navigation_launch.py 实际列表一致 |
| `config/nav2_params_opentcs.yaml` | 添加 route_server 配置段 | Nav2 Jazzy 新增节点需要默认配置 |
| `config/nav2_params_opentcs.yaml` | 添加 bond_timeout/attempt_respawn_reconnection | 增强生命周期管理鲁棒性 |

### 9.4 route_server 崩溃修复 (2026-05-26)

**日志**: `log/rs485_2026-05-26_15-01-10.log`

**错误**: `parameter_value_from failed for parameter 'route_files': No parameter value set` → exit code -6 (SIGABRT)

**根因**: route_server 配置段参数名和插件名全部错误：
- `route_files` → 不存在的参数，正确为 `graph_filepath`
- `GoalPose`/`TraverseRoute`/`Validator` → 不存在的插件，正确为 `DistanceScorer`/`DynamicEdgesScorer`/`AdjustSpeedLimit`/`ReroutingService`

| 参数 | 旧值（错误） | 新值（正确） | 原因 |
|------|-------------|-------------|------|
| `route_files` | `[]` | (删除) | 不存在的参数名，导致启动时抛出 InvalidParameterValueException |
| `base_frame` | (缺失) | `"body_link"` | 与项目其他节点一致 |
| `route_frame` | (缺失) | `"map"` | 全局坐标系 |
| `graph_filepath` | (缺失) | `""` | 暂无路由图文件，空值允许启动后通过 set_graph service 加载 |
| `graph_file_loader` | (缺失) | `"GeoJsonGraphFileLoader"` | 官方默认的 GeoJSON 图加载器 |
| `edge_cost_functions` | (缺失) | `["DistanceScorer", "DynamicEdgesScorer"]` | 官方默认边评分器 |
| `operations` | (缺失) | `["AdjustSpeedLimit", "ReroutingService"]` | 官方默认路由操作 |
| `GoalPose` 插件 | `nav2_route::GoalPoseOperation` | (删除) | 不存在的插件类 |
| `TraverseRoute` 插件 | `nav2_route::TraverseRouteOperation` | (删除) | 不存在的插件类 |
| `Validator` 插件 | `nav2_route::RouteValidator` | (删除) | 不存在的插件类 |

### 9.5 新增 route_graph_loader 节点 (2026-05-26)

**目的**：接收 Sidecar 发布的 GeoJSON 路由图，保存到文件并调用 route_server 的 `SetRouteGraph` 服务加载。

**数据流**：
```
Sidecar → /route_graph (std_msgs/String, Transient Local) → route_graph_loader
  → 保存 /tmp/route_graph.geojson → 调用 route_server/set_route_graph 服务
```

| 文件 | 说明 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/route_graph_loader.py` | 新节点 |
| `src/lidar_slam_nodes/setup.py` | 新增 entry_point |
| `launch/sim_ackermann_rs485.launch.py` | 35s 延迟启动（route_server 30s 启动后） |

**节点参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `graph_save_path` | `/tmp/route_graph.geojson` | GeoJSON 文件保存路径 |
| `set_graph_service` | `route_server/set_route_graph` | route_server 的 set_graph 服务名 |
| `service_timeout` | `30.0` | 等待服务就绪的超时 |

**接口文档**：`docs/jvs-opentcs-ros2-sidecar-route-graph-topic-2026-05-26.md`

---

## 26. lifecycle_manager_localization 启动修复 (2026-05-27)

### 根因

`sim_ackermann_rs485.sh` 启动后 RViz 提示 map 不存在。日志显示 `map_server` 成功加载地图但发送 lifecycle `change_state` 服务响应时 FastRTPS 超时：

```
[map_server] [WARN]: failed to send response to /map_server/change_state (timeout)
```

`lifecycle_manager_localization` 随后卡住，无法完成 Configure → Activate 流程。对比之前成功运行的日志，确认为间歇性问题，由 Gazebo 启动期间的高 CPU 负载触发。

### 26.1 launch/sim_ackermann_rs485.launch.py

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| localization TimerAction delay | 15s | 25s | 给 Gazebo 更多时间稳定，减少高负载下 FastRTPS 响应超时 |

### 26.2 config/nav2_params_opentcs.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `lifecycle_manager_localization.service_call_timeout` | 30000 | 60000 | 增加超时容忍度，与 navigation lifecycle_manager 一致 |
| `lifecycle_manager_localization.bond_timeout` | (缺失) | 10.0 | 添加 bond 超时检测 |
| `lifecycle_manager_localization.bond_heartbeat_period` | (缺失) | 0.1 | 心跳间隔 |
| `lifecycle_manager_localization.attempt_respawn_reconnection` | (缺失) | true | 允许节点断连后自动重连 |

---

## 27. 修复启动后 map 不存在 + 车辆不移动

日期：2026-05-27

### 27.1 新建 launch/localization_custom.launch.py

| 变更 | 原因 |
|------|------|
| 参照 navigation_custom.launch.py 模式创建 | 标准 localization_launch.py 不向 lifecycle_manager 传递 configured_params，导致 service_call_timeout 未生效，FastDDS RMW 超时后 map_server 卡在 inactive |
| lifecycle_manager_localization 接收 configured_params | 使 nav2_params_opentcs.yaml 中的 service_call_timeout: 60000 生效 |

### 27.2 launch/sim_ackermann_rs485.launch.py

| 变更 | 原因 |
|------|------|
| localization 改用 localization_custom.launch.py | 修复 map_server lifecycle 超时问题 |

### 27.3 launch/navigation_custom.launch.py

| 变更 | 原因 |
|------|------|
| bt_navigator 添加 remapping `goal_pose → _unused_goal_pose` | 禁用 bt_navigator 的 /goal_pose 订阅，避免与 opentcs_vehicle_node 和远程 opentcs_nav2_bridge 三路竞争导致导航目标被反复取消 |

---

## 28. 修复 Sidecar 与 Vehicle Node 话题不匹配 + Nav2 use_sim_time 缺失 (2026-05-27)

### 28.1 根因

Sidecar (`jvs_opentcs_ros2_sidecar`) 使用 `vehicle_name=ackermann_robot` 作为话题前缀，发布到 `/ackermann_robot/goal_pose`，订阅 `/ackermann_robot/robot_state` 和 `/ackermann_robot/battery_state`。但 `opentcs_vehicle_node` 的话题参数配置为非命名空间路径（`/goal_pose`、`/robot_state`、`/battery_state`），导致：

- Sidecar 发布的目标 → 无人接收（subscription count = 0）
- Vehicle node 发布的状态 → Sidecar 收不到

此外，Nav2 多个节点（planner_server、controller_server 等）缺少 `use_sim_time: True`，导致仿真时钟不同步，NavigateToPose action 接受目标后立即返回 SUCCEEDED（实际未移动）。

同时 `opentcs_nav2_bridge` 与 `opentcs_vehicle_node` 功能完全重复（都订阅 goal_pose → 调用 NavigateToPose），在同一 launch 中同时运行会导致同一个目标被发送两次到 Nav2。

### 28.2 config/opentcs_vehicle.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `goal_pose_topic` | `/goal_pose` | `/ackermann_robot/goal_pose` | 匹配 Sidecar 发布的话题名 |
| `robot_state_topic` | `/robot_state` | `/ackermann_robot/robot_state` | 匹配 Sidecar 订阅的话题名 |
| `battery_state_topic` | `/battery_state` | `/ackermann_robot/battery_state` | 匹配 Sidecar 订阅的话题名 |

注：`amcl_pose_topic` 和 `amcl_subscribe_topic` 保持 `/amcl_pose` 不变，Sidecar 从 `/ackermann_robot/robot_state` JSON 获取位姿，不直接订阅 amcl_pose。

### 28.3 config/nav2_params_opentcs.yaml

为以下节点添加 `use_sim_time: True`：

| 节点 | 原值 | 新值 | 原因 |
|------|------|------|------|
| `bt_navigator` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `controller_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `local_costmap` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `global_costmap` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `planner_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `smoother_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `behavior_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `waypoint_follower` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `velocity_smoother` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `collision_monitor` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `docking_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |
| `route_server` | 无 | `use_sim_time: True` | 仿真时钟同步 |

### 28.4 launch/sim_ackermann_opentcs.launch.py

| 变更 | 原因 |
|------|------|
| 移除 `opentcs_nav2_bridge` 节点及其 TimerAction | 与 `opentcs_vehicle_node` 功能完全重复（都订阅 goal_pose → 调用 NavigateToPose），同时运行导致同一目标被发送两次到 Nav2 |

### 28.6 删除 opentcs_nav2_bridge.py

| 文件 | 操作 | 原因 |
|------|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_nav2_bridge.py` | 删除 | 功能已被 opentcs_vehicle_node 完全覆盖，所有 launch 文件均已不引用 |
| `src/lidar_slam_nodes/setup.py` | 移除 entry_point | 对应源码已删除 |

### 28.5 遗留问题

- `/ackermann_robot/navigate_to_pose` action 无 server（Sidecar 是唯一 client），Sidecar 直接调用该 action 会失败。需通过话题方式（goal_pose）下发目标
- Route graph GeoJSON 边缺少 `start/end` 节点引用（均为 None），导致 `/compute_and_track_route` action 返回错误码 400
- Planner 使用 NavfnPlanner（自由空间规划），未接入 route_server 的图路径规划

---

## 29. 机器人名称可配置化：动态构建 Sidecar Topic (2026-05-27)

### 29.1 根因

Sidecar 端订阅 `/{vehicle_name}/amcl_pose` 等带机器人名称前缀的 topic，但 `opentcs_vehicle_node` 的 AMCL pose 发布在 `/amcl_pose`（全局 topic，无前缀），导致 sidecar 收不到位姿信息。同时 `goal_pose`、`robot_state`、`battery_state` 等 topic 中 `ackermann_robot` 是硬编码在 YAML 字符串里的，不同机器人需要手动修改多处配置。

附带修复：Nav2 AMCL 和 vehicle_node 都发布到 `/amcl_pose`（同一 topic 竞争），修改后 vehicle_node 发布到 `/{vehicle_name}/amcl_pose`，不再冲突。

### 29.2 src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py

| 变更 | 说明 |
|------|------|
| 调整参数声明顺序 | 先声明并读取 `vehicle_name`，再声明 sidecar topic 参数 |
| sidecar topic 默认值动态构建 | `amcl_pose_topic` → `/{vehicle_name}/amcl_pose`，`goal_pose_topic` → `/{vehicle_name}/goal_pose`，`robot_state_topic` → `/{vehicle_name}/robot_state`，`battery_state_topic` → `/{vehicle_name}/battery_state` |
| Nav2/内部 topic 保持不变 | `amcl_subscribe_topic` → `/amcl_pose`，`nav_action_name` → `/navigate_to_pose`，`odom_topic` → `/odom` |

### 29.3 config/opentcs_vehicle.yaml

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `amcl_pose_topic` | `/amcl_pose` | (移除) | 由代码从 vehicle_name 动态构建 |
| `goal_pose_topic` | `/ackermann_robot/goal_pose` | (移除) | 同上 |
| `robot_state_topic` | `/ackermann_robot/robot_state` | (移除) | 同上 |
| `battery_state_topic` | `/ackermann_robot/battery_state` | (移除) | 同上 |

只需配置 `vehicle_name: "ackermann_robot"`，所有 sidecar topic 自动添加前缀。YAML 中仍可显式设置这些 topic 参数覆盖默认值。

---

## 30. 统一 goal_pose 和 initialpose 话题命名 (2026-05-27)

### 30.1 根因

RViz 的 "2D Nav Goal" 工具发布到 `/goal_pose`，"2D Pose Estimate" 发布到 `/initialpose`，均为全局 topic 无 vehicle_name 前缀。而 `opentcs_vehicle_node` 订阅 `/ackermann_robot/goal_pose`，AMCL 订阅 `/initialpose`（C++ 硬编码）。多机器人场景下这些全局 topic 会混淆。

### 30.2 修改文件

#### RViz 配置文件

| 文件 | Topic | 旧值 | 新值 |
|------|-------|------|------|
| `config/nav.rviz` | initialpose | `/initialpose` | `/ackermann_robot/initialpose` |
| `config/nav.rviz` | goal_pose | `/goal_pose` | `/ackermann_robot/goal_pose` |
| `config/slam.rviz` | initialpose | `/initialpose` | `/ackermann_robot/initialpose` |
| `config/slam.rviz` | goal_pose | `/goal_pose` | `/ackermann_robot/goal_pose` |
| `config/explore.rviz` | goal_pose | `/goal_pose` | `/ackermann_robot/goal_pose` |

#### launch/localization_custom.launch.py

| 变更 | 说明 |
|------|------|
| 新增 `vehicle_name` launch argument | 默认 `'ackermann_robot'` |
| AMCL remappings 添加 `('initialpose', ['/', vehicle_name, '/initialpose'])` | 将 AMCL 的 initialpose 订阅 remap 到 `/{vehicle_name}/initialpose` |

#### launch/sim_ackermann_rs485.launch.py

| 变更 | 说明 |
|------|------|
| localization launch_arguments 添加 `'vehicle_name': LaunchConfiguration('vehicle_name')` | 将 vehicle_name 传递给 localization_custom |

### 30.3 Topic 统一结果

| Topic | 发布者 | 订阅者 |
|-------|--------|--------|
| `/ackermann_robot/goal_pose` | RViz + Sidecar | opentcs_vehicle_node |
| `/ackermann_robot/initialpose` | RViz | AMCL |
| `/ackermann_robot/amcl_pose` | opentcs_vehicle_node | Sidecar |
| `/ackermann_robot/robot_state` | opentcs_vehicle_node | Sidecar |
| `/ackermann_robot/battery_state` | opentcs_vehicle_node | Sidecar |
| `/amcl_pose`（内部） | Nav2 AMCL | opentcs_vehicle_node（取协方差） |

---

## 31. 目标点竞态条件修复 + RPP 绕圈问题 (2026/05/27)

### 31.1 问题描述

通过 `ros2 topic pub --once /ackermann_robot/goal_pose ...` 发布新目标点时，机器人没有切换到新目标，继续围着旧目标点转圈。

根因：
1. `_result_cb` 竞态条件：旧目标取消后的结果回调会覆盖新目标的 `_goal_handle` 和状态，导致节点认为没有活跃目标
2. RPP 控制器振荡：Ackermann 车辆无法原地旋转，但 `yaw_goal_tolerance: 0.25` (14.3°) 过于严格，导致机器人绕目标转圈无法到达

### 31.2 修复内容

**opentcs_vehicle_node.py** — 代际计数器防竞态：
- 添加 `_goal_generation` 单调递增计数器
- `_goal_cb` 每次收到新目标时递增代际
- `_goal_response_cb` 用闭包捕获代际传递给 `_result_cb`
- `_result_cb` 检查代际匹配，过期回调直接忽略
- `_cancel_goal_internal` 取消时立即清理 `_current_goal_id` 和 `_current_order_id`

**nav2_params_opentcs.yaml** — 放宽目标容差：
| 参数 | 旧值 | 新值 |
|------|------|------|
| `xy_goal_tolerance` | 0.25 | 0.30 |
| `yaw_goal_tolerance` | 0.25 | 0.50 |
| `use_rotate_to_heading` | (未设置) | false |

**ackermann_nav.xml** — 减少恢复重试：
- `number_of_retries`: 6 → 3

**opentcs_vehicle_node.py** — 防护增强：
- `_goal_response_cb` 添加 try-except，防止 `future.result()` 异常导致节点卡死
- `_goal_cb` 开头添加诊断日志，记录收到目标时的状态

---

## 32. 第二次导航无响应 — BT 时序 + 代价地图修复 (2026/05/27)

### 32.1 问题描述

第一次导航到 (13, 0) 成功后，第二次发送目标 (-13, 0) 时机器人无响应。日志显示：
- 规划器找到路径（RViz 可见终点在 (-13,0)），但路径起点不在机器人位置
- 控制器报 `Resulting plan has 0 poses in it`，5秒后中止
- BT 恢复被 `WouldAControllerRecoveryHelp` 条件跳过，没有重试

根因：
1. **全局代价地图阻挡**：机器人到达 (13,0) 后，障碍层在附近标记障碍物，膨胀层覆盖了机器人位置，规划器无法从机器人位置开始
2. **BT 时序问题**：`RateController hz="0.1"` 导致规划器 10 秒才更新一次，`PipelineSequence` 在规划完成前就 tick 了 FollowPath，发送空路径
3. **恢复被跳过**：空路径错误不触发恢复条件检查，BT 直接放弃

### 32.2 修复内容

**ackermann_nav.xml** — BT 行为树：
| 修改 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `RateController hz` | 0.1 | 1.0 | 标准Nav2值，每秒重规划，避免空路径时序问题 |
| `ComputePathToPose` 重试次数 | 1 | 2 | 增加规划重试机会 |
| `FollowPath` 重试次数 | 1 | 2 | 增加控制重试机会 |
| 恢复条件检查 | `Fallback{WouldA*RecoveryHelp...}` | 移除，改用 `ReactiveFallback` | 确保恢复动作始终执行，不被错误码条件跳过 |

**nav2_params_opentcs.yaml** — 全局代价地图障碍层：
| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `raytrace_max_range` | 3.0 | 5.0 | 扩大射线清除范围，减少陈旧障碍 |
| `obstacle_max_range` | 2.5 | 3.5 | 扩大障碍标记范围 |
| `obstacle_min_range` | 0.0 | 0.3 | 过滤近距离噪声，防止自身标记为障碍 |

---

## 33. 目标点地图边界校验（2026-05-27）：防止越界目标触发物理移动

### 33.1 背景

当 Sidecar 下发超出地图边界的目标坐标（如 x=115 而地图仅延伸到 ~50），`opentcs_vehicle_node` 直接将目标转发给 Nav2。Nav2 规划器立即报错 `"Goal Coordinates outside bounds"`，但 BT 的 recovery 行为（BackUp 3.5m、BackUp 2.0m）会让小车物理后退多次后才最终失败（最多 3 轮 × 2 次 BackUp = 6 次物理后退）。

### 33.2 方案

订阅 `/global_costmap/costmap`（OccupancyGrid）缓存地图元数据，在 `_goal_cb` 中发送 Nav2 目标前进行边界和代价验证。越界目标直接 REJECTED，不发送给 Nav2，小车完全不动。

### 33.3 新增参数 (`config/opentcs_vehicle.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_bounds_check_enabled` | true | 是否启用目标点边界检查 |
| `goal_bounds_tolerance` | 0.5 | 边界内缩容忍距离 (m)，目标点距地图边界小于此值也被拒绝 |
| `goal_reject_unknown_cost` | true | 是否同时拒绝未知/障碍区域上的目标点 |

### 33.4 验证逻辑

1. 检查全局代价地图缓存是否已建立（首次启动约 1 秒后可用）
2. 计算地图边界：`[origin_x, origin_x + width × resolution] × [origin_y, origin_y + height × resolution]`
3. 目标点必须在边界内缩 `goal_bounds_tolerance` 米的范围内
4. 若 `goal_reject_unknown_cost=true`，检查目标点所在格子代价值：-1（未知）或 >= 90（障碍）则拒绝
5. 验证失败：`DispatchStatus.REJECTED` + fault code `GOAL_OUT_OF_BOUNDS`，状态保持 IDLE
6. 代价地图缓存不可用时：降级通过，记录警告日志（30 秒节流），由 Nav2 自身处理

### 33.5 数据流

```
Nav2 global_costmap → /global_costmap/costmap (OccupancyGrid) → _global_costmap_cb → _costmap_info + _costmap_data
Sidecar → /ackermann_robot/goal_pose → _goal_cb → _validate_goal_in_costmap(缓存) → [通过] → NavigateToPose
                                                                  → [拒绝] → REJECTED + IDLE
```

### 33.6 修改文件

| 文件 | 改动 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py` | 新增 `_global_costmap_cb`、`_validate_goal_in_costmap`，`_goal_cb` 中插入验证，新增订阅和参数 |
| `config/opentcs_vehicle.yaml` | 新增 3 个参数 |
| `docs/param-tuning.md` | 本节 |

---

## 34. Ackermann 航向对齐：Reeds-Shepp 规划 + 多点转向 (2026/05/28)

### 34.1 问题描述

第 31 节的修复（放宽容差 + 禁用 rotate_to_heading）没有从根本上解决问题。机器人仍然围绕目标点绕圈，无法到达。根因分析：

1. **Navfn 规划器忽略运动学约束**：生成的路径包含 Ackermann 车辆无法执行的急转弯，接近目标时控制器无法跟踪
2. **RPP 控制器禁用倒车**：即使规划器生成了好的路径，控制器也无法跟随倒车段
3. **无航向对齐机制**：Ackermann 车辆不能原地旋转，当到达 xy 容差但航向偏差大时，只能绕圈

### 34.2 解决方案

三层修复：

**层1 — 规划器替换**：Navfn → SmacPlannerHybrid (REEDS_SHEPP)
- Reeds-Shepp 曲线包含倒车段，能规划出以正确航向到达目标的运动学可行路径
- 最小转弯半径 1.0m 匹配车辆参数（轴距 0.58m / tan(30°) ≈ 1.0m）
- 倒车惩罚系数 2.0，优先走前进路径，仅在对齐航向需要时使用倒车

**层2 — RPP 控制器调整**：
- `allow_reversing: true` — 允许跟随倒车段
- `yaw_goal_tolerance`: 0.50 → 0.80 rad — 让 Nav2 更容易判定"到达"
- `lookahead_dist`: 0.8 → 0.6 — 缩短前瞻距离，减少接近目标时的超调

**层3 — 航向对齐行为**（opentcs_vehicle_node.py 新增）：
- Nav2 目标成功后，检查航向偏差是否超过阈值
- 若超过，自动执行多点转向（前进-停车-倒车-停车 循环）
- 利用 Ackermann 运动学：前进时打 A 方向方向盘→航向转 A；倒车时打 B 方向方向盘→航向继续转 A
- P 控制器根据航向误差计算角速度，大误差时大转弯，小误差时微调

### 34.3 参数变更

**nav2_params_opentcs.yaml — 规划器**：

| 参数 | 旧值 | 新值 |
|------|------|------|
| `planner plugin` | `nav2_navfn_planner::NavfnPlanner` | `nav2_smac_planner::SmacPlannerHybrid` |
| `expected_planner_frequency` | 20.0 | 5.0 |
| `motion_model_for_search` | — | `REEDS_SHEPP` |
| `minimum_turning_radius` | — | 1.0 |
| `goal_heading_mode` | — | `FORWARD` |
| `reverse_penalty` | — | 2.0 |
| `non_straight_penalty` | — | 1.2 |
| `cost_penalty` | — | 10.0 |
| `tolerance` | 0.5 | 0.5（不变） |

**nav2_params_opentcs.yaml — RPP 控制器**：

| 参数 | 旧值 | 新值 |
|------|------|------|
| `allow_reversing` | false | true |
| `yaw_goal_tolerance` | 0.50 | 0.80 |
| `lookahead_dist` | 0.8 | 0.6 |
| `min_lookahead_dist` | 0.4 | 0.3 |
| `max_lookahead_dist` | 1.2 | 1.0 |

**opentcs_vehicle_node.py — 新增对齐参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_heading_alignment_enabled` | true | 是否启用航向对齐 |
| `goal_heading_alignment_tolerance` | 0.15 | 对齐精度 (~8.6°) |
| `alignment_forward_speed` | 0.3 | 前进速度 (m/s) |
| `alignment_reverse_speed` | 0.25 | 倒车速度 (m/s) |
| `alignment_angular_gain` | 2.0 | 角速度 P 增益 |
| `alignment_max_omega` | 0.8 | 最大角速度 (rad/s) |
| `alignment_forward_dist` | 0.8 | 每次前进距离 (m) |
| `alignment_reverse_dist` | 0.6 | 每次倒车距离 (m) |
| `alignment_max_iterations` | 5 | 最大前进-倒车循环次数 |
| `alignment_stop_duration` | 0.5 | 阶段间停车时间 (s) |
| `alignment_timeout` | 30.0 | 总超时 (s) |

### 34.4 对齐算法

```
状态机: FORWARD → STOP1 → REVERSE → STOP2 → FORWARD → ...

每步：
1. 计算航向误差 = normalize_angle(goal_yaw - current_yaw)
2. 若 |误差| < 0.15 rad → 对齐成功，报告 SUCCEEDED
3. 若迭代 >= 5 次或超时 → 放弃，报告 SUCCEEDED（尽力而为）
4. P 控制器: omega = clamp(gain × 误差, -max_omega, max_omega)
5. 前进阶段: cmd_vel = (forward_speed, omega)
6. 倒车阶段: cmd_vel = (-reverse_speed, omega)
   - cmd_vel_bridge 自动处理：倒车时反转方向盘方向
   - 效果：前进和倒车阶段航向都朝目标方向变化
```

### 34.5 对齐期间的应急处理

- 新目标到达 → 立即取消对齐，停车，处理新目标
- 对齐超时 (30s) → 尽力而为报告 SUCCEEDED
- 对齐命令发布到 /cmd_vel，经 velocity_smoother 和 collision_monitor 保障安全

### 34.6 修改文件

| 文件 | 改动 |
|------|------|
| `config/nav2_params_opentcs.yaml` | 规划器替换、RPP 参数调整、目标容差调整 |
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py` | 添加航向对齐状态机、cmd_vel 发布器、12 个新参数 |
| `docs/param-tuning.md` | 本节 |

---

## 35. 操作执行方案升级 (v1.0 → v2.0)

**日期**: 2026-05-29
**场景**: sim_ackermann_rs485
**文档**: `docs/ROS2-操作执行技术方案.md` v2.0

### 35.1 变更摘要

| 变更项 | v1.0 | v2.0 |
|--------|------|------|
| 通信模式 | ROS2 Topic (`std_msgs/String`) | ROS2 Action (LoadMaterials / UnloadMaterials) |
| 消息格式 | JSON 字符串 | MaterialSpec / MaterialActual 强类型消息 |
| 操作类型 | load / unload / charge | load / unload (物料操作) |
| 反馈能力 | 无 | Action Feedback (阶段 + 进度) |
| 取消能力 | 无 | Action Cancel 支持 |
| 包结构 | 未定义 | jvs_material_actions + jvs_operation_handler |

### 35.2 新增包

| 包名 | 类型 | 说明 |
|------|------|------|
| `jvs_material_actions` | ament_cmake | Action 定义 (LoadMaterials.action, UnloadMaterials.action, MaterialSpec.msg, MaterialActual.msg) |
| `jvs_operation_handler` | ament_python | 操作节点 (Action Server) + 硬件驱动层 (StubDriver / PlcDriver) |

### 35.3 新增参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| operation_handler | `namespace` | `""` | ROS2 命名空间，对应车辆名 |
| operation_handler | `driver_type` | `"stub"` | 驱动类型：stub(模拟) / plc(真实硬件) |
| operation_handler | `operation_timeout` | `60.0` | 单次操作超时(秒) |

### 35.4 Launch 集成

- 在 `sim_ackermann_rs485.launch.py` 中 opentcs_vehicle_node (30s) 之后，32s 延迟启动 operation_handler_node
- Action 名称：`/ackermann_robot/load_materials`、`/ackermann_robot/unload_materials`

---

## 36. 上下货 Action 仿真 GUI（JVS-VGA控制台）(2026-05-30)

**日期**: 2026-05-30
**场景**: sim_ackermann_rs485
**文档**: `docs/ROS2端上下货动作实现方案-2026-05-29.md`

### 36.1 概述

新增 PyQt5 GUI 应用（JVS-VGA控制台），在 Gazebo 仿真环境中模拟装货/卸货 Action Server。操作员通过界面查看物料信息并勾选完成状态。

### 36.2 新增包

| 包名 | 类型 | 说明 |
|------|------|------|
| `jvs_agv_material_msgs` | ament_cmake | 接口定义：MaterialSpec.msg, MaterialActual.msg, LoadMaterials.action, UnloadMaterials.action |
| `jvs_agv_material_actions` | ament_python | PyQt5 GUI + Action Server 节点 |

### 36.3 新增参数

**文件**: `config/material_action.yaml`

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| material_action_server | `vehicle_name` | `"ackermann_robot"` | 车辆名称，用于 goal 验证 |
| material_action_server | `action_timeout_ms` | `300000` | Action 超时(毫秒) |

### 36.4 Action 接口

| Action | Goal 关键字段 | Result 关键字段 | Feedback |
|--------|--------------|----------------|----------|
| `load_materials` | job_id, vehicle_name, materials[] | success, status, actual_loaded[], current_load[] | phase, progress, message |
| `unload_materials` | 同上 | actual_unloaded[] 替代 actual_loaded[] | 同上 |

### 36.5 新增文件

| 文件 | 说明 |
|------|------|
| `src/jvs_agv_material_msgs/` | 接口包（msg + action + CMakeLists） |
| `src/jvs_agv_material_actions/` | GUI 包（Action Server + Qt 界面） |
| `config/material_action.yaml` | GUI 节点参数 |

### 36.6 Launch 集成

| 文件 | 变更 |
|------|------|
| `launch/sim_ackermann_rs485.launch.py` | 添加 TimerAction(period=32.0) 启动 material_action_gui 节点 |

### 36.7 GUI 操作说明

- 窗口标题：JVS-VGA控制台
- 物料表格每行有复选框，操作员勾选实际完成的物料
- **提交**：全部勾选→SUCCEEDED，部分勾选→PARTIAL，全部未勾选→FAILED
- **全部失败**：忽略勾选，直接返回 FAILED
- **取消**：返回 CANCELLED

### 36.8 触摸屏优化 (2026-05-30)

**目的**: 将 GUI 界面从鼠标操作优化为触摸屏手指操作。

| 优化项 | 旧值 | 新值 | 原因 |
|--------|------|------|------|
| 全局基础字体 | 系统 ~10pt | 18px (~14pt) | 触摸屏最小可读字体 |
| 按钮字体 | 14px | 22px (~16pt) | 按钮文字需要更大更清晰 |
| 按钮最小高度 | 无限制 (~28px) | 48px | Material Design 触摸目标最小 48dp |
| 按钮内边距 | 8px 20px | 14px 32px | 增大点击热区 |
| 表格复选框 | 默认 ~13x13px | 自绘 40x40px + 整列热区 | 默认复选框手指无法精确点击 |
| 表格行高 | 默认 ~30px | 48px | 行高需容纳手指点击 |
| 表格字体 | 系统 ~10pt | 18px | 与全局字体一致 |
| 全选/全不选按钮 | fixedWidth:60px | minimumWidth:100px | 触摸目标至少 100px 宽 |
| 消息输入框高度 | 默认 ~24px | 40px | 手指点击更容易聚焦 |
| 进度条高度 | 默认 ~20px | 28px | 进度条更容易看到 |
| 窗口最小尺寸 | 700x600 | 900x700 | 触摸屏通常更大 |
| 布局间距 | 6px | 10px | 防止相邻控件误触 |
| 历史日志高度 | 120px | 160px | 触摸屏显示更多历史记录 |
| 滚动条宽度 | 默认 ~12px | 20px | 触摸屏滚动条更易拖拽 |

**新增文件**:

| 文件 | 说明 |
|------|------|
| `gui/styles.py` | 全局样式常量 + QSS 样式表 |
| `gui/checkbox_delegate.py` | 大号复选框委托（40x40px，整列热区） |

**修改文件**:

| 文件 | 修改内容 |
|------|----------|
| `gui/material_table.py` | 行高、列宽、复选框委托、工具栏按钮尺寸 |
| `gui/action_panel.py` | 按钮字体/padding、标签对齐 |
| `gui/status_bar.py` | 标签字体、进度条 |
| `gui/main_window.py` | 窗口尺寸、间距、历史日志高度 |
| `material_action_gui.py` | 应用全局样式表 |

### 36.9 布局重新设计 (2026-05-30)

**目的**: 解决 Action 信息占据大量垂直空间导致物料清单只能显示 1-2 行的问题。

**布局变更**: 从垂直堆叠改为左右分栏。

| 区域 | 旧布局 | 新布局 |
|------|--------|--------|
| 整体 | QVBoxLayout 垂直堆叠 | 顶部状态栏 + QHBoxLayout 左右分栏 |
| Action 信息 | QGroupBox + QFormLayout 8行，占 ~250px 高 | 左侧面板精简为 4 字段摘要 + [详情] 弹窗 |
| 物料清单 | 仅 ~100px 高，显示 1-2 行 | 右侧主区域，占 ~57% 面积，可显示 8-10 行 |
| 操作按钮 | 在物料清单下方，水平排列 | 左侧面板垂直排列，始终可见 |
| 操作历史 | QGroupBox 固定 160px | 可折叠，默认收起，点击展开 |
| Action 详情弹窗 | 无 | 新增 QDialog，通过 [📋详情] 按钮弹出 |

**新增文件**:

| 文件 | 说明 |
|------|------|
| `gui/action_detail_dialog.py` | Action 完整信息弹窗（8 字段 + 关闭按钮） |

**空间分配（900×700 窗口）**:

| 区域 | 宽度 | 高度 | 占比 |
|------|------|------|------|
| 左侧面板 | 250px | 全高 | 28% |
| 右侧物料表格 | ~650px | ~560px | **57%**（原 15%） |
| 右侧操作历史 | ~650px | ~140px（可折叠） | 15% |

## 37. Lifecycle 管理与保活机制重构 (2026/05/31)

### 37.1 问题

1. `use_respawn='False'`：所有节点崩溃后无法恢复
2. 自定义节点（cmd_vel_bridge, rs485_bridge, rs485_receiver, opentcs_vehicle_node, vehicle_controller, route_graph_loader）是 plain Node，无有序状态管理
3. TimerAction 固定延时启动，在慢机器上不够、快机器上浪费
4. 无保活/看门狗机制
5. 无 sim/real 参数化支持

### 37.2 解决方案

**LifecycleNode 转换**：6 个自定义节点全部转为 LifecycleNode

| 节点 | 语言 | 变更 |
|------|------|------|
| cmd_vel_bridge | Python | Node → rclpy_lifecycle.LifecycleNode |
| rs485_chassis_bridge | Python | Node → rclpy_lifecycle.LifecycleNode |
| rs485_chassis_receiver | Python | Node → rclpy_lifecycle.LifecycleNode |
| opentcs_vehicle_node | Python | Node → rclpy_lifecycle.LifecycleNode |
| route_graph_loader | Python | Node → rclpy_lifecycle.LifecycleNode |
| vehicle_controller | C++ | rclcpp::Node → rclcpp_lifecycle::LifecycleNode |

**三级 lifecycle_manager 架构**：

| Manager | 管理节点 |
|---------|---------|
| lifecycle_manager_localization | map_server, amcl |
| lifecycle_manager_navigation | controller_server, smoother_server, planner_server, route_server, behavior_server, velocity_smoother, collision_monitor, bt_navigator, waypoint_follower, docking_server |
| lifecycle_manager_custom (新增) | cmd_vel_bridge, rs485_chassis_bridge, rs485_chassis_receiver, opentcs_vehicle_node, route_graph_loader, vehicle_controller |

### 37.3 参数变更

| 参数 | 旧值 | 新值 | 位置 | 原因 |
|------|------|------|------|------|
| `use_respawn` | `'False'` | `'True'` | launch file → localization/navigation custom | 启用崩溃自动恢复 |
| `respawn` | 无 | `True` | 所有自定义 LifecycleNode | 启用崩溃自动恢复 |
| `respawn_delay` | 无 | `2.0` | 所有自定义 LifecycleNode | 防止快速重启循环 |
| `lifecycle_manager_custom` | 不存在 | 新增 | nav2_params_opentcs.yaml | 管理自定义 LifecycleNode |
| `service_call_timeout` | `60000.0` | **已移除** | 全部 lifecycle_manager | Nav2 Jazzy 的 lifecycle_manager 不读取此参数，属于无效配置 |
| `bond_timeout` | — | `10.0` | lifecycle_manager_custom | 心跳超时检测 |
| `attempt_respawn_reconnection` | — | `true` | lifecycle_manager_custom | respawn 后自动重连 |

### 37.4 新增文件

| 文件 | 用途 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/wait_for_topic.py` | Topic 就绪检测 readiness guard |
| `src/lidar_slam_nodes/lidar_slam_nodes/wait_for_service.py` | Service 就绪检测 readiness guard |
| `src/lidar_slam_nodes/lidar_slam_nodes/node_watchdog.py` | 运行时健康监控（发布 /system_health） |
| `config/watchdog.yaml` | Watchdog 监控配置 |
| `scripts/launch/real_ackermann_nav.sh` | 真实小车启动脚本 |

### 37.5 事件驱动启动序列（替换 TimerAction）

```
T=0     socat + gz_sim + bridge + robot_state_publisher + laser_tf + rviz2 + watchdog
  ↓ wait_for_topic(/scan)
spawn_robot + ekf
  ↓ wait_for_service(controller_manager)
load_controllers
  ↓ wait_for_topic(/joint_states)
rs485_receiver + vehicle_controller → 1s → rs485_bridge
  ↓ wait_for_tf(odom→body_link)
localization (AMCL + map_server)
  ↓ wait_for_topic(/amcl_pose)
navigation + lifecycle_manager_custom + cmd_vel_bridge + opentcs_vehicle
  ↓ wait_for_service(route_server/set_route_graph)
route_graph_loader
  5s → material_action_gui
```

### 37.6 Sim/Real 参数化

Launch argument `simulation:=True/False` 控制节点启停：

| 节点 | simulation=True | simulation=False |
|------|----------------|-----------------|
| socat, gz_sim, bridge | ✅ | ❌ |
| spawn_robot, load_controllers | ✅ | ❌ |
| rs485_receiver, vehicle_controller | ✅ | ❌ |
| rs485_bridge | ✅ (虚拟串口) | ✅ (物理串口) |
| cmd_vel_bridge, Nav2, opentcs_vehicle | ✅ | ✅ |
| use_sim_time | True | False |

---

## 38. 自定义 lifecycle_starter 替代 Nav2 lifecycle_manager（2026-05-31）

### 38.1 根因

`sim_ackermann_rs485.sh` 启动后，`lifecycle_manager_localization` 日志打印 `"Configuring map_server"` 后卡住不动（最长 286 秒），后续 AMCL 永远不会被 activate。

**根因**：Nav2 lifecycle_manager 的 `service_client.hpp` 使用无超时的 `spin_until_future_complete()`（第 98-127 行），在 Fast-DDS + Gazebo 高 CPU 负载下，DDS 服务请求可能卡在传输层数分钟。Nav2 lifecycle_manager 没有重试逻辑，`changeStateForNode()` 失败后直接 abort。

### 38.2 解决方案

创建自定义 `lifecycle_starter.py` 节点，提供**可控超时 + 自动重试 + 健康监控**的 lifecycle 状态转换，替代 Nav2 lifecycle_manager 管理 map_server、amcl 和所有自定义 Python LifecycleNode。

### 38.3 架构变更

```
之前（不稳定）：
  lifecycle_manager_localization  → map_server, amcl（Nav2 内置，无超时无重试）
  lifecycle_manager_custom        → cmd_vel_bridge, rs485_bridge, ...（Nav2 内置，无超时无重试）
  lifecycle_manager_navigation    → 10 个 Nav2 C++ 节点（保留）

之后（稳定）：
  lifecycle_starter_localization  → map_server, amcl（自定义，30s 超时 + 5 次重试 + 指数退避）
  lifecycle_starter_custom        → cmd_vel_bridge, rs485_bridge, ...（自定义，同上）
  lifecycle_manager_navigation    → 10 个 Nav2 C++ 节点（保留 Nav2 内置，C++ 支持 bond）
```

### 38.4 事件链变更

```
之前：
  wait_ekf_tf → localization_custom.launch.py (内含 lifecycle_manager_localization)
  wait_amcl (/amcl_pose) → navigation + lifecycle_manager_custom + ...

之后：
  wait_ekf_tf → map_server + amcl + lifecycle_starter_localization
                lifecycle_starter 执行: configure(map_server) → activate → configure(amcl) → activate
                成功后发布 /lifecycle_starter_localization/ready
  wait_for_topic(/lifecycle_starter_localization/ready) → navigation + lifecycle_starter_custom + ...
```

关键区别：不再依赖 `/amcl_pose`（需要 AMCL 定位收敛才有数据），而是依赖 lifecycle_starter 的 `ready` 信号（所有节点 active 后立即发布）。

### 38.5 新增节点参数

**lifecycle_starter_localization**（one-shot 模式，完成后退出）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `node_names` | `['map_server', 'amcl']` | 管理的 lifecycle 节点 |
| `configure_timeout` | 30.0 | 单次 configure 服务调用超时 (s) |
| `activate_timeout` | 30.0 | 单次 activate 服务调用超时 (s) |
| `max_retries` | 5 | 每个状态转换最大重试次数 |
| `retry_delay` | 2.0 | 重试间隔 (s)，实际使用指数退避 |
| `startup_delay` | 2.0 | 启动前等待 DDS 稳定 (s) |
| `monitor_period` | 0.0 | **禁用**（避免嵌套 spin_once 导致崩溃） |
| `starter_name` | `'localization'` | 标识符 |

**lifecycle_starter_custom**（one-shot + 两 pass 模式）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `node_names` | `['cmd_vel_bridge', 'rs485_chassis_bridge', 'rs485_chassis_receiver', 'opentcs_vehicle_node', 'route_graph_loader', 'vehicle_controller']` | 6 个自定义 LifecycleNode |
| `startup_delay` | 5.0 | 等待 RS485 链路节点先启动 |
| `monitor_period` | 0.0 | **禁用** |
| 其余参数 | 同 localization | — |
| `starter_name` | `'custom'` | 标识符 |

### 38.6 修改文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/lifecycle_starter.py` | 已存在 | 自定义 lifecycle starter 节点（之前创建） |
| `src/lidar_slam_nodes/setup.py` | 已有 | 已包含 lifecycle_starter entry_point |
| `launch/sim_ackermann_rs485.launch.py` | 修改 | 替代 localization_custom include + lifecycle_manager_custom |
| `config/nav2_params_opentcs.yaml` | 修改 | lifecycle_manager_localization/custom 段添加注释标记已弃用 |

### 38.7 lifecycle_starter 行为特性

1. **超时控制**：每个服务调用有独立超时，不会无限阻塞
2. **自动重试**：失败后重试最多 max_retries 次，重试间隔使用指数退避（2s → 4s → 8s → 16s → 32s）
3. **DDS 稳定等待**：startup_delay 让 DDS 服务发现在高负载下有足够时间完成
4. **两 pass 启动**（custom starter）：首次 pass 跳过不可用节点继续处理后续节点，5s 后重试失败节点，共 3 轮
5. **ready 信号**：发布 `/lifecycle_starter_{name}/ready` (Bool)，下游节点可精确等待
6. **one-shot 模式**：startup 完成后进程退出（monitor_period=0），避免嵌套 spin_once 与 rclpy.spin() 冲突
7. **变量命名**：`_managed_clients` 避免与 rclpy Node 内部 `_clients` (list) 冲突
8. **参数类型**：`node_names` 默认值 `['']`（非 `[]`），确保 ROS2 识别为 STRING_ARRAY 而非 BYTE_ARRAY


## 39. Material Action P0/P1 完善（2026-06-01）

**问题**：
1. `material_action_server.py` 无超时机制 — 配置声明 `action_timeout_ms: 300000` 但从未读取，execute 回调无限等待
2. Cancel 竞态 — `_cancel_requested` 和 `_result_event` 无原子保护，cancel 与 submit 可同时触发
3. Cancel 路径错误调用 `goal_handle.succeed()` 而非 `canceled()`
4. 缺少 TIMEOUT Result 状态
5. Feedback 阶段仅有 ACCEPTED/CHECKING，不够丰富

**修改**：

### material_action_server.py 核心重写
- 引入 `GoalState` 枚举（IDLE/ACTIVE/CANCELLING/SUBMITTING/TIMED_OUT/COMPLETED），用 `_goal_lock` 保护，消除竞态
- 新增超时机制：每轮询周期检查 `time.monotonic()`，超时后原子切换到 `TIMED_OUT`
- Cancel 路径改用 `goal_handle.canceled()`，确保客户端不收到"成功"
- 提交/取消方法（`submit_result`/`cancel_result`）加锁检查 `_goal_state == ACTIVE`
- 拆分终态处理为 `_handle_cancel()` / `_handle_timeout()` / `_handle_submit()`
- Feedback 扩展到 5 个初始阶段：ACCEPTED → CHECKING → MOVING_TO_STATION → MOVING_ACTUATOR → PICKING/PLACING

### 新增参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `action_timeout_ms` | 300000 (5min) | 默认超时 |
| `load_timeout_ms` | -1 | 装货超时，-1 用默认 |
| `unload_timeout_ms` | -1 | 卸货超时，-1 用默认 |

配置文件：`config/material_action.yaml`

### GUI 增强（P1）
- `status_bar.py`：阶段名中文映射（ACCEPTED→已接受、CHECKING→检查中等）
- `action_panel.py`：新增车辆名和实时阶段/进度标签
- `main_window.py`：feedback 更新 ActionPanel 阶段显示；区分超时/取消日志

### Result 状态语义
| 状态 | success | error_code | 触发条件 |
|------|---------|------------|----------|
| SUCCEEDED | true | 空 | 所有物料勾选 |
| PARTIAL | true | 空 | 部分物料勾选 |
| FAILED | false | LOAD_FAILED / UNLOAD_FAILED | 全部失败或操作员点"全部失败" |
| CANCELLED | false | 空 | 操作员或 Sidecar 取消 |
| TIMEOUT | false | MATERIAL_ACTION_TIMEOUT | 超过配置超时未完成 |

---

## 十一、scripts/tools/cleanup_ros2.sh 自杀 bug 修复 (2026-06-01)

### 11.1 问题现象

执行 `./scripts/launch/gazebo_opentcs_nav.sh` 后, 只看到 cleanup 输出, 没有 launch 输出,
直接回到 shell 提示符, 日志文件 (`log/gazebo_opentcs_nav_*.log`) 也没生成。

### 11.2 根因

`cleanup_ros2.sh` 中使用 `pkill -f "<pattern>"` 杀进程, 但 `pkill -f` 按**命令行字符串**匹配。

启动脚本 `gazebo_opentcs_nav.sh` 的命令行包含 `ros2 launch ...` 这段字符串 (还没执行, 只是写在脚本里)。
`pkill -f "ros2 launch"` 会**同时匹配到调用它的父 bash 进程** (因为父 bash 的命令行
=`bash gazebo_opentcs_nav.sh`, 里面就含 "ros2 launch"), 把脚本自己的 bash 杀掉, 导致
后续 `exec ros2 launch ...` 永远执行不到。

### 11.3 修复方案

新增 `pk()` 包装函数替代裸 `pkill`:

- 用 `pgrep -f "<pattern>"` 拿到匹配 PID
- 排除 `$$` (当前 shell) 和 `$PARENT_PID` (调用本脚本的父 shell)
- 用 `kill -<signal> <pid>` 逐个发信号 (避免 pkill -f + 显式 PID 同时使用的歧义)
- `-9` 自动转换为 `SIGKILL`, 其它使用 `SIGTERM`

将原脚本中所有 `pkill` / `pkill -9` 替换为 `pk pkill` / `pk pkill -9`。

### 11.4 验证

- 独立测试: `pk` 不会误杀调用者, 但能 SIGTERM/SIGKILL 杀真实匹配进程
- 集成测试: 在命令行包含 `ros2 launch` 的父 shell 中执行 `cleanup_ros2.sh`,
  输出正常到 `=== Cleanup complete ===`, 退出码 0, 调用者未被自杀

### 11.5 涉及文件

- `scripts/tools/cleanup_ros2.sh` (新增 `pk()` 函数, 全部 `pkill` 改为 `pk pkill`)

---

## 十二、scripts/launch/ 启动脚本重命名 (2026-06-01, vscode 断连根因修复)

### 12.1 问题现象

执行 `scripts/launch/gazebo_opentcs_nav.sh` 后, vscode remote-ssh 每次都断开连接。
但脚本能跑出 cleanup 输出, 也能看到 ros2 launch 启动了一段时间, 只是
"过一会儿 vscode 弹 Connection lost"。

### 12.2 根因 (经 Explore agent 实证分析)

`cleanup_ros2.sh` 用 `pkill -f "<pattern>"` 按**命令行字符串**匹配进程。
其中模式 `"gazebo"` 来自第 91/147 行的 `pk pkill -f "gazebo"`.

启动脚本**自身文件名**叫 `gazebo_opentcs_nav.sh`, 它的命令行
`bash scripts/launch/gazebo_opentcs_nav.sh` 包含子串 `gazebo`.

进程树:
```
vscode ptyHost
  └─ 集成终端 bash (--init-file ...shellIntegration-bash.sh)
       └─ bash scripts/launch/gazebo_opentcs_nav.sh   (P_SCRIPT, cmdline 含 "gazebo")
            └─ bash scripts/tools/cleanup_ros2.sh     (P_CLEANUP = $$)
```

`pk()` 只保护 `$$` 和 `PARENT_PID` (cleanup 的直接父进程).
标准链下 P_SCRIPT = PARENT_PID, 表面安全.
但 Explore agent 通过实测确认以下**边界 case** 让单层保护失效, P_SCRIPT 被 SIGKILL:
1. `gazebo_opentcs_nav.sh` 内部 `exec ros2 launch` 后, 旧 P_SCRIPT 被替换, PPID 链断裂
2. cleanup 被嵌套调用 (`bash -c "bash cleanup_ros2.sh"` 等)
3. cleanup 第二次执行时老的 ros2 launch 树还残留, PPID 已被 init 收养

P_SCRIPT 被 SIGKILL 后, P_TERMINAL (vscode 集成终端) 失去唯一前台任务, ptyHost
误判 pty 关闭, 引发 `Remote-SSH: Connection lost`, vscode 客户端断连.

### 12.3 修复方案: 重命名启动脚本 (从源头消除)

将 `scripts/launch/*.sh` 中所有含 cleanup 模式子串的命名改为**纯场景命名**:

| 旧名 | 新名 |
|------|------|
| `gazebo_explore.sh`        | `sim_explore.sh`        |
| `gazebo_opentcs_kernal.sh` | `sim_opentcs_kernal.sh` |
| `gazebo_opentcs_nav.sh`    | `sim_opentcs_nav.sh`    |
| `gazebo_opentcs_overview.sh` | `sim_opentcs_overview.sh` |
| `gazebo_slam.sh`           | `sim_slam.sh`           |
| `raspberry_explore.sh`     | `rpi_explore.sh`        |
| `raspberry_opentcs.sh`     | `rpi_opentcs.sh`        |
| `raspberry_opentcs_kernal.sh` | `rpi_opentcs_kernal.sh` |
| `raspberry_opentcs_nav.sh` | `rpi_opentcs_nav.sh`    |
| `raspberry_opentcs_overview.sh` | `rpi_opentcs_overview.sh` |
| `raspberry_slam.sh`        | `rpi_slam.sh`           |

**新名避开了全部 41 个 cleanup 模式子串** (`ros2`/`gazebo`/`gz`/`amcl`/`nav2`/`rviz2`/`controller`/`tf2`/`ekf`/`lifecycle`/`socat`/`chassis` 等).

**保留不动的业务标识符** (与 cleanup 无关, 改了会破坏业务):
- map 文件名 (如 `~/maps/raspberry_auto_map.yaml`)
- `vehicle_name:=raspberry_agv`

### 12.4 同步修改

- `scripts/launch/*.sh` (11 个) — 文件重命名 + 内部注释/用法行更新
- `docs/system-architecture.md` — 脚本清单表格 + 进程树注释
- `scripts/tools/cleanup_ros2.sh` — 顶部加 WARNING 块说明命名约束

第十一节历史记录**保留原样** (描述的 "旧名 gazebo_opentcs_nav.sh" 事实不变).

### 12.5 验证

- 模式扫描: 所有 scripts/launch/*.sh 都不含 cleanup 模式子串 ✅
- 语法检查: 11 个改过的脚本 + cleanup_ros2.sh 全部 `bash -n` 通过 ✅
- 不自杀测试: 命令行含 `sim_opentcs_nav` 字符串的父 shell 跑 cleanup,
  退出码 0, 输出到 `=== Cleanup complete ===`, 调用者未被自杀 ✅
- 端到端: 由用户在 vscode 集成终端跑 `bash scripts/launch/sim_opentcs_nav.sh`,
  预期 vscode 不断连

### 12.6 涉及文件

- `scripts/launch/gazebo_*.sh` (5 个) → `scripts/launch/sim_*.sh`
- `scripts/launch/raspberry_*.sh` (6 个) → `scripts/launch/rpi_*.sh`
- `scripts/tools/cleanup_ros2.sh` (顶部加 WARNING 注释)
- `docs/system-architecture.md` (脚本清单)
- 本文件 (本节新增)

---

## 十三、sim_* 脚本添加 ROS_LOCALHOST_ONLY=1 (2026-06-01, VSCode 断链第二轮修复)

### 13.1 问题现象

重命名修复 (第十二节) 后, 用户在仿真服务器上执行 `sim_opentcs_nav.sh`,
VSCode Remote SSH 仍在**启动过程中**断链 (cleanup 完成, Gazebo/Nav2 节点启动期间)。
服务器 32GB RAM, 排除 OOM。

### 13.2 根因

仿真脚本设置了 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (Fast-DDS), 但**未设置**
`ROS_LOCALHOST_ONLY=1`. 启动时 25+ 个 ROS2 节点同时上线, 每个节点向**所有网络接口**
发送 UDP 多播发现报文 (端口 7400/7401). 多播风暴涌入物理网卡 (SSH 使用的同一网卡),
挤占 SSH keepalive 数据包, 导致 SSH 超时 → VSCode "Connection lost".

断链时机与"启动过程中"吻合: 节点大量启动 = 多播突发高峰。

### 13.3 修复方案

在所有 **sim_* 仿真脚本**中, `RMW_IMPLEMENTATION` 之后添加:

```bash
export ROS_LOCALHOST_ONLY=1
```

**仅修改 sim_* 脚本** — 真实硬件脚本 (`rpi_*`, `real_*`, `rs485_nav.sh`) 不改,
它们可能需要跨机 DDS 通信。

**修改的脚本** (9 个):
- `sim_opentcs_nav.sh`, `sim_slam.sh`, `sim_explore.sh`, `sim_opentcs_kernal.sh`
- `sim_ackermann_nav.sh`, `sim_ackermann_rs485.sh`, `sim_ackermann_opentcs.sh`
- `sim_ackermann_slam.sh`, `sim_ackermann_explore.sh`

### 13.4 同步修改: cleanup_ros2.sh 加固

1. `pkill -f "rqt"` → `pkill -f "rqt_"` (SIGTERM + SIGKILL 共 2 处)
   原因: `"rqt"` 太宽泛, 可能匹配非 rqt 进程; 实际 rqt 工具名为 `rqt_gui` 等
2. `/dev/shm` 清理改为 `find ... -user $(id -u)` 限定当前用户, 避免误删其他用户文件

### 13.5 涉及文件

- `scripts/launch/sim_*.sh` (9 个) — 添加 `ROS_LOCALHOST_ONLY=1`
- `scripts/tools/cleanup_ros2.sh` — 收紧 rqt 模式 + 安全共享内存清理
- 本文件 (本节新增)

---

## 35. 修复 LifecycleNode 未激活导致机器人不动 (2026-06-01)

### 根因分析

多个 launch 文件中 `cmd_vel_bridge` 和 `vehicle_controller` 是 LifecycleNode，但没有配置 `lifecycle_starter` 来执行 configure+activate。节点启动后停留在 `unconfigured` 状态，不订阅任何话题、不发布任何数据。

**受影响的数据流**：
```
Nav2 controller → /cmd_vel → [cmd_vel_bridge: LifecycleNode] → /steering_angle + /velocity
  → [vehicle_controller: LifecycleNode] → /forward_*_controller/commands → Gazebo
```

两个节点都未激活 → Nav2 的速度命令无法传递到 Gazebo → 机器人永远不动。

### 35.1 受影响的 launch 文件

| 文件 | 缺失的 LifecycleNode |
|------|---------------------|
| `sim_ackermann_explore.launch.py` | `cmd_vel_bridge` + `vehicle_controller` |
| `sim_ackermann_nav.launch.py` | `cmd_vel_bridge` + `vehicle_controller` |
| `sim_ackermann_opentcs.launch.py` | `cmd_vel_bridge` + `vehicle_controller` |

### 35.2 修复方案

为每个受影响的 launch 文件添加 `lifecycle_starter_*` 节点，管理 `cmd_vel_bridge` 和 `vehicle_controller` 的生命周期。参照已正常工作的 `sim_ackermann_rs485.launch.py` 中的 `lifecycle_starter_custom` 模式。

| 文件 | 新增节点 | 管理的节点列表 | TimerAction |
|------|---------|--------------|-------------|
| `sim_ackermann_explore.launch.py` | `lifecycle_starter_explore` | `cmd_vel_bridge`, `vehicle_controller` | T+25s |
| `sim_ackermann_nav.launch.py` | `lifecycle_starter_nav` | `cmd_vel_bridge`, `vehicle_controller` | T+25s |
| `sim_ackermann_opentcs.launch.py` | `lifecycle_starter_opentcs` | `cmd_vel_bridge`, `vehicle_controller` | T+25s |

### 35.3 lifecycle_starter 参数

```yaml
node_names: ['cmd_vel_bridge', 'vehicle_controller']
configure_timeout: 30.0
activate_timeout: 30.0
max_retries: 5
retry_delay: 2.0
startup_delay: 5.0    # 等待 DDS 发现目标节点
monitor_period: 0.0    # 禁用健康监控（避免嵌套 spin_one 问题）
```

---

## 第四十轮: Launch & Scripts 分层架构重构 (2026-06-02)

### 40.1 动机

项目长期迭代后，launch 文件和 scripts 膨胀且职责重叠：
- 12 个 launch 文件中 5 个旧版使用 TimerAction 硬编码延迟
- 18+ 个 scripts 中多对功能完全重复（如 sim_slam.sh vs sim_ackermann_slam.sh）
- 只有 nav_main.launch.py 做了硬件 profile 抽象，其他场景仍各自独立

### 40.2 三层解耦架构

```
应用层 (3个场景入口)
  slam_main.launch.py    — 手工建图 (teleop + slam_toolbox)
  explore_main.launch.py — 自动探索建图 (frontier_explorer + Nav2)
  nav_main.launch.py     — 调度集成 (openTCS + AMCL + Nav2)

中间层 (复杂子系统)
  localization.launch.py — AMCL 定位 (ex-localization_custom)
  navigation.launch.py   — Nav2 导航栈 (ex-navigation_custom)

硬件抽象层 (4种 profile)
  hardware/gazebo_hardware.launch.py      — Gazebo 仿真
  hardware/rs485_hardware.launch.py       — RS-485 实车
  hardware/raspberry_hardware.launch.py   — 树莓派
  hardware/rplidar_s2l_hardware.launch.py — 纯激光雷达 (新增)
```

### 40.3 关键变更

| 变更 | 说明 |
|------|------|
| 新增 `rplidar_s2l` profile | 支持无底盘的纯激光雷达 SLAM 建图（笔记本电脑+RPLIDAR S2L），使用 rf2o 激光里程计替代 EKF |
| Profile YAML 新增 `sensing.type` 字段 | `ekf`（默认）或 `rf2o`，让应用层 launch 根据配置决定启动哪个传感器融合节点 |
| nav_main.launch.py 重构 | 移除内联的 map_server/amcl/lifecycle_starter_localization，改用 IncludeLaunchDescription 引用 localization.launch.py |
| Scripts 统一入口 | 合并 18+ 个重复脚本为 5 个：slam.sh / explore.sh / dispatch.sh / save_map.sh / teleop.sh |
| 删除旧文件 | 9 个旧 launch + 20 个旧 scripts 直接删除 |

### 40.4 删除的文件

**Launch 文件 (9个)**:
- `sim_ackermann.launch.py` → `slam_main.launch.py` (profile=gazebo)
- `sim_ackermann_nav.launch.py` → `nav_main.launch.py` (profile=gazebo)
- `sim_ackermann_explore.launch.py` → `explore_main.launch.py` (profile=gazebo)
- `sim_ackermann_opentcs.launch.py` → `nav_main.launch.py` (profile=gazebo)
- `sim_ackermann_rs485.launch.py` → `nav_main.launch.py` (profile=rs485)
- `real_slam.launch.py` → `slam_main.launch.py` (profile=rplidar_s2l)
- `rplidar_s2l.launch.py` → 内联到 `hardware/rplidar_s2l_hardware.launch.py`
- `localization_custom.launch.py` → `localization.launch.py` (重命名)
- `navigation_custom.launch.py` → `navigation.launch.py` (重命名)

**Scripts (20个)**: 合并为 5 个统一入口脚本 (slam/explore/dispatch/save_map/teleop)

### 40.5 新 Scripts 用法

```bash
# 手工建图
./scripts/launch/slam.sh                              # gazebo 仿真
./scripts/launch/slam.sh --profile rs485               # RS-485 实车
./scripts/launch/slam.sh --profile raspberry            # 树莓派
./scripts/launch/slam.sh --profile rplidar_s2l          # 纯激光雷达
./scripts/launch/slam.sh --no-teleop                    # 不启动遥控

# 自动探索建图
./scripts/launch/explore.sh                             # gazebo 仿真
./scripts/launch/explore.sh --profile raspberry

# 调度集成
./scripts/launch/dispatch.sh                            # gazebo 仿真
./scripts/launch/dispatch.sh --profile rs485 --map /path/map.yaml

# 保存地图
./scripts/launch/save_map.sh -f maps/my_map

# 键盘遥控 (独立终端)
./scripts/launch/teleop.sh
```