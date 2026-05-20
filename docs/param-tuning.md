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
