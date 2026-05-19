# 自动探索建图参数调整记录

> 调整日期：2026-05-16 (第一轮), 2026-05-18 (第二轮)  
> 基于日志：`log/explore_2026-05-15_19-22-02.log`  
> 问题摘要（第一轮）：探索过程中 279 次碰撞告警、45 次规划器超迭代、36 次 costmap 超时、34 次卡住，成功率仅 31%，最终地图仅 419×518 像素（约 21m×26m）  
> 问题摘要（第二轮）：第一轮调整后机器人仍频繁撞击障碍物，导航路径未考虑激光雷达数据。根因分析发现 `obstacle_min_range` 设置过大导致近场障碍物被忽略

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

### 第二轮 (2026-05-18)：修复激光雷达障碍物检测失效

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 近场障碍物被忽略 | 机器人频繁撞击障碍物，导航路径穿越墙壁/货架 | 2 项 | local `obstacle_min_range` 0.6→0.0, global `obstacle_min_range` 1.0→0.3 |
| 控制器无碰撞检测 | 机器人不减速冲向障碍物 | 2 项 | `use_collision_detection` ↑, `use_cost_regulated_linear_velocity_scaling` ↑ |
| 速度过高 | 狭窄过道制动距离不足 | 1 项 | `desired_linear_vel` 0.4→0.25 |
| 碰撞监视不完整 | FootprintApproach 预测关闭 | 1 项 | `FootprintApproach.enabled` ↑ |

---

## 四、config/explore_lite_params.yaml

### 4.1 frontier 代价函数（第三轮 2026-05-18）：修复机器人不走大片空地

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `explore_node.ros__parameters` | `potential_scale` | `3.0` | `1.0` | 减小距离惩罚，使机器人愿意走更远去大片未知区域而非就近选小 frontier |
| `explore_node.ros__parameters` | `gain_scale` | `1.0` | `2.5` | 增大面积奖励，大 frontier（空旷区域）获得更高优先级，优先探索开阔地带 |

**根因分析**：frontier 代价函数 `cost = potential_scale × distance - gain_scale × size`。`potential_scale=3.0` 使距离权重远超大小权重，机器人偏好近处小 frontier（贴着障碍物边缘），而不是远处的大片空地 frontier。Nav2 收到这个贴着障碍物的目标点后只能规划出经过障碍物附近的路径，导致频繁卡住。

### 第三轮 (2026-05-18)：修复机器人不走大片空地、路径贴障碍物

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| frontier 目标选择 | 机器人规避大片空地，路径贴着障碍物边缘，频繁卡住 | 4 项 | `potential_scale` 3.0→1.0, `gain_scale` 1.0→2.5, `xy_goal_tolerance` 0.1→0.5, `cost_penalty` 8.0→15.0 |

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

**根因分析**：日志显示 "Passing new path to controller" 每 ~3.35s 触发一次，由行为树 `<RateController hz="0.3">` 控制。每次新路径到达都会重置 RPP 控制器的内部跟踪状态。Ackermann 车辆倒车调头需要连贯的 steering+throttle 序列，被频繁打断后表现为：后轮仅轻微后退、前轮反复修正、机器人原地调整方向。由于新路径持续到达，控制器的 progress_checker 也被反复重置不会报错，最终只能靠 explore 节点的 `progress_timeout`(90s) 超时取消。

---

## 五、第五轮 (2026-05-18)：修复规划器超迭代、大规模代价地图搜索失败

### 5.1 planner_server.GridBased（第五轮）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `planner_server.ros__parameters.GridBased` | `downsample_costmap` | `false` | `true` | **核心修复**：开启代价地图降采样，搜索空间缩小 4 倍，使 SmacPlannerHybrid 能在大地图上找到路径 |
| `planner_server.ros__parameters.GridBased` | `downsampling_factor` | `1` | `2` | 降采样倍数，2 倍降采样 + 144 角度分箱 → 状态数从 4.3 亿降至 1.1 亿 |
| `planner_server.ros__parameters.GridBased` | `max_iterations` | `1000000` | `2000000` | 翻倍搜索预算，配合降采样后在同等时间内探索更大比例的状态空间 |
| `planner_server.ros__parameters.GridBased` | `analytic_expansion_ratio` | `3.5` | `4.0` | 增大分析扩展比例，在开阔区域跳过多余的图搜索节点 |
| `planner_server.ros__parameters.GridBased` | `analytic_expansion_max_length` | `3.0` | `5.0` | 增大分析扩展最大长度，在稀疏障碍物区域快速跳跃到目标方向 |

### 第五轮 (2026-05-18)：修复规划器超迭代

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 规划器超迭代 | "exceeded maximum iterations" 在 1507×1996 代价地图上反复失败，120 秒内 12 次规划全部失败，机器人长时间停止不动 | 5 项 | `downsample_costmap` 开启, `downsampling_factor` 2, `max_iterations` 翻倍, `analytic_expansion` 增大 |

---

## 六、第六轮 (2026-05-18)：修复碰撞检测瘫痪——局部代价地图膨胀半径过大

### 6.1 local_costmap.inflation_layer（第六轮）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `1.5` | `0.6` | **核心修复**：1.5m 膨胀半径在 6×6m 局部代价地图中造成大面积致命代价区，RPP 碰撞检测（前视 0.75m）持续命中致命单元，导致机器人 0.4s 内触发 "Controller patience exceeded"，完全无法移动。0.6m = 车体半长 0.45 + 安全余量 0.15，足够安全 |

### 第六轮 (2026-05-18)：修复碰撞检测瘫痪

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞检测瘫痪 | "collision ahead!" 每 50ms 触发一次，0.4s 后 "Controller patience exceeded"，explore 反复选择同一 1.17m 外的 frontier，20+ 次连续失败，机器人完全不动 | 1 项 | local `inflation_radius` 1.5→0.6 |

**根因分析**：碰撞检测本身正常工作——`max_allowed_time_to_collision_up_to_carrot: 3.0`（前视 0.75m）在检测到致命代价时正确触发警告。问题在于局部代价地图的 `inflation_radius: 1.5` 导致障碍物周围 1.5m 内布满膨胀代价，而 6×6m 窗口内几乎所有区域都被致命代价覆盖。当 frontier 目标仅 1.17m 远且位于已知/未知边界时，RPP 前视弧必然命中致命单元，控制器拒绝输出速度。第一轮曾将 local `inflation_radius` 从 1.0 降至 0.55，但后续被改回 1.5。0.6m 的膨胀半径（车体半长 0.45m + 0.15m 余量）在保证安全的同时，避免过度填充狭窄过道。

**根因分析**：代价地图扩张至 1507×1996 像素（约 75m×100m），SmacPlannerHybrid 使用 REEDS_SHEPP 运动模型 + 144 角度分箱，总状态数约 4.3 亿。100 万次迭代仅搜索了 0.23% 的状态空间。开启 2 倍降采样后状态数降至 1.1 亿，配合 200 万次迭代可探索约 1.8%。同时增大分析扩展参数使规划器在开阔区域快速跳过大片空白格子。

---

## 七、第七轮 (2026-05-18)：修复 Start Occupied 死锁——机器人与障碍物碰撞后永久卡死

> 基于日志：`log/explore_2026-05-18_21-43-34.log`

### 根因分析

机器人自主探索时与障碍物碰撞后永久卡死，循环报错 `Start occupied`（SmacPlannerHybrid 错误码 205）。问题分三层：

1. **如何进入障碍物**：控制器检测到碰撞 → `failure_tolerance: 5.0` 允许 5 次连续失败 → 每次失败清除 local costmap → 机器人逐渐向前蠕动 → 最终物理上进入障碍物内部
2. **为何无法恢复（核心 Bug）**：SmacPlannerHybrid 返回错误码 205（START_OCCUPIED），但 BT 的恢复门 `WouldAPlannerRecoveryHelp` 只检查 200/207/208 三个错误码，**不包含 205**。Fallback 门返回 FAILURE，整个恢复分支（包含 BackUp 倒车）被跳过
3. **explore 雪上加霜**：explore 节点收到 ABORTED 后立即将 frontier 加入黑名单 → 选下一个 → 同样 Start occupied → 也被黑名单 → 最终所有 frontier 被黑名单 → 永久死锁

### 7.1 behavior_trees/ackermann_nav.xml（第七轮）

| 参数位置 | 变更 | 调整原因 |
|----------|------|----------|
| 恢复门 Fallback | 添加 `AreErrorCodesPresent error_code="{compute_path_error_code}" error_codes_to_check="205;206"` | **核心修复**：START_OCCUPIED(205) 和 GOAL_OCCUPIED(206) 错误码被 WouldAPlannerRecoveryHelp 忽略，导致 BackUp 倒车恢复永远不执行 |

### 7.2 config/nav2_params_exploration.yaml（第七轮）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `controller_server.ros__parameters` | `failure_tolerance` | `5.0` | `2.0` | 减少允许连续失败次数，阻止机器人蠕动进入障碍物 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `inflation_radius` | `0.6` | `0.8` | 增大本地膨胀半径，提早避开障碍物 |
| `local_costmap.local_costmap.ros__parameters.inflation_layer` | `cost_scaling_factor` | `2.0` | `3.0` | 配合更大膨胀半径，使代价衰减更陡峭 |

### 7.3 explore_lite 源码修改（第七轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.h` | 添加 `consecutive_aborts_` 计数器和 `kMaxConsecutiveAborts=5` 常量 | 替代立即黑名单的逻辑，允许 BT 有机会执行恢复 |
| `explore.cpp reachedGoal()` | ABORTED 时递增计数器，仅连续 5 次 abort 后才黑名单 | 原代码首次 abort 即黑名单，导致 Start Occupied 时迅速耗尽所有 frontier |

---

## 八、第八轮 (2026-05-18)：修复 explore 二进制未更新 + frontier 选择振荡

> 基于日志：`log/explore_2026-05-18_22-45-25.log`

### 根因

1. **旧二进制未编译**：上一轮 colcon build 未检测到源码变更，运行的是 5 月 16 日的旧二进制，包含源码中不存在的 "Frontier too close" 功能
2. **centroid vs middle**：源码使用 `frontier->centroid`（常在机器人附近）而非 `frontier->middle`（更远的目标点），导致"瞬间到达"然后空等 progress_timeout
3. **same_goal 阻塞**：目标成功后 `prev_goal_` 未重置，`same_goal` 检查阻止发送新目标
4. **abort 振荡**：`planner_frequency=0.5`（每 2 秒重规划）导致每 4-6 秒 abort 当前导航，`kMaxConsecutiveAborts=5` 过低，正常重规划即触发黑名单 → 双向黑名单 → 远距离跳跃

### 8.1 explore 源码修改（第八轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.h` | `kMaxConsecutiveAborts` 5→20 | 允许更多正常重规划 abort，避免过早黑名单 |
| `explore.cpp` | `frontier->centroid` → `frontier->middle` | 用中点（更远）而非质心（常在机器人附近）作为目标 |
| `explore.cpp` | `RCLCPP_DEBUG` → `RCLCPP_INFO`（found frontiers, Sending goal） | 关键日志提升到 INFO 级别，便于调试 |
| `explore.cpp` | 成功后重置 `prev_goal_` | 解除 same_goal 阻塞，允许发送新目标 |

### 8.2 config/explore_lite_params.yaml（第八轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `planner_frequency` | 0.5 | 0.2 | 降低重规划频率（2s→5s），减少 abort 次数，给机器人更多时间完成当前导航 |
| `potential_scale` | 1.0 | 2.0 | 增加距离惩罚，减少追逐远距离大 frontier 导致的方向跳跃 |
| `gain_scale` | 2.5 | 2.0 | 略降低大小奖励，平衡距离和大小因素 |

### 第八轮 (2026-05-18)：修复 explore 编译和 frontier 振荡

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 旧二进制运行 | "Frontier too close" 消息不在源码中 | 强制清理 build/ 重新编译 |
| 目标点过近 | centroid (0.01,-0.05) 即刻到达 | `frontier->centroid` → `frontier->middle` |
| same_goal 阻塞 | 目标成功后 90 秒无新目标 | 成功后重置 `prev_goal_` |
| abort 振荡 | 19m 跳跃 SE↔SW，双向黑名单 | `kMaxConsecutiveAborts` 5→20, `planner_frequency` 0.5→0.2, 代价权重调整 |

---

## 九、第九轮 (2026-05-18)：修复 frontier 远距离跳跃 + 目标持久化

> 基于日志：`log/explore_2026-05-18_22-54-25.log`

### 根因

1. **代价函数失衡**：`min_distance`（米，2-20）和 `size`（格数，3000-60000）都乘 resolution(0.05)，距离项 0.1-1.0 vs size 项 150-3000，差距 1000 倍。无论怎么调 potential_scale/gain_scale，大 frontier 永远赢
2. **same_point 过严**：容差 0.01m，SLAM 更新使 frontier middle 移动 0.5-1.5m，导致同一 frontier 被识别为新目标 → abort 当前导航
3. **prev_goal_ 重置**：上一轮添加的成功后重置导致 same_goal 检查失效

### 9.1 源码修改（第九轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.cpp same_point()` | 容差 0.01m → 2.0m | SLAM 更新使 frontier middle 移动 0.5-1.5m，2.0m 容差确保同一 frontier 不被 abort |
| `explore.cpp reachedGoal()` | 移除成功后的 `prev_goal_` 重置 | 到达后 SLAM 更新会自然改变 frontier 排列，无需强制重置 |
| `frontier_search.cpp frontierCost()` | 移除 `min_distance × resolution` | min_distance 已是米制，再乘 0.05 无意义。只对 size 乘 resolution 转 m² |

### 9.2 参数调整（第九轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `potential_scale` | 2.0 | 5.0 | 配合代价函数修复，距离惩罚生效（距离项 10-100 vs size 项 100-3000） |
| `gain_scale` | 2.0 | 1.0 | 降低 size 权重，平衡距离 |

### 第九轮 (2026-05-18)：修复 frontier 远距离跳跃

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 代价函数失衡 | 远 17m frontier(size=6567) 总赢近 5m(size=3000) | 移除 distance × resolution，增大 potential_scale |
| 重规划 abort | 每 5 秒 abort 切换方向 | same_point 容差 0.01→2.0m + 恢复 prev_goal_ |

### 9.3 时间源崩溃修复（第九轮续）

**现象**：explore 节点启动后立即崩溃：`std::runtime_error: can't subtract times with different time sources [1 != 2]`

**根因**：`last_progress_`（`rclcpp::Time`）默认构造使用 RCL_ROS_TIME (source=1)，但 `use_sim_time: True` 时 `this->now()` 返回 SIM_TIME (source=2)。在 `makePlan()` 中 `this->now() - last_progress_` 减法要求同一时间源。

**修复**：添加 `bool progress_initialized_` 标志，首次成功赋值 `last_progress_` 时才设为 true，超时检查仅在已初始化时执行：

| 文件 | 改动 | 说明 |
|------|------|------|
| `explore.h` | 添加 `bool progress_initialized_ = false` | 新增标志位 |
| `explore.cpp makePlan()` | 赋值分支添加 `progress_initialized_ = true` | 首次赋值标记 |
| `explore.cpp makePlan()` | 超时检查前置 `progress_initialized_ &&` | 避免未初始化时做时间减法 |

### 9.4 首个目标被 same_goal 吞掉（第九轮续）

**现象**：explore 每 5 秒找到同一 frontier 但从不发送导航目标，机器人静止不动

**根因**：`prev_goal_` 初始值 (0,0,0) 与第一个 frontier middle (-1.05, 1.04) 距离 1.48m < same_point 容差 2.0m，被误判为"同一目标"跳过

**修复**：添加 `bool first_goal_sent_ = false`，`same_goal = first_goal_sent_ && same_point(...)`，发送目标后设 `first_goal_sent_ = true`

### 9.5 引入 navigating_ 状态锁（第九轮最终修复）

**现象**：机器人导航到 frontier A 途中，SLAM 发现更大的 frontier B（代价更低），timer 触发 makePlan() 后发送新目标到 B，Nav2 preempts 当前导航，机器人转向

**根因**：`same_point` 机制只能阻止发送**同一个** frontier（距离 < 2.0m），**完全无法阻止**发送一个**不同的** frontier。代码中没有任何"机器人是否正在导航"的状态追踪

**修复**：用 `bool navigating_` 标志替代整个 `same_goal`/`same_point` 机制：

| 位置 | 改动 |
|------|------|
| `explore.h` | `navigating_` 替换 `first_goal_sent_`，移除 same_goal 相关逻辑 |
| `makePlan()` 开头 | `navigating_==true` 时只检查 progress timeout（用机器人到目标的距离），不搜索新 frontier |
| `makePlan()` 发送目标时 | 设置 `navigating_=true`、`prev_goal_`、`prev_distance_`、`last_progress_` |
| `reachedGoal()` | 所有分支开头设 `navigating_=false` |
| `stop()` | 设 `navigating_=false` |

**行为变化**：
- 导航中 timer 触发 → 只检查是否卡住，不做 frontier 搜索，不发新目标
- 到达目标(SUCCEEDED) → navigating_=false → makePlan() 搜索新 frontier 并发送
- 卡住超时(90s) → cancel 当前目标、blacklist、navigating_=false → 搜索新 frontier
- ABORTED → navigating_=false → 等 timer 触发时自动搜索新 frontier

---

## 十、第十轮 (2026-05-19)：修复机器人卡死——无限循环发送同一目标点

> 基于日志：`log/explore_2026-05-19_09-19-22.log`

### 根因分析

机器人探索时卡死，explore节点每 0.5 秒循环：发送目标 → Nav2 瞬间"到达" → 再次选同一frontier → 重复 596 次。

**四个叠加的bug：**

1. **`gain_scale=1.0` 使巨型frontier永远被选中**：代价公式 `cost = potential_scale × min_distance - gain_scale × size × resolution`，frontier 0 (size=10430) 的 cost=-517.6，其他frontier均为正值(60+)，永远排第一
2. **Nav2瞬间判定到达**：规划器 tolerance=2.0m，目标距机器人仅 0.78m，控制器 0.2ms 内报告 "Reached the goal!"
3. **成功到达后不加黑名单**：`reachedGoal(SUCCEEDED)` 只调用 `makePlan()`，不加黑名单
4. **黑名单逻辑不一致**：存储的是 middle 点(-38.97, 7.07)，检查的是 centroid(-43.72, 14.25)，两者相距 7.8m，黑名单永远匹配不上

### 10.1 explore 源码修改（第十轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.h` | 添加 `geometry_msgs::msg::Point prev_centroid_` 成员 | 存储 frontier 质心，用于正确的黑名单匹配 |
| `explore.cpp makePlan()` | frontier 选择条件添加 `f.min_distance < 1.0` 跳过太近的frontier | 距机器人<1.0m 的 frontier 的 middle 点已在脚下，Nav2 瞬间判定到达 |
| `explore.cpp makePlan()` | 添加 `prev_centroid_ = frontier->centroid` | 存储质心供黑名单使用 |
| `explore.cpp reachedGoal()` | SUCCEEDED 分支添加 `frontier_blacklist_.push_back(prev_centroid_)` | 成功到达后加黑名单，防止重复选择同一 frontier |
| `explore.cpp makePlan()` progress timeout | `frontier_blacklist_.push_back(prev_goal_)` → `prev_centroid_` | 修复黑名单：用 centroid 而非 middle 存储，与 `goalOnBlacklist(f.centroid)` 检查一致 |

### 10.2 config/explore_lite_params.yaml（第十轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `gain_scale` | 1.0 | 0.5 | 降低巨型 frontier 的支配力，让距离因素更重要。frontier 0(size=10430) 的 size 项从 521.5 降至 260.8 |

### 10.3 config/nav2_params_exploration.yaml（第十轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `planner_server.GridBased.tolerance` | 2.0 | 0.75 | 规划器容差过大导致 0.78m 外的目标被瞬间"到达"，降至 0.75m 迫使规划器创建实际路径 |
| `controller_server.general_goal_checker.xy_goal_tolerance` | 0.5 | 0.35 | 更严格的目标到达判定，配合缩小的规划容差 |

### 第十轮 (2026-05-19)：修复机器人卡死无限循环

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 同一frontier被重复选择 | 596次发送同一目标(-38.97, 7.07) | 成功后加黑名单(用centroid)、最小距离过滤(<1.0m跳过)、gain_scale降低 |
| Nav2瞬间到达 | 控制器0.2ms报告"Reached the goal!" | planner tolerance 2.0→0.75, xy_goal_tolerance 0.5→0.35 |
| 黑名单失效 | centroid与middle相差7.8m，永远匹配不上 | 黑名单存储改为用centroid |

---

## 十一、第十一轮 (2026-05-19)：修复同一 frontier 因黑名单容忍度过小被重复选择

> 基于日志：`log/explore_2026-05-19_09-45-17.log`

### 根因分析

机器人探索时卡死，614 次发送同一目标 (-32.23, -28.47)，机器人完全不移动。三层叠加：

1. **二进制未更新（主因）**：源码最后修改 09:42，编译二进制停留在 18 日 23:26。第十轮的 `min_distance < 1.0` 过滤器和 `prev_centroid_` 黑名单修复未编译进运行中的二进制
2. **黑名单容忍度过小**：`goalOnBlacklist()` 使用 `5 × resolution = 0.25m` 容忍度匹配质心。同一物理 frontier 在 SLAM 更新后质心偏移 0.5-0.75m（如 (-39.45,-19.34)→(-40.09,-19.02)），远超 0.25m 阈值，黑名单形同虚设
3. **巨型 frontier 支配**：frontier 0 (size=7561) 代价 cost=-186，远超其他 frontier（frontier 1 cost=-175），始终被选中

### 11.1 explore 源码修改（第十一轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.cpp goalOnBlacklist()` | `tolerace` 5→40 (0.25m→2.0m) | 同一 frontier 质心在 SLAM 更新间偏移 0.5-0.75m，0.25m 容忍度无法匹配。2.0m 覆盖质心偏移且不误匹配不同 frontier（间距通常 >5m） |

### 11.2 编译修复（第十一轮）

| 操作 | 说明 |
|------|------|
| `colcon build --packages-select explore_lite --cmake-clean-cache` | 强制完全重编译，确保第十轮和第十一轮的所有源码修改生效 |

### 第十一轮 (2026-05-19)：修复黑名单容忍度过小

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 同一 frontier 被重复选择 | 614 次发送 (-32.23, -28.47)，机器人静止 | 黑名单容忍度 0.25m→2.0m、重编译 |
| 二进制未更新 | 源码比二进制新 10 小时 | `--cmake-clean-cache` 强制重编译 |

---

## 十二、第十二轮 (2026-05-19)：减少自动探索碰撞——双层修复

> 基于日志：`log/explore_2026-05-19_11-16-55.log`

### 根因分析

11 分钟运行中：562 次碰撞检测、47 次导航中止、35 次规划器超迭代、13 次后退失败，最终卡死。

**双重根因**：
1. **Nav2 参数**：local inflation_radius=0.8 + cost_scaling_factor=3.0 保护带过窄；velocity_smoother 峰值速度 1.0 m/s 制动距离过长；控制器 transform_tolerance=2.0 允许过时 TF
2. **Explore 源码**：frontier 目标选择时没有检查与已知障碍物的距离，导航目标点可能紧贴墙壁/障碍物

### 12.1 explore 源码修改（第十二轮）

| 文件 | 变更 | 调整原因 |
|------|------|----------|
| `explore.h` | 添加 `bool isTooCloseToObstacle(const Point& point)` 方法声明 | 新增障碍物距离检查 |
| `explore.h` | 添加 `double min_obstacle_distance_` 成员变量 | 可配置的最小障碍物距离阈值 |
| `explore.cpp` 构造函数 | 声明和获取 `min_obstacle_distance` 参数（默认 0.75m） | 通过 YAML 配置，无需重编译即可调整 |
| `explore.cpp` | 实现 `isTooCloseToObstacle()`：在目标点周围圆形区域内搜索 LETHAL_OBSTACLE 单元格 | 使用 SLAM 原始地图（无膨胀），检查真实障碍物位置 |
| `explore.cpp` makePlan() | 过滤条件添加 `isTooCloseToObstacle(f.middle)` | 跳过导航目标点周围 0.75m 内有障碍物的 frontier |

**性能**：0.75m / 0.05m = 15 格半径，31×31 = 961 单元格/frontier。100 个 frontier × 961 ≈ 96K 查询/5 秒，< 1ms。

### 12.2 config/explore_lite_params.yaml（第十二轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `min_obstacle_distance` | (新增) | `0.75` | frontier 导航目标点到最近障碍物的最小允许距离。机器人 footprint 最远端 0.45m，留 0.30m 安全余量 |

### 12.3 config/nav2_params_exploration.yaml（第十二轮）

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

### 第十二轮 (2026-05-19)：减少自动探索碰撞

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 碰撞检测频繁 | 562 次 "detected collision ahead" | 10 项参数 + 1 项源码 | inflation_radius ↑, cost_scaling_factor ↓, max_velocity ↓, 新增 isTooCloseToObstacle 过滤 |
| 导航频繁中止 | 47 次 abort，3 次黑名单 | 间接改善 | 控制器参数收紧 + frontier 障碍物距离过滤 |
| 规划器超迭代 | 35 次超迭代，目标不可达 | 间接改善 | global obstacle_min_range 归零 |
| 后退恢复失败 | 13 次 backup failed | 间接改善 | 碰撞减少后触发次数降低 |

---

## 十三、第十三轮 (2026-05-19)：优化 Ackermann 脱困/调头行为——消除前后振荡

> 问题：脱困和调头时前后移动幅度过小，频繁前进后退振荡

### 根因分析

Ackermann 小车遇到死胡同或需要调头时，RPP 控制器（`allow_reversing=true`）在航向偏差超过 90° 时发出倒车+转向命令，但由于倒车速度过低（-0.35 m/s），移动一点后 lookahead 点更新导致航向偏差回落到 90° 以下，RPP 切换回前进方向 → 小车又朝障碍物移动 → collision_detection 触发减速/停止 → 回到倒车 → **无限振荡**。

同时 `regulated_linear_scaling_min_speed=0.05` 和 `min_approach_linear_velocity=0.08` 导致在障碍物附近以极低速度蠕行（实际≈不动），`cost_scaling_factor=2.0` 使代价衰减不够快，大面积高代价区域持续降速。

### 13.1 config/nav2_params_exploration.yaml（第十三轮）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `velocity_smoother.ros__parameters` | `min_velocity` | `[-0.35, 0.0, -1.0]` | `[-0.5, 0.0, -1.0]` | 提高倒车最大速度，让小车在调头时有足够速度完成弧线动作，避免被RPP频繁切换方向 |
| `controller_server.FollowPath` | `min_approach_linear_velocity` | `0.08` | `0.15` | 提高最低速度，避免在障碍物附近以 0.08 m/s 蠕行（实际≈不动），至少保持可感知的移动 |
| `controller_server.FollowPath` | `regulated_linear_scaling_min_speed` | `0.05` | `0.15` | 提高最小巡航速度，costmap 高代价区域不再降到 0.05 m/s，保持有效移动能力 |
| `local_costmap.inflation_layer` | `cost_scaling_factor` | `2.0` | `4.0` | 加速代价衰减，远离障碍物的区域代价更低，减少不必要的减速区域面积 |
| `global_costmap.inflation_layer` | `cost_scaling_factor` | `2.0` | `4.0` | 同上，全局代价地图也加速衰减 |
| `planner_server.GridBased` | `reverse_penalty` | `2.0` | `1.5` | 降低倒车惩罚，鼓励规划出连贯的倒车调头路径而非多次方向切换 |

### 13.2 config/nav2_params_ackermann.yaml（第十三轮）

| 参数位置 | 参数名称 | 旧值 | 新值 | 调整原因 |
|----------|----------|------|------|----------|
| `velocity_smoother.ros__parameters` | `min_velocity` | `[-0.35, 0.0, -1.0]` | `[-0.5, 0.0, -1.0]` | 与探索模式保持一致 |
| `controller_server.FollowPath` | `min_approach_linear_velocity` | `0.05` | `0.15` | 与探索模式保持一致 |
| `controller_server.FollowPath` | `regulated_linear_scaling_min_speed` | `0.05` | `0.15` | 与探索模式保持一致 |
| `local_costmap.inflation_layer` | `cost_scaling_factor` | `1.0` | `2.0` | 加速代价衰减，减少减速区域 |
| `global_costmap.inflation_layer` | `cost_scaling_factor` | `1.5` | `3.0` | 同上 |

### 13.3 behavior_trees/ackermann_nav.xml（第十三轮）

| 参数位置 | 变更 | 调整原因 |
|----------|------|----------|
| `RecoveryActions.RoundRobin` 顺序 | 清地图→等3s→退2.5m→退1.0m 改为 **退3.5m→清地图→等2s→退2.0m** | 脱困时最需要的是立即后退，不应先浪费时间清地图和等待。后退前置可第一时间脱离障碍物 |
| `BackUp` 第一个 | `backup_dist=2.5, backup_speed=0.25` → `backup_dist=3.5, backup_speed=0.35` | 增大后退距离和速度，配合提高的 min_velocity(-0.5)，一次后退足够远离障碍物 |
| `BackUp` 第二个 | `backup_dist=1.0, backup_speed=0.2` → `backup_dist=2.0, backup_speed=0.3` | 同上 |
| `Wait` | `wait_duration=3` → `wait_duration=2` | 减少无效等待时间 |

### 第十三轮 (2026-05-19)：优化 Ackermann 脱困/调头行为

| 问题类别 | 日志表现 | 调整项数 | 核心改动 |
|----------|----------|----------|----------|
| 调头前后振荡 | 小车在死胡同频繁前进后退，移动幅度极小 | 6 参数 + BT | velocity_smoother 倒车速度 ↑, 最小速度 ↑, cost_scaling_factor ↑, reverse_penalty ↓, BackUp 前置+增大距离 |
| 障碍物附近蠕行 | 0.05 m/s 速度下实际未移动 | 2 参数 | min_approach_velocity ↑, regulated_min_speed ↑ |
| BT恢复效率低 | 脱困时先清地图等3s才后退 | BT重构 | BackUp 移到 RoundRobin 第一位 |

---

## 十四、openTCS 集成参数（2026-05-19）

### 概述

新增 `opentcs_nav2_bridge` 桥接节点，将 openTCS-NeNa 调度系统的 topic 接口与 Nav2 NavigateToPose action 对接。

### 14.1 桥接节点参数 (`opentcs_nav2_bridge`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_pose_topic` | `/goal_pose` | openTCS 发布的导航目标 topic (PoseStamped) |
| `amcl_pose_topic` | `/amcl_pose` | 发布给 openTCS 的机器人位置 topic (PoseWithCovarianceStamped) |
| `nav_action_name` | `/navigate_to_pose` | Nav2 导航 action 名称 |
| `target_frame` | `map` | TF 目标帧（用于读取机器人位置） |
| `source_frame` | `body_link` | TF 源帧 |
| `pose_publish_rate` | `10.0` | 位置发布频率 Hz，10Hz 足以满足调度系统跟踪需求 |

### 14.2 DDS 兼容性配置

| 环境变量 | 值 | 说明 |
|----------|-----|------|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 与 openTCS-NeNa IHMC Fast-RTPS 对齐，确保 DDS 发现兼容 |
| `ROS_DOMAIN_ID` | `42` | 当前环境默认值 42，openTCS-NeNa 默认=30，需在 Kernel Control Center 中改为 42 |

### 14.3 工厂世界关键坐标参考

充电站位置（来自 factory.sdf）：
| 名称 | ROS2 (x, y) m |
|------|---------------|
| charger_1 | (38, 38) |
| charger_2 | (41, 38) |
| charger_3 | (44, 38) |
| charger_4 | (38, 44) |
| charger_5 | (41, 44) |
| charger_6 | (44, 44) |

### 14.4 相关文件

| 文件 | 说明 |
|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_nav2_bridge.py` | 桥接节点 |
| `launch/sim_ackermann_opentcs.launch.py` | 集成 launch 文件（基于 sim_ackermann_nav + 桥接节点） |
| `scripts/launch/sim_ackermann_opentcs.sh` | 启动脚本 |

---

## 十五、第十五轮 (2026-05-19)：修复探索永久卡死——黑名单失效 + 虚假碰撞

### 概述

机器人在 Gazebo 仿真中探索约 60 分钟后卡在 (6.77, -32.79)，日志显示 344 次连续 abort。
根因：黑名单 middle/centroid 不匹配导致死循环 + `obstacle_min_range=0` 产生幽灵障碍物 +
`minimum_turning_radius` 偏小 65% 导致规划路径不可执行。

### 15.1 源码修复：黑名单 middle/centroid 不匹配

**文件**: `third-party/m-explore-ros2/explore/src/explore.cpp` 第 428 行

| 修改 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| ABORTED 分支黑名单存入点 | `frontier_goal`（= `frontier->middle`） | `prev_centroid_`（= `frontier->centroid`） | `goalOnBlacklist()` 检查的是 `f.centroid`，存 `middle` 导致永远匹配不上，同一不可达 frontier 被无限重选 |

其他存入点已正确使用 `prev_centroid_`：第 252 行（progress timeout）、第 422 行（goal succeeded）。

### 15.2 参数修复：消除虚假 "collision ahead"

**文件**: `config/nav2_params_exploration.yaml`

| 参数 | 位置 | 旧值 | 新值 | 原因 |
|------|------|------|------|------|
| `obstacle_min_range` | local costmap（第 98 行） | 0.0 | 0.5 | LiDAR 硬件最小量程 0.15m，设为 0 允许车身边缘回波/噪声被标记为障碍物，经 inflation 在 footprint 内产生 lethal cost，RPP 检查整个 footprint 不区分前后导致误报 |
| `obstacle_min_range` | global costmap（第 133 行） | 0.0 | 0.5 | 同上 |
| `minimum_turning_radius` | SmacPlannerHybrid（第 162 行） | 0.35 | 1.0 | 实际最小转弯半径 = wheel_base/tan(max_steer) = 0.58/tan(30°) = 1.004m，偏小 65% 导致规划路径包含机器人无法执行的急转弯 |

### 15.3 相关文件

| 文件 | 说明 |
|------|------|
| `third-party/m-explore-ros2/explore/src/explore.cpp` | 黑名单 bug 修复 |
| `config/nav2_params_exploration.yaml` | 三处参数调整 |
| `log/explore_2026-05-19_15-42-25.log` | 问题日志 |

---

## 十六、第十六轮 (2026-05-19)：替换探索器为 Ackermann 感知版本，修复方向不在前进方向

### 根因分析

Ackermann 机器人在自动探索时频繁倒退或侧向移动。根因：

1. **explore_lite 不考虑机器人朝向**：`orientation_scale: 0.0` 完全忽略机器人当前朝向，目标选择只看距离和面积
2. **目标朝向始终为 identity (w=1.0)**：explore_lite 发给 Nav2 的目标姿态永远是朝东（heading=0°），无论机器人面朝哪里
3. **SmacPlannerHybrid 允许倒车**：`REEDS_SHEPP` + `allow_reverse_expansion: true` + `reverse_penalty: 1.5`（过低），规划器对身后目标生成倒车路径
4. **RPP 控制器执行倒车**：`allow_reversing: true` 使控制器执行倒车段

项目中已有自定义 `frontier_explorer.py`（含朝向感知评分和 Ackermann 可行性过滤），但未被使用。

### 16.1 启动文件修改

**文件**: `launch/sim_ackermann_explore.launch.py`

| 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| 探索节点 | `explore_lite`（package=`explore_lite`, executable=`explore`） | `frontier_explorer`（package=`lidar_slam_nodes`, executable=`frontier_explorer`） | 使用 Ackermann 感知的自定义探索器，包含朝向评分和可行性过滤 |
| 参数文件 | `config/explore_lite_params.yaml` | `config/frontier_explorer_params.yaml` | 新建专用参数配置 |
| `map_saver_watcher` | 包含在 launch 中 | 移除 | `frontier_explorer.py` 内部已有 `save_map()` 方法，不需要外部 watcher。watcher 依赖 explore_lite 的 `/explore/status` 话题，新探索器不发布该话题 |

### 16.2 新建参数配置

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

### 16.3 Nav2 规划参数优化

**文件**: `config/nav2_params_exploration.yaml`

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `planner_server.GridBased.reverse_penalty` | 1.5 | 5.0 | 大幅惩罚倒车路径，配合朝向感知的探索器减少不必要的倒车 |

### 第十六轮 (2026-05-19)：替换探索器修复方向问题

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 机器人频繁倒退/侧向 | explore_lite 忽略朝向，目标朝向固定为东方 | 替换为 frontier_explorer（朝向评分 + Ackermann 可行性过滤） |
| 倒车路径成本低 | reverse_penalty=1.5 倒车仅比前进贵 50% | reverse_penalty 提升至 5.0 |

---

## 十七、第十七轮 (2026-05-19)：修复位置级卡死——前沿方向过滤+黑名单+位置检测

> 基于日志：`log/explore_2026-05-19_17-28-41.log`

### 根因分析

机器人成功完成 37 个目标后卡在 (30.91, -26.12)，连续 8 次尝试西侧前沿全部失败（移动仅 0-3mm）。三层叠加：

1. **`max_heading_diff` 过宽**：3.0 弧度 = 171.9°，允许 160°~177° 的目标通过。Ackermann 无法高效到达身后 170°+ 的目标
2. **黑名单距离过小**：`failed_centroids` 用 `min_goal_distance`(1.0m) 过滤，但卡住期间各前沿相距 7-13m，黑名单完全无效
3. **无位置级卡住检测**：在同一位置反复尝试不同前沿，没有机制识别"这个位置已经无法继续"

### 17.1 frontier_explorer.py 源码修改（第十七轮）

| 变更 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_heading_diff` 默认值 | 3.0 | 2.5 | 3.0(172°) 允许几乎正后方的目标，2.5(143°) 拒绝需要大角度掉头的目标 |
| 新增 `blacklist_radius` 参数 | — | 5.0m | 黑名单匹配距离从 1.0m 增大到 5.0m，同一区域不同前沿被归为同一不可达区域 |
| 新增 `stuck_position_count` 参数 | — | 3 | 连续 3 个目标在同一位置失败时触发位置级卡住检测 |
| 新增 `_update_stuck_position()` 方法 | — | 追踪连续失败的物理位置 | 位移 >2m 视为不同位置并重置计数 |
| `explore_step()` 添加位置级检查 | — | 达到阈值后清黑名单重试 | 给被误判的前沿一次重新评估机会 |
| `result_callback()` SUCCEEDED 重置 | — | 清零 `stuck_position` 和计数 | 成功导航后重置位置级追踪 |

### 17.2 config/frontier_explorer_params.yaml（第十七轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `max_heading_diff` | 3.0 | 2.5 | ~143°，拒绝需要大角度掉头的目标 |
| `blacklist_radius` | (新增) | 5.0 | 黑名单匹配半径，覆盖同一不可达区域的不同前沿 |
| `stuck_position_count` | (新增) | 3 | 连续失败触发阈值 |

### 第十七轮 (2026-05-19)：修复位置级卡死

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| 极端角度目标 | 160°~177° 目标通过 Ackermann 过滤 | `max_heading_diff` 3.0→2.5 |
| 黑名单无效 | 各前沿相距 7-13m，1.0m 黑名单无过滤 | `blacklist_radius` 5.0m |
| 位置级循环 | 同一位置 8 次尝试不同前沿全部失败 | `_update_stuck_position` + `stuck_position_count=3` |

---

## 十八、第十八轮 (2026-05-19)：让 Nav2 BT 恢复动作有机会执行——增大 stuck_timeout

### 根因分析

frontier_explorer 的 `stuck_timeout=10s` 在 Nav2 完成恢复前就取消了目标。时间线冲突：

| 组件 | 超时 | 动作 |
|------|------|------|
| frontier_explorer `stuck_timeout` | 10 秒 | 取消目标，换下一个前沿 |
| Nav2 `progress_checker.movement_time_allowance` | 60 秒 | 报告无进展 |
| Nav2 BT RecoveryNode | 6 轮 × ~20s | BackUp + ClearCostmap + Wait |

frontier_explorer 10 秒就取消 → Nav2 的 BackUp 恢复动作**从未执行** → 机器人永远无法通过后退脱困。

**正确分工**：后退恢复是 Nav2 BT 的工作。frontier_explorer 的 stuck 检测只是最终安全网，应等 Nav2 完成全部恢复尝试后再介入。

### 18.1 config/frontier_explorer_params.yaml（第十八轮）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `stuck_timeout` | 10.0 | 180.0 | 给 Nav2 足够时间完成完整恢复周期：progress_checker 60s + BT 6 轮恢复 × ~20s |
| `stuck_distance` | 0.15 | 0.5 | 配合更长的超时，0.5m 表示机器人确实在移动（而非噪声） |

### 第十八轮 (2026-05-19)：增大 stuck_timeout 让 Nav2 恢复生效

| 问题类别 | 日志表现 | 核心改动 |
|----------|----------|----------|
| Nav2 BT 恢复被抢占 | stuck_timeout=10s 在 Nav2 BackUp 执行前取消目标 | stuck_timeout 增至 180s，让 Nav2 完成全部 6 轮恢复 |

---

## 十九、第十九轮 (2026-05-19)：朝向过滤器导致角落处过早宣布完成

### 根因分析

机器人在地图东北角 (48.27, 47.99) 完成最后一个目标后，面向角落墙壁。所有未探索前沿都在机器人身后（朝向差 >143°）。`max_heading_diff: 2.5`（≈143°）作为**硬过滤器**，把所有身后前沿全部剔除 → `find_frontiers()` 返回空 → 连续 10 次无前沿 → 错误宣布"探索完成"。

实际上地图仍有一半区域未探索。

### 19.1 frontier_explorer.py — 两轮过滤策略

**代码改动**（`find_frontiers()` 方法）：

旧逻辑：`is_ackermann_feasible()` 硬过滤所有朝向差 > `max_heading_diff` 的目标。

新逻辑：分两个桶收集候选目标：
1. **严格桶**（`strict_goals`）：朝向差 ≤ `max_heading_diff`（优先）
2. **宽松桶**（`relaxed_goals`）：朝向差 > `max_heading_diff`，重算分数（去掉朝向惩罚）

优先返回严格桶；如果严格桶为空，回退到宽松桶，并打印日志 `"No heading-feasible frontiers, relaxing heading constraint for N behind-robot frontiers"`。

**效果**：
- 正常情况下（前方有前沿）→ 行为不变，优先选择前方目标
- 在角落/死胡同（前方无前沿）→ 放宽朝向约束，接受身后目标，让 SmacPlannerHybrid REEDS_SHEPP 规划掉头路径
- 朝向仍通过 `heading_weight` 作为**评分偏好**，不会倒退到原来 explore_lite 的无朝向感知状态
