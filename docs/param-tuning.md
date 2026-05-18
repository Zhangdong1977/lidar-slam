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
