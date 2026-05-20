# openTCS-NeNa 调度对接说明

> 本文档供 openTCS 服务端对接人员参考，包含机器人端 ROS2 接口的全部技术细节。

---

## 1. 系统架构

```
┌─────────────────────────┐         ROS2 Topic          ┌──────────────────────────────┐
│     openTCS 服务端       │                              │     机器人端 (ROS2 Jazzy)     │
│  (openTCS-NeNa Java)    │                              │                              │
│                         │  /goal_pose (PoseStamped)    │  opentcs_nav2_bridge         │
│  goalPublisher  ───────────────────────────────────────▶  → NavigateToPose action   │
│                         │  /initialpose                │  → Nav2 导航                  │
│  initialPosePub ──────────────────────────────────────▶                              │
│                         │                              │                              │
│  amclPoseSub   ◀──────────────────────────────────────  /amcl_pose                   │
│                         │  (PoseWithCovarianceStamped) │  ← TF (map→body_link) 10Hz  │
│                         │                              │                              │
│  navStatusSub  ◀──────────────────────────────────────  /navigate_to_pose/_action/  │
│                         │  (GoalStatusArray)           │    status                    │
│                         │                              │  ← Nav2 action server        │
└─────────────────────────┘                              └──────────────────────────────┘
```

### 机器人端节点链

```
Gazebo
  → ros_gz_bridge (传感器 + /tf, /scan_raw→/scan)
  → ackermann_control (底盘控制)
  → static_tf (body_link → ackermann_robot/body_link/lidar)
  → EKF (odom+imu → odom→body_link TF)
  → AMCL + map_server (加载 auto_exploration_map, 提供 map→odom TF)
  → Nav2 navigation (规划+控制)
  → cmd_vel_bridge (Twist → Ackermann 转向)
  → opentcs_nav2_bridge (/goal_pose → Nav2 action, TF → /amcl_pose)
```

---

## 2. ROS2 通信接口

### 2.1 openTCS → 机器人（openTCS 发布）

| Topic | 消息类型 | 方向 | 说明 |
|-------|---------|------|------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | openTCS → 机器人 | 导航目标点，frame_id 为 `map`，坐标单位为米 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | openTCS → 机器人 | 设置机器人初始位姿，frame_id 为 `map`。**注意：本项目机器人始终从原点 (0,0) 启动，AMCL 已预配置初始位姿 (`set_initial_pose: true`)，此 topic 实际无需发送** |

### 2.2 机器人 → openTCS（机器人发布）

| Topic | 消息类型 | 方向 | 说明 |
|-------|---------|------|------|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 机器人 → openTCS | 机器人实时位置，10Hz（`pose_publish_rate: 10.0`），frame_id 为 `map` |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Nav2 → openTCS | 导航任务状态（STATUS_UNKNOWN=0, ACCEPTED=1, EXECUTING=2, SUCCEEDED=4, CANCELED=5, ABORTED=6） |

---

## 3. 网络配置

| 配置项 | 值 | 说明 |
|--------|-----|------|
| **ROS_DOMAIN_ID** | **42** | 当前机器人环境固定为 42，openTCS-NeNa 需在 Kernel Control Center 中将车辆 Domain ID 从默认 30 改为 42 |
| DDS 中间件 | Fast-RTPS (`rmw_fastrtps_cpp`) | 机器人端使用 Fast-RTPS（启动脚本中 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`）；openTCS-NeNa IHMC 库也基于 Fast-RTPS |
| Namespace | 空（单车）/ `robotN`（多车） | 单车时为空字符串；多车时每个机器人分配独立 namespace，topic 变为 `/robotN/goal_pose` 等 |

---

## 4. 地图与坐标系

### 4.1 地图参数

| 项目 | 值 |
|------|-----|
| 地图文件 | `maps/auto_exploration_map.yaml` + `auto_exploration_map.pgm` |
| 地图分辨率 | 0.05 m/像素 |
| 地图原点 | (-50.200, -50.286) m |
| 地图尺寸 | ~100m × 100m (2000×2000 像素) |
| occupied_thresh | 0.65 |
| free_thresh | 0.196 |
| mode | trinary |

### 4.2 坐标系

| 项目 | 值 |
|------|-----|
| 坐标系 | `map` frame |
| 坐标单位 | 米（ROS2 标准） |
| 坐标范围 | X: -50 ~ +50 m, Y: -50 ~ +50 m |
| 机器人初始位置 | (0, 0, 0) m, yaw=0 |
| 航向约定 | 标准 ROS2：X 正方向 yaw=0，逆时针为正 |

### 坐标转换

openTCS 内部使用毫米，通过 `UnitConverterLib` 和 `ScaleCorrector` 转换：

```
ROS2坐标(m) = openTCS坐标(mm) / 1000 × plantModelScale
```

**重要**：请确认 `ros2.adapter.plantModelScale` 的值。如果 scale=1.0，则 openTCS 坐标直接对应毫米（1mm = 0.001m）；如果 scale=0.1（默认），则 openTCS 坐标需要放大 10 倍。

---

## 5. 机器人参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 车身尺寸 | 0.9m × 0.6m | 长 × 宽 |
| Footprint | [[0.45,0.30], [0.45,-0.30], [-0.45,-0.30], [-0.45,0.30]] | 用于 Nav2 碰撞检测 |
| 最大速度 | 0.5 m/s | 直线（`desired_linear_vel`） |
| 最大倒车速度 | 0.5 m/s | |
| 最大转向角 | ±30° (±0.5236 rad) | 阿克曼转向，不能原地旋转 |
| 最小转弯半径 | ~0.35 m | |
| 轴距 | 0.58 m | |
| 导航目标容差 | ±0.25 m, yaw ±0.25 rad | `xy_goal_tolerance` + `yaw_goal_tolerance` |
| 帧名称 | `body_link` | 机器人基座 frame |
| Laser frame | `ackermann_robot/body_link/lidar` | 静态 TF，z=0.22m |
| 控制器 | RegulatedPurePursuit | 阿克曼兼容，支持倒车 (`allow_reversing: true`) |
| 控制器频率 | 20 Hz | |
| Lookahead 距离 | 0.4 ~ 1.2 m | |

---

## 6. 工厂世界关键坐标（map frame，单位米）

### 6.1 充电站（NE 角，绿色地板区域）

| 名称 | X (m) | Y (m) | 备注 |
|------|-------|-------|------|
| charger_1 | 38 | 38 | 充电地板中心约 (42, 42) |
| charger_2 | 41 | 38 | |
| charger_3 | 44 | 38 | |
| charger_4 | 38 | 44 | |
| charger_5 | 41 | 44 | |
| charger_6 | 44 | 44 | |

### 6.2 区域中心点

| 区域 | X (m) | Y (m) | 特征 |
|------|-------|-------|------|
| NW（西北仓储） | -35 | 25 | 货架区 |
| NE（东北充电） | 30 | 35 | 充电站、开阔地 |
| SW（西南仓储） | -35 | -25 | 货架区 |
| SE（东南仓储） | 30 | -25 | 货架区 |
| CW（中北工位） | 0 | 25 | 桌子、桶 |
| CC（中南走廊） | 0 | -25 | 桌子 |
| QC（质检区） | 17 | 5 | 桶 |

### 6.3 道路交叉口

| 位置 | X (m) | Y (m) |
|------|-------|-------|
| 中心十字 | 0 | 0 |
| 北侧十字 | -15 / 15 | 25 |
| 南侧十字 | -15 / 15 | -25 |
| 外围道路 | ±48 | ±48 |

---

## 7. 启动流程

### 7.1 机器人端

```bash
bash scripts/launch/sim_ackermann_opentcs.sh
```

启动后约 30 秒，所有节点就绪。桥接节点日志输出：
```
[opentcs_nav2_bridge] opentcs_nav2_bridge started: goal=/goal_pose, amcl=/amcl_pose, ...
```

**节点启动时序：**

| 延迟 | 节点 | 说明 |
|------|------|------|
| 0s | Gazebo, ros_gz_bridge（含 /tf 桥接 + /scan 重映射）, laser_tf, cmd_vel_bridge, RViz2 | 基础环境 |
| 2s | ackermann_control | 底盘控制 |
| 5s | EKF | 里程计融合 |
| 15s | localization（map_server + AMCL） | 加载 `auto_exploration_map`，AMCL 定位 |
| 25s | Nav2 navigation | 规划 + 控制 |
| 30s | opentcs_nav2_bridge | openTCS 桥接 |

### 7.2 openTCS 端

1. 启动 Kernel + Control Center + Plant Overview
2. 加载 plant model（定义点位、路径、车辆）
3. 在 Control Center 中为车辆设置：
   - **Namespace**: 空（单车）
   - **Domain ID**: **42**
   - **Initial Position**: 选择起点对应的 plant model point
4. 启用车辆驱动
5. 从 Plant Overview 发送运输订单

---

## 8. 对接验证步骤

### 第一步：确认网络互通

机器人端启动后，在 openTCS 所在网络（同一网段 + Domain ID=42）执行：

```bash
# 检查是否能看到机器人的 topic
ros2 topic list | grep -E "goal_pose|amcl_pose|navigate_to_pose"
```

应看到：
```
/amcl_pose
/goal_pose
/navigate_to_pose/_action/status
```

### 第二步：验证位置反馈

```bash
ros2 topic hz /amcl_pose
```

应输出约 `average rate: 10.0`。

### 第三步：手动测试导航

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0},
  orientation: {w: 1.0}}}"
```

机器人应开始向 (5, 0) 移动。

### 第四步：验证状态追踪

```bash
ros2 topic echo /navigate_to_pose/_action/status --once
```

导航中应看到 `status: 2` (EXECUTING)，完成后变为 `status: 4` (SUCCEEDED)。

### 第五步：openTCS 端到端测试

通过 openTCS Plant Overview 发送运输订单，观察：
1. `/goal_pose` 收到目标
2. 机器人开始导航
3. `/amcl_pose` 实时更新位置
4. 导航完成后 action status 变为 SUCCEEDED
5. openTCS 界面显示运输订单完成

---

## 9. 注意事项

1. **阿克曼转向约束**：机器人不能原地旋转，调头需要弧线空间。plant model 中相邻路径点之间需要留出足够的转弯半径（≥0.5m）
2. **目标容差**：导航到达判定为 ±0.25m + yaw ±0.25rad，plant model 中点位不需要精确到厘米级
3. **目标点必须可到达**：目标位置必须在代价地图的自由空间内（距离障碍物 ≥0.6m，inflation_radius=1.0~1.2m）
4. **新目标自动取消旧目标**：桥接节点收到新 `/goal_pose` 时会自动取消前一个未完成的导航目标
5. **坐标系一致性**：所有坐标均在 `map` frame 下，使用预建地图 `auto_exploration_map.yaml`
6. **地图更新**：如工厂环境发生重大变化（货架移动、新增障碍物等），需重新建图并替换 `auto_exploration_map.yaml`
7. **控制器使用 RegulatedPurePursuit**：已启用 `allow_reversing: true`，机器人可在必要时倒车；规划器使用 Navfn (Dijkstra)
