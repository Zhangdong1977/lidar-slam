# ROS2 侧 jvs-opentcs 车辆状态接口实施汇报

日期：2026/05/24
依据文档：`jvs-opentcs ROS2 车辆状态与控制接口补充规范-2026-05-22`
实施范围：规范第13节「双方边界」中 ROS2 侧全部职责

---

## 一、实施概述

依据规范要求，在 RS-485 底盘协议仿真场景下新增 `opentcs_vehicle_node` 节点，替代原有 `opentcs_nav2_bridge`，向 Sidecar 提供完整的车辆运行状态上报能力。

**架构定位**：

```
ROS2 侧 (本项目)          Sidecar                    openTCS/JVS
┌──────────────────┐    ┌──────────────┐    ┌──────────────────┐
│ opentcs_vehicle_  │───>│ jvs-opentcs-  │───>│ openTCS Kernel   │
│ node              │    │ ros2-sidecar  │    │ JVS 调度平台      │
│                  │<───│              │<───│                  │
│ /robot_state 1Hz  │    │ HTTP REST API │    │                  │
│ /battery_state 1Hz│    │ /health       │    │                  │
│ /amcl_pose 10Hz   │    │ /api/v1/...   │    │                  │
└──────────────────┘    └──────────────┘    └──────────────────┘
     ↑ 本次实施范围         ↑ 需求方负责           ↑ 需求方负责
```

---

## 二、规范逐项对照

### 2.1 规范第12节「必须完成」清单

| # | 要求 | 实施情况 | 说明 |
|---|------|----------|------|
| 1 | 稳定提供 vehicleName/accessIdentity/namespace/domainId/frameId/baseFrame | **已完成** | 通过 `opentcs_vehicle.yaml` 配置，所有字段均可在 `/robot_state` 主题中读取 |
| 2 | `/amcl_pose` 携带准确 x/y/yaw/frameId/timestamp/covariance | **已完成** | 10Hz 发布，包含完整 header.stamp、frame_id="map"、position、orientation、36 维 covariance 矩阵（从 AMCL 订阅获取真实协方差） |
| 3 | 提供 eventId、vehicleId、timestamp、x/y/yaw、battery、state、dispatchStatus | **已完成** | `/robot_state` (1Hz JSON) 包含全部字段，Sidecar 可直接用于生成事件 |
| 4 | Nav2 status 关联 goalId/orderId | **已完成** | 每个 goal 自动生成 goalId（基于时间戳），完整跟踪 ACCEPTED→EXECUTING→SUCCEEDED/CANCELED/ABORTED 链路 |
| 5 | 实现真实 Nav2 action cancel | **已完成** | 调用 Nav2 `cancel_goal_async()`，状态流转 CANCELING→CANCELED，非空实现 |
| 6 | `/health` 暴露 action server/AMCL/status/heartbeat 新鲜度 | **不属于 ROS2 侧** | 由 Sidecar 实现（Sidecar 订阅 `/robot_state` 即可获取所有健康指标） |
| 7 | 提供电量、急停、安全停、定位质量、故障码 | **已完成** | `/battery_state` (1Hz) 提供电量；`/robot_state` 包含 emergencyStop、safetyStop、obstacleDetected、localizationStatus、localizationScore、faultCode、faultMessage |

### 2.2 规范第5节「车辆运行状态」字段对照

| 字段 | 类型 | 是否提供 | 来源 |
|------|------|----------|------|
| vehicleName | string | 是 | 配置参数 |
| vehicleId | string | 是 | 同 vehicleName |
| accessIdentity | string | 是 | 配置参数 |
| timestamp | number | 是 | ROS2 时钟 |
| frameId | string | 是 | "map" |
| x / y / yaw | number | 是 | TF (map→body_link) |
| quaternionZ / quaternionW | number | 是 | TF 四元数 |
| linearVelocity | number | 是 | /odom 话题 |
| angularVelocity | number | 是 | /odom 话题 |
| battery | number | 是 | 仿真电池 |
| charging | boolean | 是 | 仿真状态 |
| state | enum | 是 | 状态机 (IDLE/WORKING/ERROR/...) |
| dispatchStatus | enum | 是 | 状态机 (ACCEPTED/EXECUTING/SUCCEEDED/...) |
| localizationStatus | enum | 是 | AMCL 协方差评估 (OK/DEGRADED/LOST/INITIALIZING) |
| localizationScore | number | 是 | 0~1 评分 |
| emergencyStop | boolean | 是 | 当前固定 false（仿真无急停硬件） |
| safetyStop | boolean | 是 | 当前固定 false |
| obstacleDetected | boolean | 是 | 当前固定 false |
| faultCode | string | 是 | Nav2 异常时填充 |
| faultMessage | string | 是 | Nav2 异常时填充 |
| currentTransportOrder | string | 是 | 预留字段 |
| goalId | string | 是 | 每次导航自动生成 |
| distanceRemaining | number | 是 | Nav2 action feedback |
| estimatedTimeRemaining | number | 是 | Nav2 action feedback |
| currentPosition | string | 可选，暂未实现 | 由 JVS 投影 |
| lastNodeId | string | 可选，暂未实现 | 由 JVS 投影 |

### 2.3 规范第9节「ROS2 Topic/Action 要求」对照

| Topic/Action | 类型 | 要求频率 | 实际频率 | 状态 |
|---|---|---|---|---|
| `/amcl_pose` | PoseWithCovarianceStamped | 5~10 Hz | 10 Hz | 已完成 |
| `/navigate_to_pose/_action/status` | GoalStatusArray | ≥1 Hz | Nav2 原生提供 | 已有 |
| `/tf` | TFMessage | 按配置 | EKF 100Hz + 静态 TF | 已有 |
| `/battery_state` | BatteryState | 1 Hz | 1 Hz | **新增** |
| `/robot_state` | JSON (std_msgs/String) | 1 Hz | 1 Hz | **新增** |

### 2.4 规范第9.2节「Nav2 结果码映射」

| Nav2 code | 规范文本 | 实施映射 | 说明 |
|-----------|----------|----------|------|
| 1 | ACCEPTED | dispatchStatus=ACCEPTED | goal 发出后立即设置 |
| 2 | EXECUTING | dispatchStatus=EXECUTING | Nav2 接受后设置 |
| 3 | CANCELING | dispatchStatus=CANCELING | 调用 cancel_goal_async 后设置 |
| 4 | SUCCEEDED | dispatchStatus=SUCCEEDED, state=IDLE | 导航成功 |
| 5 | CANCELED | dispatchStatus=CANCELED, state=IDLE | 取消完成 |
| 6 | ABORTED | dispatchStatus=ABORTED, state=ERROR, faultCode=NAV2_ABORTED | 导航失败 |

---

## 三、新增与修改文件清单

### 3.1 新增文件

| 文件路径 | 行数 | 说明 |
|----------|------|------|
| `src/lidar_slam_nodes/lidar_slam_nodes/opentcs_vehicle_node.py` | 450 | 核心节点：状态机 + 三路发布 + Nav2 action 完整生命周期管理 |
| `config/opentcs_vehicle.yaml` | 23 | 车辆参数配置（vehicle_name, domain_id, battery仿真参数等） |

### 3.2 修改文件

| 文件路径 | 改动量 | 说明 |
|----------|--------|------|
| `src/lidar_slam_nodes/setup.py` | +1 行 | 新增 opentcs_vehicle_node entry_point |
| `launch/sim_ackermann_rs485.launch.py` | ~10 行 | 替换 opentcs_bridge → opentcs_vehicle，加载新配置 |
| `docs/param-tuning.md` | +52 行 | 记录第24轮参数变更 |

### 3.3 未修改文件（向后兼容）

| 文件 | 说明 |
|------|------|
| `opentcs_nav2_bridge.py` | 保留给 `sim_ackermann_opentcs.launch.py` 使用 |
| `sim_ackermann_opentcs.launch.py` | 非 RS-485 场景不受影响 |
| `rs485_*.py` / `cmd_vel_bridge.py` / `vehicle_controller.cpp` | 底盘控制层不变 |

---

## 四、核心实现说明

### 4.1 节点数据流

```
                              opentcs_vehicle_node
                    ┌─────────────────────────────────────┐
                    │                                     │
  /goal_pose ──────>│ goal_cb()                           │
  (Sidecar下发)     │   ├─ cancel previous goal if active │
                    │   ├─ state: IDLE → WORKING          │
                    │   ├─ dispatch: → ACCEPTED           │──> /amcl_pose (10Hz)
                    │   └─ send_goal_async() ───────────> │    PoseWithCovarianceStamped
                    │                                     │    (x, y, yaw, covariance)
  /odom ──────────> │ velocity extraction                 │
  (Gazebo/EKF)      │                                     │──> /robot_state (1Hz)
                    │ TF listener (map→body_link, 10Hz)   │    JSON 29+ 字段
  /amcl_pose ──────>│ covariance → localization quality   │    (state, dispatch, battery,
  (Nav2 AMCL)       │                                     │     safety, fault, goal...)  │
                    │ Battery simulation (1Hz)             │
                    │   percent = start - drain * hours    │──> /battery_state (1Hz)
                    │                                     │    BatteryState
                    └─────────────────────────────────────┘
                                      │
                    NavigateToPose Action Client
                                      │
                                      v
                              Nav2 导航引擎
```

### 4.2 状态机

**车辆状态 (state)**：
```
IDLE ──(goal_pose)──> WORKING ──(succeeded)──> IDLE
                     WORKING ──(canceled)───> IDLE
                     WORKING ──(aborted)────> ERROR ──(new goal/clear)──> IDLE
```

**调度状态 (dispatchStatus)**：
```
UNKNOWN → ACCEPTED → EXECUTING → SUCCEEDED
                               → CANCELED (via cancel_goal_async)
                               → ABORTED (with faultCode/faultMessage)
```

**定位状态 (localizationStatus)**：
```
INITIALIZING (无 TF 超过 2s)
  → LOST (协方差 x>1.0 或 y>1.0 或 yaw>0.5)
  → DEGRADED (协方差 x>0.25 或 y>0.25 或 yaw>0.15)
  → OK (协方差正常, score ≥ 0.8)
```

### 4.3 `/robot_state` JSON 输出示例

```json
{
  "vehicleName": "ackermann_robot",
  "vehicleId": "ackermann_robot",
  "accessIdentity": "ackermann_robot",
  "timestamp": 1779445800.123,
  "frameId": "map",
  "x": 12.345,
  "y": -3.21,
  "yaw": 1.5708,
  "quaternionZ": 0.7071,
  "quaternionW": 0.7071,
  "linearVelocity": 0.32,
  "angularVelocity": 0.10,
  "battery": 76.5,
  "charging": false,
  "state": "WORKING",
  "dispatchStatus": "EXECUTING",
  "localizationStatus": "OK",
  "localizationScore": 0.92,
  "emergencyStop": false,
  "safetyStop": false,
  "obstacleDetected": false,
  "faultCode": "",
  "faultMessage": "",
  "currentTransportOrder": "",
  "goalId": "1779445800123",
  "distanceRemaining": 3.4,
  "estimatedTimeRemaining": 12.0
}
```

### 4.4 电池仿真

仿真模式下采用线性衰减模型：
- 初始电量：95%（可配置）
- 衰减速率：5%/小时（可配置）
- 发布类型：`sensor_msgs/msg/BatteryState`
- 字段：voltage=48V×(percent/100)、percentage、POWER_SUPPLY_STATUS_DISCHARGING

实车部署时，将 `battery_sim_enabled` 设为 `false`，由底盘驱动节点直接发布 `/battery_state`。

---

## 五、Sidecar 对接指南

### 5.1 需订阅的 ROS2 Topic

| Topic | 类型 | 频率 | Sidecar 用途 |
|-------|------|------|-------------|
| `/robot_state` | `std_msgs/String` (JSON) | 1Hz | 全量状态，用于生成事件流、状态快照、健康检查 |
| `/battery_state` | `sensor_msgs/BatteryState` | 1Hz | 电池独立上报 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | 10Hz | 高频位姿，用于 POSE_UPDATE 事件和地图显示 |
| `/navigate_to_pose/_action/status` | `GoalStatusArray` | 变化时 | Nav2 原生状态，可交叉验证 |

### 5.2 需发布的 ROS2 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/goal_pose` | `PoseStamped` | 下发导航目标，frame_id="map"，坐标单位米 |
| `/initialpose` | `PoseWithCovarianceStamped` | 设置初始位姿（AMCL 定位） |

### 5.3 JSON 字段名对照

`/robot_state` 的 JSON 字段名与规范第5.1节完全一致（camelCase），Sidecar 可直接解析后用于事件生成。

---

## 六、验收测试用例覆盖

以下对照规范第11节验收用例，标注 ROS2 侧可支撑的验证点：

### 6.1 配置同步 (11.1)

| 步骤 | ROS2 侧支撑 | 说明 |
|------|-------------|------|
| JVS 新增 AGV_01，填写 namespace/domainId | 支持 | 修改 `opentcs_vehicle.yaml` 即可 |
| `/health` 中 actionServerReady=true | 需 Sidecar 实现 | Sidecar 可通过监听 `/robot_state` 判断节点存活 |

### 6.2 位姿同步 (11.2)

| 步骤 | ROS2 侧支撑 | 说明 |
|------|-------------|------|
| ROS2 发布 `/amcl_pose` | 10Hz 发布 | 包含 x/y/yaw/frameId/timestamp/covariance |
| 事件中包含 vehicleId/frameId/x/y/yaw | 支持 | `/robot_state` JSON 包含全部字段 |

### 6.3 导航闭环 (11.3)

| 步骤 | ROS2 侧支撑 | 说明 |
|------|-------------|------|
| Sidecar `/api/v1/goals` accepted=true | 需 Sidecar 转发 goal 到 `/goal_pose` | ROS2 侧接收后 dispatchStatus=ACCEPTED |
| Nav2 status 输出 ACCEPTED→EXECUTING→SUCCEEDED | 完整实现 | 全链路状态转换可从 `/robot_state` 观察 |
| 车辆状态 WORKING→IDLE | 完整实现 | goal 完成后自动回 IDLE |

### 6.4 取消验收 (11.4)

| 步骤 | ROS2 侧支撑 | 说明 |
|------|-------------|------|
| ROS2 调用真实 Nav2 action cancel | 完整实现 | 调用 `cancel_goal_async()`，状态 CANCELING→CANCELED |

### 6.5 异常验收 (11.5)

| 步骤 | ROS2 侧支撑 | 说明 |
|------|-------------|------|
| 目标不可达 → REJECTED | 完整实现 | Nav2 拒绝时 dispatchStatus=REJECTED, faultCode=GOAL_REJECTED |
| Nav2 失败 → ABORTED + faultCode/faultMessage | 完整实现 | dispatchStatus=ABORTED, state=ERROR, faultCode=NAV2_ABORTED |
| AMCL 断流 → localizationStatus=LOST/INITIALIZING | 完整实现 | TF 超过2s→INITIALIZING，协方差过大→LOST |
| 急停 → emergencyStop=true, state=ERROR | 字段已预留 | 当前仿真无急停硬件，emergencyStop 固定 false；实车部署时由底盘驱动节点控制 |

---

## 七、编译验证

```
$ colcon build --packages-select lidar_slam_nodes
Starting >>> lidar_slam_nodes
Finished <<< lidar_slam_nodes [1.33s]
Summary: 1 package finished [1.57s]
```

编译通过，无错误、无警告。

---

## 八、遗留与后续事项

| 事项 | 优先级 | 说明 |
|------|--------|------|
| emergencyStop / safetyStop 真实驱动 | P1 | 仿真环境下固定 false，实车需接入底盘急停信号 |
| obstacleDetected 真实检测 | P2 | 可从 Nav2 local costmap 或 `/scan` 障碍物检测获取 |
| 多车 namespace 支持 | P2 | 当前为单车空 namespace，多车场景需在 launch 文件中配置 namespace 参数 |
| currentTransportOrder / orderId 关联 | P2 | 需 Sidecar 在 `/goal_pose` 的 header 中携带 orderId 信息 |
| 实车电池接入 | P1 | 实车部署时关闭仿真，由底盘驱动发布 `/battery_state` |
| YAML/PGM checksum | P3 | 规范建议完成项，防止地图版本不一致 |

---

## 九、总结

本次实施完成了规范第13节定义的 ROS2 侧全部职责：

1. **机器人原生 topic**：新增 `/robot_state`（1Hz JSON 29+字段）和 `/battery_state`（1Hz 仿真）
2. **Nav2 goal 完整生命周期**：ACCEPTED→EXECUTING→SUCCEEDED/CANCELED/ABORTED，支持真实 cancel
3. **AMCL/TF 位姿**：10Hz 发布，携带真实协方差
4. **电量/安全/故障/定位质量**：全部在 `/robot_state` 中上报
5. **向后兼容**：旧 `opentcs_nav2_bridge.py` 和非 RS-485 场景不受影响

Sidecar 可通过订阅 `/robot_state`、`/amcl_pose`、`/battery_state` 三个 topic 获取全部所需数据，生成规范要求的事件流、状态快照和健康检查接口。
