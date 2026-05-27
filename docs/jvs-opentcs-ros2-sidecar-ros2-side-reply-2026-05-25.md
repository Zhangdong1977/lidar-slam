# ROS2 侧需求回复 — 致 Sidecar 团队

日期：2026/05/25
回复文档：`docs/jvs-opentcs-ros2-sidecar-ros2-side-needs-2026-05-25.md`
修改分支：master（未提交）

---

## 一、概述

针对需求文档中 ROS2 侧需补充的 6 项事项，已全部完成代码实现。以下逐项回复实施状态、sidecar 侧需要配合的改动，以及接口变更细节。

---

## 二、逐项回复

### 2.1 P0: orderId / currentTransportOrder 关联 ✅ 已完成

**实施方式**：采用方案 A1（frame_id 编码）

sidecar 在发布 `/goal_pose` 时，将 orderId 编码到 `header.frame_id` 中，格式如下：

```
frame_id: "map/orderId=TO-000123"
```

ROS2 节点收到后：
1. 解析 `/orderId=` 后面的字符串作为 `currentTransportOrder`
2. 还原 `frame_id` 为 `"map"` 后发送给 Nav2
3. goal 结束（成功/取消/中止）时自动清空 orderId

**sidecar 需要配合**：

| 改动 | 说明 |
|------|------|
| `send_goal()` 中设置 `frame_id` | 将 `pose.header.frame_id` 从 `"map"` 改为 `"map/orderId=" + orderId` |
| 无 orderId 时保持原样 | frame_id 为 `"map"` 不含 `/orderId=`，`currentTransportOrder` 将为空字符串 |

**验证**：`/robot_state` JSON 中 `currentTransportOrder` 字段将在收到 goal 后填充，goal 结束后清空。

**可选关闭**：参数 `goal_order_id_parse: true`，设为 `false` 可关闭解析。

---

### 2.2 P1: emergencyStop / safetyStop / obstacleDetected ✅ 已完成

#### emergencyStop / safetyStop

当前为**可配置订阅**模式，默认不订阅（仿真环境无硬件急停信号）。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `emergency_stop_topic` | `""` | 非空时订阅 `std_msgs/Bool` |
| `safety_stop_topic` | `""` | 非空时订阅 `std_msgs/Bool` |

**实车接入时** sidecar 需配合：
- 确认底盘驱动节点发布的急停 topic 名称
- 在 `opentcs_vehicle.yaml` 中填入对应 topic

#### obstacleDetected

采用 Nav2 collision_monitor 作为主检测源。

| 模式 | 参数值 | 订阅 topic | 说明 |
|------|--------|------------|------|
| collision_monitor（默认） | `"collision_monitor"` | `/collision_monitor_state` | Nav2 内置碰撞监控，状态为 CLEAR/APPROACH/STOP |
| scan 直检 | `"scan"` | `/scan` | 前方 ±30° 内障碍物 < 0.5m 时触发 |
| 禁用 | `"disabled"` | 无 | 始终返回 false |

**Nav2 配置变更**：
- `collision_monitor.FootprintApproach.enabled`: `False` → `True`
- `lifecycle_manager_navigation.node_names`: 新增 `'collision_monitor'`

**sidecar 无需配合**：`obstacleDetected` 字段值由 ROS2 节点自动计算，sidecar 直接读取 `/robot_state` JSON 即可。

**验证**：在 Gazebo 中放置障碍物，`/robot_state` JSON 中 `obstacleDetected` 应变为 `true`。

---

### 2.3 P1: 实车电池接入 ✅ 已完成

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `battery_sim_enabled` | `true` | `true`=仿真模式，`false`=实车模式 |
| `battery_real_topic` | `"/battery_state_real"` | 实车 BMS 数据源（仅 `battery_sim_enabled: false` 时生效） |

**切换方式**：
```yaml
battery_sim_enabled: false
battery_real_topic: "/bms/battery_state"  # 底盘驱动节点实际发布的 topic
```

**行为差异**：

| 模式 | 数据来源 | 发布频率 | charging 状态 |
|------|----------|----------|---------------|
| 仿真 (`true`) | 线性衰减模型 | 1Hz | 始终 `false` |
| 实车 (`false`) | 订阅 BMS topic | 1Hz（定时转发） | 根据 `POWER_SUPPLY_STATUS_CHARGING` 判断 |

**sidecar 无需配合**：`/battery_state` topic 消息格式不变（`sensor_msgs/BatteryState`），sidecar 透明消费。

---

### 2.4 P2: 多车 namespace 支持 ✅ 已完成

**节点侧**：

| 新增参数 | 默认值 | 说明 |
|----------|--------|------|
| `odom_topic` | `"/odom"` | 原硬编码，现可配置 |
| `amcl_subscribe_topic` | `"/amcl_pose"` | 原硬编码，现可配置 |

**launch 侧**：

```bash
ros2 launch lidar_slam_nodes sim_ackermann_rs485.launch.py \
    vehicle_name:=robot1 \
    namespace:=robot1
```

launch 文件通过 ROS2 原生 namespace 机制为 `opentcs_vehicle_node` 设置命名空间。

**sidecar 多车时需配合**：

| 改动 | 说明 |
|------|------|
| 每个 vehicle 对应不同 topic | sidecar 需订阅 `/robot1/robot_state`、`/robot2/robot_state` 等 |
| goal 发布到对应 namespace | sidecar 发布 `/robot1/goal_pose`、`/robot2/goal_pose` |

**注**：当前所有 topic 参数使用绝对路径（以 `/` 开头），namespace 仅影响节点自身命名。多车时需在 YAML 配置中为每辆车指定独立的 topic 名称。

---

### 2.5 P2: currentPosition / lastNodeId / nextPosition ✅ 已完成

**方案**：ROS2 侧提供接口，由 sidecar 注入位置信息。

| 新增参数 | 默认值 | 说明 |
|----------|--------|------|
| `position_report_topic` | `""` | 非空时订阅 `std_msgs/String`（JSON） |

**sidecar 发布格式**（topic 名称由参数配置）：

```json
{
  "nodeId": "Point-0007",
  "currentPosition": "Point-0005",
  "nextPosition": "Point-0007",
  "timestamp": 1779445800.123
}
```

字段均为可选，有哪个发哪个。

**`/robot_state` JSON 新增字段**：

| 字段 | 类型 | 初始值 | 来源 |
|------|------|--------|------|
| `currentPosition` | string | `""` | sidecar 通过 position_report 注入 |
| `lastNodeId` | string | `""` | sidecar 通过 position_report 注入 |
| `nextPosition` | string | `""` | sidecar 通过 position_report 注入 |

**sidecar 可选配合**：
- 如果 sidecar 已有 `VehiclePointProjectionContract` 做坐标→点位投影，可直接通过 `/robot_state` 中的 x/y/yaw 自行计算，无需发布 `position_report`。
- 如果需要 ROS2 侧上报点位名称，sidecar 发布 `position_report` topic 即可。

---

### 2.6 P3: YAML/PGM checksum ✅ 已完成

| 新增参数 | 默认值 | 说明 |
|----------|--------|------|
| `map_yaml_file` | `""` | 非空时启动时计算 PGM 文件 SHA256 |

**`/robot_state` JSON 新增字段**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `mapChecksum` | string | PGM 文件 SHA256 前 16 位，空字符串表示未配置 |

**配置示例**：
```yaml
map_yaml_file: "/home/hello/lidar-slam/maps/auto_exploration_map.yaml"
```

启动时从 YAML 中解析 `image:` 字段定位 PGM 文件，计算 SHA256。checksum 在节点生命周期内不变。

**sidecar 无需配合**：直接读取 `/robot_state` JSON 中 `mapChecksum` 字段。

---

## 三、`/robot_state` JSON 完整字段（更新后）

| 字段 | 类型 | 变更 | 说明 |
|------|------|------|------|
| vehicleName | string | — | |
| vehicleId | string | — | |
| accessIdentity | string | — | |
| timestamp | float | — | |
| frameId | string | — | |
| x | float | — | |
| y | float | — | |
| yaw | float | — | |
| quaternionZ | float | — | |
| quaternionW | float | — | |
| linearVelocity | float | — | |
| angularVelocity | float | — | |
| battery | float | — | |
| charging | bool | — | |
| state | string | — | IDLE/WORKING/ERROR/... |
| dispatchStatus | string | — | UNKNOWN/ACCEPTED/EXECUTING/... |
| localizationStatus | string | — | OK/DEGRADED/LOST/INITIALIZING |
| localizationScore | float | — | |
| emergencyStop | bool | **新增真实值** | 原硬编码 false |
| safetyStop | bool | **新增真实值** | 原硬编码 false |
| obstacleDetected | bool | **新增真实值** | 原硬编码 false |
| faultCode | string | — | |
| faultMessage | string | — | |
| currentTransportOrder | string | **新增填充** | 原始终为空 |
| goalId | string | — | |
| distanceRemaining | float | — | |
| estimatedTimeRemaining | float | — | |
| currentPosition | string | **新增** | sidecar 通过 position_report 注入 |
| lastNodeId | string | **新增** | sidecar 通过 position_report 注入 |
| nextPosition | string | **新增** | sidecar 通过 position_report 注入 |
| mapChecksum | string | **新增** | PGM SHA256 前 16 位 |
| maxSpeed | float | **新增** | 最大速度 m/s |

---

## 四、sidecar 需配合事项汇总

| 优先级 | 事项 | sidecar 侧改动 | 影响范围 |
|--------|------|----------------|----------|
| **P0** | orderId 关联 | `send_goal()` 时在 `frame_id` 中编码 orderId | `currentTransportOrder` 字段 |
| P1 | emergencyStop/safetyStop | 实车时告知急停/安全 topic 名称 | 安全相关字段 |
| P1 | obstacleDetected | **无需改动**，直接读取 JSON | 障碍物检测字段 |
| P1 | 实车电池 | **无需改动**，ROS2 侧切换配置即可 | 电池字段 |
| P2 | 多车 namespace | 多车时订阅不同 namespace 下的 topic | topic 路径 |
| P2 | 位置字段 | **可选**，发布 position_report topic 注入位置名 | 位置相关字段 |
| P3 | 地图 checksum | **无需改动**，直接读取 JSON | mapChecksum 字段 |

---

## 五、Nav2 辅助控制接口回复

需求文档第四节中 sidecar 已实现的 HTTP 接口，ROS2 侧确认如下：

| 接口 | Nav2 service | 状态 |
|------|-------------|------|
| `/api/v1/navigation/pause` | `cancel_goal_async()` | ✅ 可用（节点已实现 cancel） |
| `/api/v1/navigation/resume` | 重新发送最近目标 | ✅ 可用（goalId 已记录） |
| `/api/v1/navigation/clear-costmaps` | `nav2_msgs/srv/ClearEntireCostmap` | ✅ Nav2 默认提供 `clear_costmap` service |
| `/api/v1/navigation/recover` | behavior_server recovery | ✅ Nav2 默认提供 |
| `/api/v1/navigation/validate-goal` | `nav2_msgs/srv/ComputePathThroughPoses` | ✅ Nav2 默认提供 |

建议：sidecar 在调用 service 前先检查 service 是否可用（`ros2 service list`），不可用时返回 `UNSUPPORTED`，不影响主流程。

---

## 六、配置文件参考

完整 `config/opentcs_vehicle.yaml`：

```yaml
opentcs_vehicle_node:
  ros__parameters:
    vehicle_name: "ackermann_robot"
    access_identity: "ackermann_robot"
    namespace: ""
    domain_id: 42
    base_frame: "body_link"
    map_frame: "map"
    amcl_pose_topic: "/amcl_pose"
    goal_pose_topic: "/goal_pose"
    robot_state_topic: "/robot_state"
    battery_state_topic: "/battery_state"
    nav_action_name: "/navigate_to_pose"
    pose_publish_rate: 10.0
    status_sample_ms: 1000
    heartbeat_timeout_ms: 30000
    cancel_timeout_ms: 5000
    max_speed: 1.4
    battery_sim_enabled: true
    battery_sim_start_percent: 95.0
    battery_sim_drain_rate: 5.0
    # --- 新增参数 ---
    goal_order_id_parse: true
    emergency_stop_topic: ""
    safety_stop_topic: ""
    obstacle_detection_mode: "collision_monitor"
    obstacle_scan_threshold: 0.5
    obstacle_scan_angle_window: 1.047
    battery_real_topic: "/battery_state_real"
    odom_topic: "/odom"
    amcl_subscribe_topic: "/amcl_pose"
    position_report_topic: ""
    map_yaml_file: ""
```
