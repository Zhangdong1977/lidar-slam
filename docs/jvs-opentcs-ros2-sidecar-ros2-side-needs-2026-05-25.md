# jvs-opentcs-ros2-sidecar ROS2 侧开发需求

日期：2026-05-25
---

## 一、文档说明

```
本文档
  └─ 整理：sidecar 已具备的能力之外，ROS2 节点还须补充什么
```

---

## 二、已完成部分（sidecar 保障）

以下能力 sidecar **已有完整实现**，ROS2 节点只需正常发布对应 topic：

| topic | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/amcl_pose` | `PoseWithCovarianceStamped` | 10Hz | sidecar 订阅，解析 x/y/yaw/covariance |
| `/robot_state` | `std_msgs/String` (JSON) | 1Hz | sidecar 订阅，解析全部状态字段 |
| `/battery_state` | `BatteryState` | 1Hz | sidecar 订阅，解析 percentage/charging |
| `/navigate_to_pose/_action/status` | `GoalStatusArray` | 变化时 | sidecar 订阅，关联 goalId/orderId |
| `/goal_pose` | `PoseStamped` | 按需 | sidecar 发布，ROS2 节点订阅 |
| `/initialpose` | `PoseWithCovarianceStamped` | 按需 | sidecar 发布，ROS2 节点订阅 |

---

## 三、ROS2 侧必须补充的事项

### 3.1 orderId / currentTransportOrder 关联（P0）

**问题**：`/goal_pose` 是 `geometry_msgs/PoseStamped`，无法携带业务元数据。JVS 发来的 `GoalRequest.order_id` 在 sidecar 转为 topic 发布后，ROS2 节点收不到 orderId。

**现状**：
- sidecar `send_goal()` 将 `currentTransportOrder` 字段写入 state store
- 但当 `opentcs_vehicle_node` 订阅 `/goal_pose` 时，PoseStamped header 无法携带 orderId
- 导致 `/robot_state` 中 `currentTransportOrder` 和 `goalId` 的映射为空

**规范要求**（Section 5.1）：
```
currentTransportOrder  建议   当前 JVS 订单号
goalId               建议   当前 Nav2 goal 标识
```

**建议方案**：

方案 A（推荐）：在 `opentcs_vehicle_node` 订阅 `/goal_pose` 时，通过以下方式关联 orderId：
- 方案 A1：在 PoseStamped 的 `header.frame_id` 中约定特殊格式，如 `map/orderId=TO-xxx`

---

### 3.2 emergencyStop / safetyStop / obstacleDetected 真实值（P1）

**问题**：实施报告 Section 8 遗留，当前 `opentcs_vehicle_node` 固定返回 `false`。

**规范要求**（Section 5.1）：
| 字段 | 类型 | 说明 |
|------|------|------|
| `emergencyStop` | boolean | 急停是否触发 |
| `safetyStop` | boolean | 安全防护停车 |
| `obstacleDetected` | boolean | 前方障碍或局部规划阻塞 |

**ROS2 节点需补充**：

1. **emergencyStop**：
   - 监听底盘安全急停信号（如 `/emergency_stop` topic 或 `/diagnostics`）
   - 收到急停触发时，在 `/robot_state` JSON 中输出 `"emergencyStop": true`
   - JVS 要求：急停触发时禁止继续自动派单

2. **safetyStop**：
   - 监听安全传感器（如安全光幕、急停按钮）
   - 安全防护触发时输出 `"safetyStop": true`

3. **obstacleDetected**：
   - 融合 `/scan`、`/obstacle` 或 Nav2 local costmap 信息
   - 检测到障碍物时输出 `"obstacleDetected": true`
   - 同时应触发 `dispatchStatus=BLOCKED`

---

### 3.3 currentPosition / lastNodeId / nextPosition（P2）

**问题**：`/robot_state` JSON 中这三个字段为空。

**规范要求**（Section 5.1）：
| 字段 | 类型 | 说明 |
|------|------|------|
| `currentPosition` | string | 当前最近 JVS 点位 |
| `lastNodeId` | string | 最近经过点位 |
| `nextPosition` | string | 下一目标点位 |

**说明**：这三个字段规范标注为"可选，由 JVS 投影"。如果 JVS 侧有地图点位映射（sidecar 有 `VehiclePointProjectionContract`），JVS 可以自己完成投影。但如果有 ROS2 侧能识别 JVS 点位（如在路径点上发送经过通知），则可提供更准确的跟踪。

**ROS2 节点可补充**：
- 在路径经过关键点时发布 `/robot_position_report` topic（自定义）
- 内容：`{nodeId: "Point-0007", timestamp: 1779445800.123}`
- sidecar 订阅后填充 `lastNodeId`

---

### 3.4 多车 namespace 支持（P2）

**问题**：实施报告 Section 8 遗留，当前 `opentcs_vehicle_node` 仅支持单车。

**影响**：多车时 topic 会冲突。

**ROS2 节点需补充**：
- launch 文件支持动态传入 `vehicle_name` 和 `namespace` 参数
- 所有 topic（`/goal_pose`、`/amcl_pose`、`/robot_state` 等）自动加上 namespace 前缀
- 多个 `opentcs_vehicle_node` 实例可同时运行

---

### 3.5 实车电池接入（P1）

**问题**：实施报告 Section 8 遗留，当前使用仿真模型。

**ROS2 节点需补充**：
- 配置项 `battery_sim_enabled: false`
- 订阅实车电池 BMS 的 `/battery_state` topic（由底盘驱动节点发布）
- 将实车数据透传给 sidecar

---

## 四、Nav2 辅助控制接口（需确认）

sidecar 已实现以下 HTTP 接口，ROS2 节点需确认 Nav2 是否配置了对应 service：

| 接口 | sidecar 实现 | 需 ROS2 侧确认 |
|------|-------------|--------------|
| `/api/v1/navigation/pause` | `cancel_goal_async()` 取消当前 goal | Nav2 lifecycle service 是否可用 |
| `/api/v1/navigation/resume` | 重新发送最近目标 pose | 需 ROS2 节点能记住上次目标（已有 goalId）|
| `/api/v1/navigation/clear-costmaps` | 调用 `std_srv/Empty` service `clear_costmap` | Nav2 是否配置该 service |
| `/api/v1/navigation/recover` | 调用 `std_srv/Empty` service `recover` | Nav2 是否配置该 service |
| `/api/v1/navigation/validate-goal` | 调用 `nav2_msgs/srv/ComputePathThroughPoses` | Nav2 是否配置该 service |

**建议**：在 Nav2 启动时检查 service 是否存在，如不存在则 sidecar 返回 `UNSUPPORTED`，不影响主流程。

---

## 五、YAML/PGM checksum（P3）

规范 Section 4 建议提供地图 checksum，防止 JVS 与 ROS2 使用不同版本地图。

**ROS2 节点可选补充**：
- 启动时计算 YAML/PGM 文件 checksum（SHA256）
- 通过 `/robot_state` 或 `/health` 上报：
  ```json
  {
    "mapVersion": "2026-05-25T01",
    "yamlChecksum": "sha256:abc123...",
    "pgmChecksum": "sha256:def456..."
  }
  ```

---

## 六、汇总清单

| 优先级 | 事项 | 影响 |
|--------|------|------|
| **P0** | orderId 关联到 goalId | `currentTransportOrder` 字段永远为空，订单状态链路断 |
| **P1** | emergencyStop / safetyStop / obstacleDetected 真实值 | 安全相关判断失效 |
| **P1** | 实车电池接入 | 电池数据为固定仿真值 |
| **P2** | 多车 namespace 支持 | 多车场景无法工作 |
| **P2** | currentPosition / lastNodeId / nextPosition | 可选，但有助于更精确的状态跟踪 |
| **P3** | YAML/PGM checksum | 可选，防止地图版本不一致 |

---
