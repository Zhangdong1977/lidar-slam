# jvs-opentcs ROS2 车辆状态与控制接口补充规范

日期：2026-05-22
适用对象：ROS2/Nav2 机器人侧、`jvs-opentcs-ros2-sidecar`、`jvs-opentcs-device-gateway`、official openTCS Kernel CommAdapter
依据文档：

- `docs/opentcs-integration(1).md`
- `docs/opentcs-http-api.md`
- `docs/jvs-opentcs与ROS2-Sidecar当前代码对接逻辑说明-2026-05-21.md`
- 当前 `jvs-opentcs` 与 `jvs-opentcs-ros2-sidecar` 代码契约

## 1. 对接目标

ROS2 侧需要在现有 `/goal_pose`、`/initialpose`、`/amcl_pose`、`/navigate_to_pose/_action/status` 基础上，补齐车辆运行状态、定位质量、电量、安全状态、控制命令回执和事件游标能力。

补齐后 JVS 侧应能做到：

1. 车辆管理中配置的 `vehicleName/accessIdentity/namespace/domainId` 能动态同步到 Sidecar。
2. openTCS Kernel 能看到同名 Vehicle，并通过 ROS2_NAV2 CommAdapter 下发导航目标。
3. ROS2 侧能持续回传车辆位置、朝向、状态、电量、心跳、故障和 Nav2 goal 状态。
4. JVS 地图上的车辆位置、朝向、在线状态、任务执行状态可以实时同步。
5. 取消、暂停、恢复、初始位姿设置等控制命令有明确受理结果和最终结果。

## 2. 当前已有能力

ROS2 对接文档中已经提供：

| 能力 | 当前约定 | 说明 |
| --- | --- | --- |
| 导航目标 | `/goal_pose`，`geometry_msgs/msg/PoseStamped` | JVS/Sidecar 发布目标点，单位米，frame 为 `map`。 |
| 初始位姿 | `/initialpose`，`geometry_msgs/msg/PoseWithCovarianceStamped` | 用于 AMCL 初始定位。 |
| 车辆位姿 | `/amcl_pose`，`geometry_msgs/msg/PoseWithCovarianceStamped` | 机器人实时位置，文档目标频率约 10 Hz。 |
| 导航状态 | `/navigate_to_pose/_action/status`，`action_msgs/msg/GoalStatusArray` | Nav2 goal 状态。 |
| 坐标系 | `map` frame，单位米 | yaw=0 指向 X 正方向，逆时针为正。 |
| 网络环境 | `ROS_DOMAIN_ID=42`，`rmw_fastrtps_cpp` | 当前机器人环境固定值。 |
| 多车隔离 | namespace | 多车使用独立 namespace，例如 `/agv_01`。 |

现有能力可以完成最小导航，但不足以支撑生产调度的状态闭环。

## 3. ROS2 侧必须补充的车辆基础信息

每台机器人需要提供以下静态配置。JVS 会通过车辆管理维护这些字段，并同步给 Sidecar；ROS2 侧需要确认字段真实可用。

| 字段 | 类型 | 必填 | 示例 | 说明 |
| --- | --- | --- | --- | --- |
| `vehicleName` | string | 是 | `AGV_01` | 必须与 JVS 车辆名称、openTCS Kernel Vehicle 名称完全一致。 |
| `accessIdentity` | string | 是 | `robot_01` | 机器人访问标识，建议与底盘唯一编号一致。 |
| `namespace` | string | 是 | `/agv_01` | ROS2 namespace。单车可为空，但多车必须唯一。 |
| `domainId` | integer | 是 | `42` | ROS_DOMAIN_ID。一个 Sidecar 进程只允许一个 Domain。 |
| `rmwImplementation` | string | 是 | `rmw_fastrtps_cpp` | RMW 实现。一个 Sidecar 进程只允许一个 RMW。 |
| `frameId` | string | 是 | `map` | 全局地图坐标系。 |
| `baseFrame` | string | 是 | `body_link` | 机器人基座 frame。 |
| `navigateAction` | string | 是 | `navigate_to_pose` | Nav2 NavigateToPose action 名称。 |
| `goalPoseTopic` | string | 是 | `goal_pose` | 兼容 topic 模式的目标点 topic。 |
| `initialPoseTopic` | string | 是 | `initialpose` | 初始位姿 topic。 |
| `amclPoseTopic` | string | 是 | `amcl_pose` | 位姿反馈 topic。 |
| `navStatusTopic` | string | 是 | `navigate_to_pose/_action/status` | Nav2 status topic。 |
| `statusSampleMs` | integer | 是 | `100` | Sidecar 向 JVS 输出 POSE_UPDATE 的降频周期。建议 100 到 500 ms。 |
| `heartbeatTimeoutMs` | integer | 是 | `3000` | 超过该时间无状态则判定离线或不健康。 |
| `cancelTimeoutMs` | integer | 是 | `5000` | 取消命令等待 Nav2 确认的超时时间。 |
| `footprint` | array | 建议 | `[[0.45,0.30],[0.45,-0.30],[-0.45,-0.30],[-0.45,0.30]]` | 机器人碰撞轮廓，便于 JVS 校验点位可达性。 |
| `maxLinearSpeed` | number | 建议 | `0.5` | 最大线速度，单位 m/s。 |
| `maxReverseSpeed` | number | 建议 | `0.5` | 最大倒车速度，单位 m/s。 |
| `minTurningRadius` | number | 建议 | `0.35` | 最小转弯半径，单位 m。 |
| `xyGoalTolerance` | number | 建议 | `0.25` | 到点容差，单位 m。 |
| `yawGoalTolerance` | number | 建议 | `0.25` | 朝向容差，单位 rad。 |

命名要求：

1. `vehicleName` 不建议包含空格、中文和特殊字符，推荐 `AGV_01`、`AGV_02`。
2. ROS2 namespace/topic 只能包含字母、数字、下划线和 `/`，不要使用 `-`。
3. `vehicleName` 与 `namespace` 可以不同，但必须一一对应。

## 4. 地图与坐标信息要求

ROS2 侧需要提供并确认每张地图的坐标转换元数据。没有这些字段，JVS 无法稳定地把 openTCS 毫米坐标转换成 ROS2 米坐标。

| 字段 | 类型 | 必填 | 示例 | 说明 |
| --- | --- | --- | --- | --- |
| `mapId` | string | 是 | `auto_exploration_map` | 地图业务标识。 |
| `mapVersion` | string | 是 | `2026-05-22T01` | 地图版本或 checksum。 |
| `mapFrame` | string | 是 | `map` | ROS2 frame。 |
| `resolution` | number | 是 | `0.05` | YAML resolution，单位 m/pixel。 |
| `originX` | number | 是 | `-50.200` | YAML origin[0]，单位 m。 |
| `originY` | number | 是 | `-50.286` | YAML origin[1]，单位 m。 |
| `originYaw` | number | 是 | `0.0` | YAML origin[2]，单位 rad。 |
| `imageWidth` | integer | 建议 | `2000` | PGM 宽度。 |
| `imageHeight` | integer | 建议 | `2000` | PGM 高度。 |
| `yamlChecksum` | string | 建议 | `sha256:...` | 防止 JVS 与 ROS2 使用不同地图版本。 |
| `pgmChecksum` | string | 建议 | `sha256:...` | 防止底图不一致。 |

坐标约定：

```text
ROS2 坐标单位：米
openTCS/JVS 内部坐标单位：毫米
yaw 单位：弧度
yaw=0：X 正方向
yaw 正方向：逆时针
```

每次更换 YAML/PGM 地图后，ROS2 侧需要同步新的 `resolution/origin/checksum`，并与 JVS 当前地图版本一致。

## 5. ROS2 侧必须回传的车辆运行状态

### 5.1 车辆状态总览

ROS2/Sidecar 至少需要向 JVS 输出以下状态。可以通过 HTTP `/api/v1/events` 事件输出，也可以通过新增 `/api/v1/robots/status` 快照输出。

| 字段 | 类型 | 必填 | 示例 | 说明 |
| --- | --- | --- | --- | --- |
| `eventId` | integer/string | 是 | `1024001` | 单调递增事件 ID，用于 JVS 游标拉取和去重。 |
| `eventType` | string | 是 | `POSE_UPDATE` | 事件类型。 |
| `vehicleName` | string | 是 | `AGV_01` | 建议与 `vehicleId` 同值。 |
| `vehicleId` | string | 是 | `AGV_01` | JVS 当前解析主字段。 |
| `accessIdentity` | string | 建议 | `robot_01` | 辅助定位车辆。 |
| `timestamp` | number/string | 是 | `1779445800.123` | 建议 epoch seconds 或 ISO-8601，必须统一。 |
| `frameId` | string | 是 | `map` | 位姿所属 frame。 |
| `x` | number | 是 | `12.345` | ROS2 map 坐标，单位 m。 |
| `y` | number | 是 | `-3.210` | ROS2 map 坐标，单位 m。 |
| `yaw` | number | 是 | `1.5708` | 车头朝向，单位 rad。 |
| `quaternionZ` | number | 建议 | `0.7071` | 四元数 z。 |
| `quaternionW` | number | 建议 | `0.7071` | 四元数 w。 |
| `linearVelocity` | number | 建议 | `0.32` | 当前线速度，单位 m/s。 |
| `angularVelocity` | number | 建议 | `0.10` | 当前角速度，单位 rad/s。 |
| `battery` | number | 是 | `76.5` | 电量百分比，0 到 100。 |
| `charging` | boolean | 建议 | `false` | 是否充电。 |
| `state` | string | 是 | `WORKING` | 车辆运行状态，枚举见下文。 |
| `dispatchStatus` | string | 是 | `EXECUTING` | 导航/调度状态，枚举见下文。 |
| `localizationStatus` | string | 是 | `OK` | 定位状态。 |
| `localizationScore` | number | 建议 | `0.92` | 定位质量，0 到 1。 |
| `emergencyStop` | boolean | 是 | `false` | 急停是否触发。 |
| `safetyStop` | boolean | 建议 | `false` | 安全防护停车。 |
| `obstacleDetected` | boolean | 建议 | `false` | 前方障碍或局部规划阻塞。 |
| `faultCode` | string | 否 | `NAV2_TIMEOUT` | 故障编码。 |
| `faultMessage` | string | 否 | `controller timeout` | 故障描述。 |
| `currentTransportOrder` | string | 建议 | `TO-202605220001` | 当前 JVS/openTCS 订单号。 |
| `goalId` | string | 建议 | `TO-202605220001:0` | 当前 Nav2 goal 标识。 |
| `currentPosition` | string | 可选 | `Point-0007` | 若 ROS2 侧能识别 JVS 点位，可回传当前最近点位。 |
| `lastNodeId` | string | 可选 | `Point-0006` | 最近经过点位。不能识别时可不传，由 JVS 投影。 |
| `nextPosition` | string | 可选 | `Point-0010` | 下一目标点位。 |

### 5.2 状态枚举

`state` 建议枚举：

| 值 | 含义 |
| --- | --- |
| `IDLE` | 空闲，可接单。 |
| `WORKING` | 正在执行导航或任务。 |
| `CHARGING` | 充电中。 |
| `PAUSED` | 已暂停。 |
| `ERROR` | 故障。 |
| `OFFLINE` | 离线。 |
| `MANUAL` | 人工接管。 |

`dispatchStatus` 建议枚举：

| 值 | 对应 Nav2/openTCS 语义 |
| --- | --- |
| `UNKNOWN` | 未知。 |
| `ACCEPTED` | goal 已被 Nav2 接收。 |
| `EXECUTING` | 正在导航。 |
| `SUCCEEDED` | 到达目标。 |
| `CANCELING` | 取消中。 |
| `CANCELED` | 已取消。 |
| `ABORTED` | 执行失败。 |
| `BLOCKED` | 被障碍物或局部规划阻塞。 |
| `REJECTED` | goal 被拒绝。 |

`localizationStatus` 建议枚举：

| 值 | 含义 |
| --- | --- |
| `OK` | 定位正常。 |
| `DEGRADED` | 定位质量下降，但可运行。 |
| `LOST` | 定位丢失，禁止继续调度。 |
| `INITIALIZING` | 定位初始化中。 |

## 6. 事件接口补充规范

当前 Sidecar 已有：

```http
GET /api/v1/events?limit=100
```

建议 ROS2/Sidecar 升级为游标拉取，避免高频状态下丢事件：

```http
GET /api/v1/events?afterId=1024000&limit=200
```

响应：

```json
{
  "success": true,
  "nextAfterId": "1024012",
  "events": [
    {
      "eventId": "1024012",
      "eventType": "POSE_UPDATE",
      "vehicleId": "AGV_01",
      "vehicleName": "AGV_01",
      "accessIdentity": "robot_01",
      "timestamp": 1779445800.123,
      "frameId": "map",
      "x": 12.345,
      "y": -3.21,
      "yaw": 1.5708,
      "battery": 76.5,
      "state": "WORKING",
      "dispatchStatus": "EXECUTING",
      "localizationStatus": "OK",
      "emergencyStop": false,
      "safetyStop": false,
      "obstacleDetected": false,
      "currentTransportOrder": "TO-202605220001",
      "goalId": "TO-202605220001:0",
      "message": "pose update"
    }
  ]
}
```

事件类型要求：

| `eventType` | 必填字段 | 触发时机 |
| --- | --- | --- |
| `HEARTBEAT` | `vehicleId/timestamp/state/battery` | 周期心跳，建议 1 Hz。 |
| `POSE_UPDATE` | `vehicleId/timestamp/frameId/x/y/yaw` | 位姿更新，建议输出 2 到 10 Hz；JVS 可按配置降频。 |
| `GOAL_ACCEPTED` | `vehicleId/orderId/goalId/status` | Nav2 接收目标后立即输出。 |
| `GOAL_REJECTED` | `vehicleId/orderId/goalId/faultCode/faultMessage` | 目标不可达或 action server 拒绝。 |
| `GOAL_STATUS` 或 `NAV_STATUS` | `vehicleId/orderId/goalId/status/statusCode` | Nav2 status 变化时输出。 |
| `GOAL_REACHED` | `vehicleId/orderId/goalId/x/y/yaw` | 到达目标。 |
| `GOAL_CANCELED` | `vehicleId/orderId/goalId` | 取消完成。 |
| `GOAL_ABORTED` | `vehicleId/orderId/goalId/faultCode/faultMessage` | Nav2 失败。 |
| `FAULT` | `vehicleId/faultCode/faultMessage/state` | 急停、定位丢失、控制器异常等。 |
| `FAULT_CLEARED` | `vehicleId/faultCode` | 故障恢复。 |

JVS 当前兼容 `eventType/type`、`vehicleId/vehicleName`、`attributes/data`，但 ROS2 侧应按上面的扁平字段输出，减少解析歧义。

## 7. 机器人状态快照接口建议

建议 Sidecar 新增：

```http
GET /api/v1/robots/status
GET /api/v1/robots/{vehicleName}/status
```

响应：

```json
{
  "success": true,
  "robots": [
    {
      "vehicleName": "AGV_01",
      "accessIdentity": "robot_01",
      "namespace": "/agv_01",
      "enabled": true,
      "online": true,
      "lastHeartbeatAt": 1779445800123,
      "state": "WORKING",
      "dispatchStatus": "EXECUTING",
      "pose": {
        "frameId": "map",
        "x": 12.345,
        "y": -3.21,
        "yaw": 1.5708,
        "updatedAt": 1779445800123
      },
      "battery": {
        "percentage": 76.5,
        "voltage": 25.1,
        "current": -3.2,
        "charging": false,
        "updatedAt": 1779445800123
      },
      "nav2": {
        "lifecycleState": "active",
        "actionServerReady": true,
        "currentGoalId": "TO-202605220001:0",
        "status": "EXECUTING",
        "statusCode": 2,
        "distanceRemaining": 3.4,
        "estimatedTimeRemaining": 12.0
      },
      "safety": {
        "emergencyStop": false,
        "safetyStop": false,
        "obstacleDetected": false
      },
      "fault": {
        "code": "",
        "message": ""
      }
    }
  ]
}
```

该接口用于页面首次加载、Sidecar 重启后状态补偿和事件丢失后的校准。

## 8. 控制接口补充规范

### 8.1 发送导航目标

当前已有：

```http
POST /api/v1/goals
```

建议请求体完整字段：

```json
{
  "requestId": "jvs-1779445800001",
  "orderId": "TO-202605220001",
  "movementCommandId": "TO-202605220001:0",
  "goalId": "TO-202605220001:0",
  "vehicleName": "AGV_01",
  "vehicleId": "AGV_01",
  "accessIdentity": "robot_01",
  "namespace": "/agv_01",
  "domainId": 42,
  "rmwImplementation": "rmw_fastrtps_cpp",
  "mapId": "auto_exploration_map",
  "mapVersion": "2026-05-22T01",
  "frameId": "map",
  "x": 12.345,
  "y": -3.21,
  "yaw": 1.5708,
  "quaternionZ": 0.7071,
  "quaternionW": 0.7071,
  "goalToleranceMeters": 0.35,
  "yawToleranceRad": 0.25,
  "maxSpeed": 0.5,
  "allowReverse": true
}
```

响应：

```json
{
  "success": true,
  "accepted": true,
  "goalId": "TO-202605220001:0",
  "status": "ACCEPTED",
  "message": "Nav2 goal accepted",
  "timestamp": 1779445800123
}
```

要求：

1. `goalId` 必须进入 Nav2 goal 映射，后续 status/result/cancel 都必须携带同一个 `goalId`。
2. 同一车辆存在未完成 goal 时，新 goal 不应静默覆盖旧 goal；必须先取消旧 goal 或返回明确冲突。
3. 如果目标不可达，返回 `success=false`、`status=REJECTED`，并给出 `faultCode/faultMessage`。

### 8.2 取消导航

当前接口存在，但 ROS2 runtime 中仍有“需要通过 Nav2 action 或自定义服务实现”的风险。ROS2 侧需要实现真实取消。

```http
POST /api/v1/goals/cancel
```

请求：

```json
{
  "requestId": "jvs-1779445801001",
  "orderId": "TO-202605220001",
  "movementCommandId": "TO-202605220001:0",
  "goalId": "TO-202605220001:0",
  "vehicleName": "AGV_01",
  "vehicleId": "AGV_01",
  "reason": "operator cancel"
}
```

响应：

```json
{
  "success": true,
  "accepted": true,
  "goalId": "TO-202605220001:0",
  "status": "CANCELING",
  "message": "cancel accepted"
}
```

最终还必须通过事件输出：

```json
{
  "eventType": "GOAL_CANCELED",
  "vehicleId": "AGV_01",
  "orderId": "TO-202605220001",
  "goalId": "TO-202605220001:0",
  "status": "CANCELED"
}
```

### 8.3 暂停与恢复

当前 HTTP 接口已预留：

```http
POST /api/v1/navigation/pause
POST /api/v1/navigation/resume
```

ROS2 侧需要明确采用以下任一实现方式：

1. Nav2 behavior tree 支持 pause/resume。
2. lifecycle service 暂停 controller 或 navigator。
3. 自定义 ROS2 service，例如 `/agv_01/navigation/pause`、`/agv_01/navigation/resume`。

如果暂不支持，必须保持明确失败：

```json
{
  "success": false,
  "accepted": false,
  "status": "UNSUPPORTED",
  "message": "Nav2 pause 未配置标准服务"
}
```

不能返回成功但实际不执行。

### 8.4 初始位姿设置

当前已有：

```http
POST /api/v1/initial-poses
```

建议补充 covariance：

```json
{
  "requestId": "jvs-1779445802001",
  "vehicleName": "AGV_01",
  "vehicleId": "AGV_01",
  "namespace": "/agv_01",
  "frameId": "map",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "quaternionZ": 0.0,
  "quaternionW": 1.0,
  "covariance": [
    0.25, 0, 0, 0, 0, 0,
    0, 0.25, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0.0685
  ]
}
```

### 8.5 建议新增辅助控制接口

| 接口 | 方法 | 用途 | 优先级 |
| --- | --- | --- | --- |
| `/api/v1/navigation/clear-costmaps` | POST | 清理 Nav2 costmap。 | P1 |
| `/api/v1/navigation/recover` | POST | 触发恢复行为。 | P1 |
| `/api/v1/navigation/validate-goal` | POST | 只校验目标点是否可达，不真正导航。 | P1 |
| `/api/v1/robots/{vehicleName}/health` | GET | 查询单车 Nav2/action/topic 健康状态。 | P0 |
| `/api/v1/robots/status` | GET | 批量状态快照。 | P0 |

## 9. ROS2 Topic/Action 要求

### 9.1 必须稳定发布

| Topic/Action | 类型 | 频率 | 要求 |
| --- | --- | --- | --- |
| `/{namespace}/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 5 到 10 Hz | 必须包含 header.stamp、header.frame_id、position、orientation、covariance。 |
| `/{namespace}/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | 状态变化立即发布，至少 1 Hz | 必须能关联当前 goal。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 按机器人配置 | 必须存在 `map -> odom -> baseFrame` 或等效 TF 链。 |
| `/{namespace}/battery_state` | `sensor_msgs/msg/BatteryState` 或自定义映射 | 1 Hz | 建议提供电量、充电状态、电压、电流。 |
| `/{namespace}/robot_state` | 自定义或标准诊断 topic | 1 Hz | 建议提供急停、安全停、故障码、定位状态。 |

### 9.2 Nav2 action result 要求

ROS2 侧不能只发布简化版 `EXECUTING`。需要在 goal 最终结束时输出准确结果：

| Nav2 status code | 文本 | JVS 处理 |
| --- | --- | --- |
| `1` | `ACCEPTED` | goal 已接收。 |
| `2` | `EXECUTING` | 车辆工作中。 |
| `3` | `CANCELING` | 取消中。 |
| `4` | `SUCCEEDED` | 到点成功，CommAdapter 可完成当前 MovementCommand。 |
| `5` | `CANCELED` | 订单取消或撤回。 |
| `6` | `ABORTED` | 执行失败，需要故障原因。 |

## 10. 健康检查要求

`GET /health` 需要返回 Sidecar 进程和 ROS2 通信健康状态：

```json
{
  "success": true,
  "runtimeMode": "ros2",
  "domainId": 42,
  "rmwImplementation": "rmw_fastrtps_cpp",
  "rosReady": true,
  "robots": [
    {
      "vehicleName": "AGV_01",
      "namespace": "/agv_01",
      "enabled": true,
      "actionServerReady": true,
      "amclPoseFresh": true,
      "lastPoseAt": 1779445800123,
      "lastStatusAt": 1779445800110,
      "lastHeartbeatAt": 1779445800120
    }
  ],
  "state": {
    "goalSessionCount": 1,
    "poseVehicleCount": 1,
    "eventCount": 120,
    "lastRobotSyncVersion": "ros2-binding-1779445700000",
    "lastRobotSyncTime": 1779445700123,
    "lastRobotSyncMessage": "已同步 1 个机器人配置"
  }
}
```

## 11. 关键验收用例

### 11.1 配置同步验收

1. JVS 车辆管理新增 `AGV_01`，填写 `namespace=/agv_01`、`domainId=42`。
2. device-gateway 同步到 Sidecar `/api/v1/robots/sync`。
3. Sidecar `/api/v1/robots` 返回 `AGV_01`。
4. `/health` 中 `robots[0].actionServerReady=true`。

### 11.2 位姿同步验收

1. ROS2 发布 `/agv_01/amcl_pose`。
2. Sidecar `/api/v1/events?limit=10` 出现 `POSE_UPDATE`。
3. 事件中包含 `vehicleId=AGV_01`、`frameId=map`、`x/y/yaw`。
4. JVS 车辆详情显示 ROS2 pose，并且地图车辆图标位置和朝向变化。

### 11.3 导航闭环验收

1. JVS 通过 openTCS Kernel 下发目标。
2. Sidecar `/api/v1/goals` 返回 `accepted=true` 和 `goalId`。
3. ROS2 Nav2 status 输出 `ACCEPTED -> EXECUTING -> SUCCEEDED`。
4. Sidecar events 输出同一 `goalId` 的 `GOAL_ACCEPTED/GOAL_STATUS/GOAL_REACHED`。
5. JVS 侧车辆状态从 `WORKING` 回到 `IDLE`。

### 11.4 取消验收

1. 车辆执行中调用 `/api/v1/goals/cancel`。
2. ROS2 调用真实 Nav2 action cancel。
3. Sidecar 先返回 `CANCELING`，最终输出 `GOAL_CANCELED`。
4. JVS 订单、车辆、资源锁状态进入取消链路。

### 11.5 异常验收

1. 目标点不可达时，Sidecar 返回 `success=false/status=REJECTED`。
2. Nav2 执行失败时，事件输出 `GOAL_ABORTED`，并携带 `faultCode/faultMessage`。
3. AMCL 断流超过 `heartbeatTimeoutMs` 时，输出 `state=OFFLINE` 或 `localizationStatus=LOST`。
4. 急停触发时，输出 `emergencyStop=true/state=ERROR`，JVS 禁止继续自动派单。

## 12. ROS2 侧开发清单

必须完成：

1. 给每台机器人稳定提供 `vehicleName/accessIdentity/namespace/domainId/frameId/baseFrame`。
2. `/amcl_pose` 必须携带准确 `x/y/yaw/frameId/timestamp/covariance`。
3. Sidecar events 必须补 `eventId`、`vehicleId`、`timestamp`、`x/y/yaw`、`battery`、`state`、`dispatchStatus`。
4. Nav2 status 必须能关联 `goalId/orderId`。
5. 实现真实 Nav2 action cancel，不能只返回“不支持”。
6. `/health` 必须暴露每台机器人 action server、AMCL、status、heartbeat 新鲜度。
7. 提供电量、急停、安全停、定位质量、故障码的 topic 或状态 API。

建议完成：

1. 新增 `/api/v1/robots/status` 批量状态快照。
2. 新增 `/api/v1/events?afterId=` 游标拉取。
3. 新增 goal 可达性校验接口。
4. 新增 clear costmap / recover 接口。
5. 提供 YAML/PGM checksum，防止地图版本不一致。

## 13. 双方边界

ROS2 侧负责：

1. 机器人原生 topic/action/service。
2. Nav2 goal 执行、取消、状态、反馈。
3. AMCL/TF 位姿。
4. 电量、安全、故障、定位质量。
5. 地图 YAML/PGM 元数据和版本一致性。

JVS/openTCS 侧负责：

1. 车辆、地图、业务位置、库存、任务模板、物流需求单。
2. openTCS Kernel PlantModel、TransportOrder、资源调度和路径规划。
3. 将 Kernel MovementCommand 转换成 ROS2 goal。
4. 将 ROS2 状态映射回车辆状态、订单状态、执行任务和前端监控。
5. 资源锁、库存占用、业务进度和异常恢复。

核心原则：ROS2 侧只证明“底盘是否执行到目标”，业务订单是否完成必须由 openTCS Kernel 与 JVS 执行链路共同判定。
