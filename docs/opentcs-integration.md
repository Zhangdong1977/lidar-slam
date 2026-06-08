# openTCS-NeNa / Sidecar 对接说明

> 本文档供 openTCS sidecar 端对接使用，内容按当前项目实现整理：ROS2 Jazzy、Fast DDS、`opentcs_vehicle_node`、`route_graph_loader` 和 `dispatch.sh`。

---

## 1. 当前对接模型

openTCS/JVS 不直接调用 Nav2。sidecar 作为 ROS2 节点加入同一个 DDS 域，通过 ROS2 topic/action 与机器人端通信。

```
openTCS Kernel / JVS
        │
        │ HTTP / Java adapter
        ▼
jvs-opentcs-ros2-sidecar
        │
        │ ROS2 DDS, Fast DDS Discovery Server
        ▼
/{vehicle_name}/goal_pose        ──▶ opentcs_vehicle_node ──▶ Nav2 NavigateToPose
/{vehicle_name}/route_graph_json ──▶ route_graph_loader    ──▶ Nav2 route_server
/{vehicle_name}/load_materials   ──▶ material_action_server
/{vehicle_name}/unload_materials ──▶ material_action_server

/{vehicle_name}/amcl_pose        ◀── opentcs_vehicle_node
/{vehicle_name}/robot_state      ◀── opentcs_vehicle_node
/{vehicle_name}/battery_state    ◀── opentcs_vehicle_node
```

`dispatch.sh` 默认将 `vehicle_name` 设置为运行时 `namespace`。例如默认启动为 `gazebo_1`，sidecar 需要使用的 ROS2 接口前缀就是 `/gazebo_1/...`。

---

## 2. 机器人端启动

默认仿真启动，连接 sidecar 端 Discovery Server：

```bash
bash scripts/launch/dispatch.sh --discovery-address <sidecar_ip>
```

常用参数：

```bash
# 第二辆仿真车
bash scripts/launch/dispatch.sh --namespace gazebo_2 --discovery-address <sidecar_ip>

# RS-485 实车
bash scripts/launch/dispatch.sh --profile rs485 --namespace c30_1 --discovery-address <sidecar_ip>

# 树莓派实车
bash scripts/launch/dispatch.sh --profile raspberry --namespace c30_1 --discovery-address <sidecar_ip>

# 指定地图
bash scripts/launch/dispatch.sh --map /path/to/map.yaml --discovery-address <sidecar_ip>

# 指定 sidecar 端 Discovery Server 地址
bash scripts/launch/dispatch.sh --discovery-address 192.168.1.10

# 不使用 Discovery Server，退回 DDS multicast
bash scripts/launch/dispatch.sh --no-discovery-server
```

启动脚本会打印当前 `Namespace`、`Domain`、`DDS发现`、地图和日志路径。sidecar 对接时以这几项为准。

---

## 3. Discovery Server 配置

### 3.1 默认行为

`dispatch.sh` 默认启用 Fast DDS Discovery Server 客户端模式。Discovery Server 由 sidecar 端提供，机器人端只连接，不启动本地 server：

| 项目 | 默认值 | 说明 |
|------|--------|------|
| Discovery Server | 启用 | 除非传入 `--no-discovery-server` |
| Port | `11811` | `--discovery-port` 可覆盖 |
| Client endpoint | 无内置默认值 | 通过 `--discovery-address <sidecar_ip>` 或环境变量 `ROS_DISCOVERY_SERVER` 指定 |
| RMW | `rmw_fastrtps_cpp` | 机器人端强制导出 |

机器人端 ROS2 进程会导出或继承：

```bash
export ROS_DISCOVERY_SERVER=<sidecar_ip>:11811
```

### 3.2 sidecar 必须设置的环境变量

sidecar 进程启动前设置：

```bash
source /opt/ros/jazzy/setup.bash
source <sidecar_ws>/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=<机器人启动日志中的 Domain>
export ROS_DISCOVERY_SERVER=<sidecar_ip>:11811
unset ROS_LOCALHOST_ONLY
```

示例：默认仿真 `gazebo_1`：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=192.168.1.10:11811
unset ROS_LOCALHOST_ONLY
```

示例：默认 RS-485/树莓派 profile：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
export ROS_DISCOVERY_SERVER=192.168.1.20:11811
unset ROS_LOCALHOST_ONLY
```

### 3.3 Domain ID 来源

Domain ID 不再固定为单一值，来源如下：

| profile | 默认 Domain ID | 配置文件 |
|---------|----------------|----------|
| `gazebo` | `42` | `config/profiles/gazebo.yaml` |
| `rs485` | `30` | `config/profiles/rs485.yaml` |
| `raspberry` | `30` | `config/profiles/raspberry.yaml` |

也可以启动机器人时显式覆盖：

```bash
bash scripts/launch/dispatch.sh --profile rs485 --namespace c30_1 --domain-id 42 --discovery-address <sidecar_ip>
```

sidecar 的 `ROS_DOMAIN_ID` 必须与机器人端完全一致，否则即使 Discovery Server 地址正确，也发现不到 topic/action。

### 3.4 多网卡/远端 sidecar 注意事项

如果 sidecar 有多个网卡，需要使用机器人可访问的 sidecar IP。机器人端启动时显式指定：

```bash
bash scripts/launch/dispatch.sh \
  --namespace c30_1 \
  --discovery-address 192.168.10.10
```

sidecar ROS2 进程使用同一个 endpoint：

```bash
export ROS_DISCOVERY_SERVER=192.168.10.10:11811
```

如果端口被占用，可同步修改机器人端和 sidecar 端端口：

```bash
bash scripts/launch/dispatch.sh --discovery-address 192.168.10.10 --discovery-port 11888
export ROS_DISCOVERY_SERVER=192.168.10.10:11888
```

### 3.5 连通性验证

在 sidecar 机器上执行：

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ROS_DISCOVERY_SERVER

ros2 topic list | grep -E "/gazebo_1|/c30_1"
```

默认 `gazebo_1` 应至少看到：

```text
/gazebo_1/amcl_pose
/gazebo_1/battery_state
/gazebo_1/goal_pose
/gazebo_1/robot_state
/gazebo_1/route_graph_json
```

如果看不到 topic，优先检查：

1. sidecar 是否设置了 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
2. sidecar 的 `ROS_DOMAIN_ID` 是否等于机器人启动日志中的 `Domain`
3. 机器人端和 sidecar 端的 `ROS_DISCOVERY_SERVER` 是否为 sidecar 可达 IP 和端口
4. sidecar 防火墙是否放行 Discovery Server 端口，如 UDP/TCP `11811`
5. 是否误设置了 `ROS_LOCALHOST_ONLY=1`

---

## 4. Topic 接口

以下以 `vehicle_name=gazebo_1` 为例。实车常用 `vehicle_name=c30_1`。实际前缀以机器人启动参数为准。

### 4.1 sidecar 发布

| Topic | 类型 | QoS/频率 | 说明 |
|-------|------|----------|------|
| `/{vehicle_name}/goal_pose` | `geometry_msgs/PoseStamped` | 普通 reliable 即可 | 下发导航目标，`frame_id` 使用 `map` |
| `/{vehicle_name}/route_graph_json` | `std_msgs/String` | 建议 reliable + transient local，depth=1 | 下发 Nav2 route graph GeoJSON |

导航目标示例：

```bash
ros2 topic pub --once /gazebo_1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

如果 sidecar 需要把 openTCS 订单号带给机器人状态，可把 `orderId` 放在 `frame_id` 后缀中：

```text
map/orderId=TO-20260604-001
```

`opentcs_vehicle_node` 会解析 `orderId`，再把 `frame_id` 还原为 `map` 后发送给 Nav2。

### 4.2 机器人发布

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/{vehicle_name}/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 10 Hz | map frame 下的实时位姿 |
| `/{vehicle_name}/robot_state` | `std_msgs/String` | 1 Hz | JSON 字符串，包含车辆状态、任务状态、定位质量、故障等 |
| `/{vehicle_name}/battery_state` | `sensor_msgs/BatteryState` | 1 Hz | 电池状态；当前默认仿真电量 |

`robot_state` JSON 主要字段：

| 字段 | 说明 |
|------|------|
| `vehicleName`, `vehicleId` | 车辆名，等于 `vehicle_name` |
| `x`, `y`, `yaw` | map frame 位姿，单位 m/rad |
| `linearVelocity`, `angularVelocity` | 速度 |
| `battery`, `charging` | 电量百分比和充电状态 |
| `state` | `IDLE`, `WORKING`, `CHARGING`, `PAUSED`, `ERROR`, `OFFLINE`, `MANUAL` |
| `dispatchStatus` | `UNKNOWN`, `ACCEPTED`, `EXECUTING`, `SUCCEEDED`, `CANCELING`, `CANCELED`, `ABORTED`, `BLOCKED`, `REJECTED` |
| `localizationStatus` | `OK`, `DEGRADED`, `LOST`, `INITIALIZING` |
| `obstacleDetected` | 障碍物检测状态 |
| `faultCode`, `faultMessage` | 故障码和故障信息 |
| `currentTransportOrder` | 从 `goal_pose.header.frame_id` 解析出的 `orderId` |
| `goalId` | 当前目标 ID |
| `distanceRemaining` | Nav2 feedback 剩余距离 |
| `estimatedTimeRemaining` | Nav2 feedback 预计剩余时间 |
| `mapChecksum` | 地图 checksum，未配置地图文件时为空 |

---

## 5. Action 接口

### 5.1 导航 Action

sidecar 不需要直接调用 Nav2 action。机器人端内部使用：

| Action | 类型 | 说明 |
|--------|------|------|
| `/{namespace}/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `opentcs_vehicle_node` 内部调用 |

sidecar 只发布 `/{vehicle_name}/goal_pose`，并通过 `/{vehicle_name}/robot_state.dispatchStatus` 追踪任务状态。

### 5.2 物料 Action

`material_action_gui` 在 namespace 下提供两个 action server：

| Action | 类型 | 说明 |
|--------|------|------|
| `/{namespace}/load_materials` | `jvs_agv_material_msgs/action/LoadMaterials` | 装料 |
| `/{namespace}/unload_materials` | `jvs_agv_material_msgs/action/UnloadMaterials` | 卸料 |

Goal 字段：

```text
string job_id
string request_id
string order_id
string order_no
int32 step_no
string vehicle_name
string point_id
string location_id
MaterialSpec[] materials
string trace_id
string request_payload_json
```

`job_id` 和 `materials` 不能为空；`vehicle_name` 不匹配时只告警，不直接拒绝。

---

## 6. route graph 接口

sidecar 发布：

```text
/{vehicle_name}/route_graph_json
```

消息类型：

```text
std_msgs/String
```

内容是 GeoJSON 字符串，顶层必须为：

```json
{
  "type": "FeatureCollection",
  "features": []
}
```

`route_graph_loader` 使用 transient local QoS 订阅，收到后会：

1. 校验 JSON
2. 统计 Point 节点和 LineString/MultiLineString 边
3. 保存到 `/tmp/route_graph.geojson`
4. 调用 `/{namespace}/route_server/set_route_graph`
5. 发布 `/{namespace}/route_graph/markers` 供 RViz 显示

建议 sidecar 发布 route graph 时使用 reliable + transient local，确保机器人端晚启动 `route_graph_loader` 时仍能拿到最后一份地图。

---

## 7. 地图与坐标

| 项目 | 值 |
|------|-----|
| 默认地图 | `maps/auto_exploration_map.yaml` |
| 坐标系 | `map` |
| ROS2 单位 | m/rad |
| 航向 | 标准 ROS：X 正方向 yaw=0，逆时针为正 |
| 默认仿真初始位置 | `gazebo_1` 为 `(0, 0, 0)`；`gazebo_N` 默认 y 偏移 `2*(N-1)` |

openTCS 内部若使用 mm，需要在 sidecar/adapter 中完成单位转换：

```text
ROS2 坐标(m) = openTCS 坐标(mm) / 1000 * plantModelScale
```

请确认 sidecar 当前使用的 `plantModelScale`。如果 `plantModelScale=1.0`，openTCS 的 `1000 mm` 对应 ROS2 的 `1.0 m`。

---

## 8. 对接验证流程

以下以默认仿真 `gazebo_1` 为例。

### 8.1 验证 DDS 发现

```bash
ros2 topic list | grep /gazebo_1
```

### 8.2 验证位姿上报

```bash
ros2 topic hz /gazebo_1/amcl_pose
```

期望约 `10 Hz`。

### 8.3 验证状态上报

```bash
ros2 topic echo /gazebo_1/robot_state --once
```

期望看到 JSON，且 `vehicleName` 为 `gazebo_1`。

### 8.4 手动下发导航目标

```bash
ros2 topic pub --once /gazebo_1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map/orderId=TEST-001'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

观察：

```bash
ros2 topic echo /gazebo_1/robot_state
```

状态应按以下路径变化：

```text
dispatchStatus: ACCEPTED -> EXECUTING -> SUCCEEDED
state: WORKING -> IDLE
currentTransportOrder: TEST-001
```

---

## 9. 常见问题

### 9.1 sidecar 看不到任何 topic

优先检查 `ROS_DOMAIN_ID` 和 `ROS_DISCOVERY_SERVER`。Discovery Server 地址必须是机器人可访问的 sidecar IP。

### 9.2 能看到 topic，但目标发布后机器人不动

检查：

1. `/{vehicle_name}/robot_state.faultCode`
2. 目标 `frame_id` 是否为 `map` 或 `map/orderId=...`
3. 目标点是否在全局代价地图范围内
4. Nav2 是否已启动完成：`ros2 action list | grep navigate_to_pose`

### 9.3 route graph 发布后没有加载

检查：

1. GeoJSON 顶层是否为 `FeatureCollection`
2. sidecar 是否发布到了 `/{vehicle_name}/route_graph_json`
3. 是否使用 transient local QoS
4. `/{namespace}/route_server/set_route_graph` 是否可用

### 9.4 多车时 topic 前缀混乱

保持 `namespace == vehicle_name`。例如：

```bash
bash scripts/launch/dispatch.sh --namespace c30_1 --discovery-address <sidecar_ip>
```

此时 sidecar 使用：

```text
/c30_1/goal_pose
/c30_1/amcl_pose
/c30_1/robot_state
/c30_1/route_graph_json
/c30_1/load_materials
/c30_1/unload_materials
```

---

## 10. 对接速查

默认仿真：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=<sidecar_ip>:11811
unset ROS_LOCALHOST_ONLY

ros2 topic echo /gazebo_1/robot_state --once
ros2 topic pub --once /gazebo_1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

默认实车：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
export ROS_DISCOVERY_SERVER=<sidecar_ip>:11811
unset ROS_LOCALHOST_ONLY

ros2 topic echo /c30_1/robot_state --once
```
