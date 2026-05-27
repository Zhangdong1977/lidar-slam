# openTCS Sidecar 路由图发布接口需求

> 日期：2026-05-26
> 状态：待确认
> 编写：ROS2 侧

## 1. 背景

Nav2 的 `route_server`（`nav2_route` 包）用于基于预定义路网图的 AGV 路由规划，支持：

- 按路网图计算全局路由（替代/补充自由空间规划器）
- 边级速度限制、单向通行、通行方向约束
- 路由跟踪、碰撞检测、动态重路由
- 多车场景下的边锁定/阻塞管理

目前 `route_server` 启动时 `graph_filepath` 为空（无路网图文件），需要 Sidecar 在车辆上线时将 openTCS 的 Plant Model 转换为路网图并通过 ROS2 话题发布，ROS2 侧接收后调用 `SetRouteGraph` 服务加载。

## 2. 接口定义

### 2.1 话题：`/route_graph`

| 字段 | 值 |
|------|-----|
| Topic | `/route_graph` |
| 类型 | `std_msgs/msg/String` |
| QoS | Reliable, Transient Local Durability (保留最新一条，新订阅者立即收到) |
| 发布方 | Sidecar |
| 订阅方 | ROS2 侧 `route_graph_loader` 节点（待开发） |
| 内容 | GeoJSON 字符串（见第 3 节格式） |
| 触发时机 | (1) Sidecar 启动时发布一次；(2) openTCS Plant Model 变更时重新发布 |

### 2.2 为什么用 topic 而不是 service

- **Transient Local Durability** 保证后启动的 ROS2 节点也能收到（车辆启动顺序不确定）
- Plant Model 变更时 Sidecar 重新发布，ROS2 侧自动热更新，无需轮询
- 不需要 ROS2 侧发起请求，解耦启动顺序

### 2.3 ROS2 侧行为

ROS2 侧新增 `route_graph_loader` 节点：

1. 订阅 `/route_graph`
2. 收到 GeoJSON 字符串后写入临时文件 `/tmp/route_graph.geojson`
3. 调用 `route_server` 的 `SetRouteGraph` 服务（`nav2_msgs/srv/SetRouteGraph`），传入文件路径
4. 日志输出加载结果

## 3. GeoJSON 格式规范

route_server 使用 `GeoJsonGraphFileLoader` 解析，遵循以下结构：

### 3.1 完整示例

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [10.0, 5.0, 0.0]
      },
      "properties": {
        "id": 1,
        "label": "P1"
      }
    },
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [20.0, 5.0, 0.0]
      },
      "properties": {
        "id": 2,
        "label": "P2"
      }
    },
    {
      "type": "Feature",
      "geometry": {
        "type": "LineString",
        "coordinates": [[10.0, 5.0, 0.0], [20.0, 5.0, 0.0]]
      },
      "properties": {
        "id": 101,
        "startid": 1,
        "endid": 2,
        "bidirectional": true,
        "speed_limit": 0.8,
        "overhead_clearance": 2.5
      }
    }
  ]
}
```

### 3.2 字段说明

#### Node（节点 = openTCS Point）

| GeoJSON 字段 | 类型 | 必需 | 说明 |
|-------------|------|------|------|
| `geometry.type` | string | 是 | 固定 `"Point"` |
| `geometry.coordinates` | [x, y, z] | 是 | 地图坐标系下的坐标（米），z 通常为 0 |
| `properties.id` | uint16 | 是 | 节点 ID，全局唯一，与边引用一致 |
| `properties.label` | string | 否 | 节点名称/标签，如 `"Charger-1"`、`"WP-A03"` |

#### Edge（边 = openTCS Path）

| GeoJSON 字段 | 类型 | 必需 | 说明 |
|-------------|------|------|------|
| `geometry.type` | string | 是 | 固定 `"LineString"` |
| `geometry.coordinates` | [[x,y,z], [x,y,z]] | 是 | 起点→终点的两个坐标 |
| `properties.id` | uint16 | 是 | 边 ID，全局唯一 |
| `properties.startid` | uint16 | 是 | 起点节点 ID |
| `properties.endid` | uint16 | 是 | 终点节点 ID |
| `properties.bidirectional` | bool | 否（默认 true） | `true`=双向通行，`false`=单向（仅 start→end） |
| `properties.speed_limit` | float | 否 | 速度限制，百分比（0.0-1.0）或绝对值 m/s（取决于 DistanceScorer/TimeScorer 配置） |
| `properties.penalty` | float | 否 | 边惩罚值（使用 PenaltyScorer 时生效） |
| `properties.overhead_clearance` | float | 否 | 净空高度（米） |
| `properties.class` | string | 否 | 语义类别（使用 SemanticScorer 时生效），如 `"corridor"`, `"intersection"`, `"charging_zone"` |

### 3.3 坐标系

- **坐标系**：与 ROS2 `map` frame 一致（右手系，x 前，y 左，单位米）
- **原点**：与 openTCS Kernel Plant Model 中的布局坐标对齐
- 如果 Sidecar 发布的坐标与 `map` frame 不一致，route_server 会通过 TF 自动变换（需配置 `route_frame: "map"`）

### 3.4 与 openTCS Plant Model 的映射关系

```
openTCS Plant Model          →    GeoJSON 路由图
─────────────────────────────────────────────────
Point (x, y)                 →    Node (Point geometry)
Path (source → dest, 1:1)    →    Edge (LineString geometry)
Path.routable=true           →    bidirectional=true
Path.routable=false          →    单向，方向取决于定义顺序
Point.name                   →    properties.label
Path.maxVelocity             →    properties.speed_limit (需单位转换)
Point type/location type     →    properties.class
```

### 3.5 ID 分配建议

- Node ID：使用 openTCS Point 的内部 ID 或自增序号（uint16 范围 0-65535）
- Edge ID：使用 openTCS Path 的内部 ID 或自增序号
- **注意**：Node ID 和 Edge ID 必须全局唯一，且边的 `startid`/`endid` 必须引用实际存在的 Node ID

## 4. 端到端数据流

```
openTCS Kernel
  │
  │ Plant Model (Points + Paths)
  ▼
Sidecar
  │
  │ ① 将 Plant Model 转换为 GeoJSON
  │ ② 发布到 /route_graph (std_msgs/String, Transient Local)
  ▼
ROS2 route_graph_loader 节点
  │
  │ ③ 收到 GeoJSON → 写入 /tmp/route_graph.geojson
  │ ④ 调用 route_server/set_route_graph 服务 (nav2_msgs/srv/SetRouteGraph)
  ▼
route_server
  │
  │ ⑤ 解析 GeoJSON → 构建路网图
  │ ⑥ 接受 ComputeRoute / ComputeAndTrackRoute 请求
  ▼
AGV 按路网图规划路由行驶
```

## 5. 伪代码参考（Sidecar 侧）

```python
# Python 伪代码，展示 Sidecar 如何将 openTCS Plant Model 转为 GeoJSON 并发布

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

class RouteGraphPublisher(Node):
    def __init__(self):
        super().__init__('route_graph_publisher')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(String, '/route_graph', qos)

    def publish_plant_model(self, points, paths):
        """将 openTCS Plant Model 发布为 GeoJSON 路由图"""
        features = []

        # 转换 Points → Nodes
        for pt in points:
            features.append({
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [pt.x, pt.y, 0.0]
                },
                "properties": {
                    "id": pt.id,          # uint16
                    "label": pt.name      # 可选
                }
            })

        # 转换 Paths → Edges
        for path in paths:
            src = get_point(path.source_point)
            dst = get_point(path.dest_point)
            features.append({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[src.x, src.y, 0.0], [dst.x, dst.y, 0.0]]
                },
                "properties": {
                    "id": path.id,                        # uint16
                    "startid": src.id,
                    "endid": dst.id,
                    "bidirectional": path.routable,       # openTCS 双向通行标记
                    "speed_limit": path.max_velocity      # 可选，需单位转换
                }
            })

        geojson = json.dumps({
            "type": "FeatureCollection",
            "features": features
        })

        msg = String()
        msg.data = geojson
        self.pub.publish(msg)
        self.get_logger().info(f"Published route graph: {len(points)} nodes, {len(paths)} edges")
```

## 6. QoS 配置参考（ROS2 侧）

```yaml
# /route_graph topic QoS
# Reliability: RELIABLE
# Durability: TRANSIENT_LOCAL
# Depth: 1
#
# 这保证：
# - Sidecar 先启动、发布图 → ROS2 后启动时仍能收到（Transient Local）
# - 消息不丢失（Reliable）
# - 只保留最新版本（Depth=1）
```

## 7. 验证方法

### 7.1 手动验证

```bash
# 终端 1：监听 topic
ros2 topic echo /route_graph --once

# 终端 2：检查 route_server 是否加载成功
ros2 service call /route_server/set_route_graph nav2_msgs/srv/SetRouteGraph \
  "{graph_filepath: '/tmp/route_graph.geojson'}"

# 终端 3：验证路由图已加载（route_server 日志会打印节点/边数量）
```

### 7.2 端到端验证

1. Sidecar 发布 `/route_graph`
2. ROS2 `route_graph_loader` 收到并加载
3. 通过 `ComputeRoute` action 测试路由规划
4. 检查返回的 Route 是否包含预期的节点和边

## 8. 待确认事项

| # | 问题 | 建议 | 状态 |
|---|------|------|------|
| 1 | openTCS Point 的坐标是否与 ROS2 map frame 一致？ | 如果不一致，需提供坐标变换参数 | 待确认 |
| 2 | Plant Model 中是否有单向通道（如单行道）？ | 映射为 `bidirectional: false` | 待确认 |
| 3 | Path.maxVelocity 的单位？ | 需确认是 m/s 还是 km/h，用于 speed_limit 字段 | 待确认 |
| 4 | Sidecar 启动时是否已有完整 Plant Model？ | 需要 Plant Model 就绪后才能发布 | 待确认 |
| 5 | Plant Model 变更时的通知机制？ | 建议变更后重新发布整个 GeoJSON（全量替换） | 待确认 |
| 6 | Node ID / Edge ID 范围是否在 uint16 (0-65535) 内？ | 如果超过需分批或调整 ID 分配策略 | 待确认 |
