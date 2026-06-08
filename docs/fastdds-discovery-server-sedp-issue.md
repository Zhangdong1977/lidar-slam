# FastDDS Discovery Server SEDP 端点发现问题 — 优化需求说明

## 1. 问题描述

FastDDS Discovery Server（部署在 sidecar 10.0.0.187:11811）能正确中继 **PDP（Participant Discovery Protocol，参与者发现协议）** 信息，但未能中继 **SEDP（Simple Endpoint Discovery Protocol，端点发现协议）** 信息，导致 ROS2 CLI 工具和第三方节点无法通过 Discovery Server 发现话题列表，即使实际数据传输通道正常工作。

## 2. 环境信息

| 项目 | 值 |
|------|-----|
| ROS2 版本 | Jazzy |
| DDS 实现 | rmw_fastrtps_cpp (eProsima FastDDS) |
| ROS Domain ID | 42 |
| Discovery Server | 10.0.0.187:11811 (UDP) |
| RMW_IMPLEMENTATION | rmw_fastrtps_cpp |
| FASTDDS_BUILTIN_TRANSPORTS | UDPv4 |
| ROS_AUTOMATIC_DISCOVERY_RANGE | SUBNET |
| 网络拓扑 | 单机仿真，2 个 namespace（gazebo_1, gazebo_2），共 83 个 ROS2 节点 |

## 3. 现象复现步骤

### 步骤 1：启动 dispatch（连接 Discovery Server）
```bash
./scripts/launch/dispatch.sh --namespace gazebo_1 \
    --discovery-address 10.0.0.187 --discovery-port 11811 --no-rviz
```

### 步骤 2：验证 PDP（参与者发现）— 正常 ✅
```bash
export ROS_DISCOVERY_SERVER=10.0.0.187:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 node list | wc -l
# 输出: 83 个节点（gazebo_1: 38, gazebo_2: 38, 全局: 7）
```

### 步骤 3：验证 SEDP（端点发现）— 异常 ❌
```bash
ros2 topic list
# 输出仅:
#   /parameter_events
#   /rosout
# 缺少所有 /gazebo_1/* 和 /gazebo_2/* 话题
```

### 步骤 4：验证数据传输 — 正常 ✅
```python
# Python subscriber 可以正常接收传感器数据
import rclpy
from sensor_msgs.msg import LaserScan
rclpy.init()
node = rclpy.create_node('test')
node.create_subscription(LaserScan, '/gazebo_1/scan', lambda m: print(f"ranges={len(m.ranges)}"), 10)
# 输出: ranges=720  ← 数据正常流动
```

## 4. 详细诊断数据

### 4.1 PDP 层 — 参与者发现（正常）

通过 Discovery Server，所有 83 个 ROS2 节点均可被发现：
- `/gazebo_1/*`: 38 个节点（amcl, controller_server, ekf_filter_node 等）
- `/gazebo_2/*`: 38 个节点
- 全局节点: 7 个（jvs_opentcs_ros2_sidecar, rviz, rviz_tf_bridge 等）

### 4.2 SEDP 层 — 端点元数据发现（异常）

| API 调用 | 预期结果 | 实际结果 |
|----------|----------|----------|
| `ros2 topic list` | 100+ 话题 | 仅 2 个（DDS 内置） |
| `ros2 topic info /gazebo_1/scan` | pub≥1, sub≥1 | "Unknown topic" |
| `node.get_topic_names_and_types()` | 包含 scan/odom/imu 等 | 仅 /parameter_events, /rosout |
| `node.count_publishers('/gazebo_1/scan')` | ≥1 | 0 |
| `node.count_subscribers('/gazebo_1/scan')` | ≥1 | 0 |

### 4.3 数据平面 — 实际数据传输（正常）

| 话题 | 消息类型 | 数据接收 |
|------|----------|----------|
| `/gazebo_1/scan` | sensor_msgs/msg/LaserScan | ✅ 720 ranges |
| `/gazebo_1/odom` | nav_msgs/msg/Odometry | ✅ 正常 |
| `/gazebo_1/imu` | sensor_msgs/msg/Imu | ✅ 正常 |
| `/gazebo_1/map` | nav_msgs/msg/OccupancyGrid | ❌ (latched，可能未推送) |
| `/gazebo_1/cmd_vel` | geometry_msgs/msg/Twist | ❌ (无导航任务时无发布，正常) |

### 4.4 矛盾总结

| 层级 | 状态 | 说明 |
|------|------|------|
| **PDP**（参与者发现） | ✅ 正常 | 83 个节点全部可见 |
| **SEDP**（端点元数据） | ❌ 异常 | 话题/发布者/订阅者均不可见 |
| **数据平面**（实际传输） | ✅ 正常 | 订阅者可收到真实传感器数据 |

## 5. 影响分析

1. **运维监控不可用**：`dispatch.sh status` 等运维脚本依赖 `ros2 topic info` 检测话题状态，当前全部误报为 `[MISS]`
2. **CLI 诊断工具失效**：`ros2 topic list/info/echo/hz` 等调试工具无法发现和操作话题
3. **第三方系统集成受限**：新加入的 ROS2 节点无法通过 Discovery Server 发现已有话题，必须硬编码话题名和消息类型才能通信
4. **状态上报不准确**：openTCS 调度系统通过 sidecar 连接，如果 sidecar 也受 SEDP 问题影响，可能导致话题匹配不稳定

## 6. 技术分析

### 6.1 预期行为

在 FastDDS Discovery Server 架构中：
```
节点A (CLIENT) ←→ Discovery Server ←→ 节点B (CLIENT)
        PDP: 注册参与者信息 ✅
        SEDP: 交换端点元数据（topic name, type, QoS）← 期望正常
```

Discovery Server 应同时中继 PDP 和 SEDP 信息。当一个 CLIENT 注册其端点时，Server 应将该端点的元数据（话题名、消息类型、QoS profile）广播给所有匹配的其他 CLIENT。

### 6.2 当前实际行为

```
节点A (CLIENT) ←→ Discovery Server ←→ 节点B (CLIENT)
        PDP: 注册参与者信息 ✅  ← 正常
        SEDP: 端点元数据交换   ❌  ← 丢失
        数据平面: 直接匹配传输  ✅  ← 偶然正常（本地同机）
```

SEDP 元数据未被正确中继，导致：
- `ros2 topic list`（依赖 SEDP）看不到话题
- `ros2 topic info`（依赖 SEDP）返回 "Unknown topic"
- 但当手动创建匹配的 subscriber 端点时，FastDDS 底层仍可通过其他机制（如共享内存/本地传输）完成数据匹配

### 6.3 可能的原因

1. **Discovery Server 未转发 SEDP 数据**：Server 只转发了 PDP，未正确转发 Participant 的 Endpoint 描述信息
2. **Server-Client ID 不匹配**：Discovery Server 的 server-id 配置与客户端期望的 ID 不一致，导致 SEDP 回退到直接发现但失败
3. **SEDP 超时配置不合理**：Server 的 SEDP 转发有超时或缓冲机制，在大规模端点场景下丢失了部分元数据
4. **Client 配置为非 SUPERCLIENT 模式**：非 SUPERCLIENT 模式的 CLIENT 只能看到与自己直接匹配的端点，而不是所有端点

## 7. 优化需求

### 需求 1：SEDP 端点元数据正确中继

Discovery Server 应正确中继所有已注册 CLIENT 的 SEDP 信息，使任意 CLIENT 能通过 Discovery Server 发现所有话题（topic name、message type、QoS profile）。

**验收标准**：
```bash
# 连接 Discovery Server 后
ros2 topic list | grep gazebo_1
# 应输出所有 /gazebo_1/* 话题（scan, odom, imu, map, cmd_vel 等）
```

### 需求 2：支持 SUPERCLIENT 模式

建议 Discovery Server 支持或默认开启 SUPERCLIENT 模式，使监控/诊断工具（如 `ros2` CLI）能作为 SUPERCLIENT 连接，一次性发现所有端点的完整信息，而非仅发现与自己匹配的端点。

**验收标准**：
```bash
ros2 topic info /gazebo_1/scan
# 应输出:
#   Publisher count: 1
#   Subscription count: N
```

### 需求 3：SEDP 发现健康度监控

建议 Discovery Server 提供 SEDP 发现的健康状态指标，例如：
- 已注册的 PDP 参与者数量
- 已发现的 SEDP 端点数量
- SEDP 转发成功率/丢失率

方便运维侧监控 Discovery Server 的实际工作状态。

## 8. 附录：诊断命令参考

```bash
# 设置环境
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=10.0.0.187:11811

# 测试 PDP（参与者发现）
ROS_DISABLE_DAEMON=1 ros2 node list | wc -l

# 测试 SEDP（端点发现）
ROS_DISABLE_DAEMON=1 ros2 topic list | wc -l

# 测试数据平面（直接订阅）
ros2 topic echo /gazebo_1/scan --once

# 测试 Discovery Server 连通性
ping -c 1 10.0.0.187
nc -uvz -w 3 10.0.0.187 11811
```
