# 真实机器人底盘控制方案

## 背景

当前项目有两套互不连通的控制路径：

- **仿真路径**：`teleop.py` → `/steering_angle` + `/velocity` → `VehicleController`(C++) → `ForwardCommandController`(ros2_control) → `gz_ros2_control` → Gazebo 物理引擎
- **真实机器人**：仅有激光雷达 SLAM（RPLIDAR S2L + rf2o + slam_toolbox），**没有任何电机控制代码**

真实底盘通过 RS-485 串口通信（协议见 [底盘485通信协议.md](底盘485通信协议.md)），CH1 控制转向、CH2 控制速度，值域 1000~2000，1500 为中位/停止。

**目标**：写一个桥接节点，订阅现有的 `/steering_angle` + `/velocity` 话题，转换为 485 帧发送给真实底盘，使同一套 teleop 脚本同时适用于仿真和真实。

---

## 方案设计

新建一个 Python 桥接节点 `chassis_bridge.py`，作为仿真中 `VehicleController` 的真实硬件替代品：

```
仿真: teleop → /steering_angle + /velocity → VehicleController → ForwardCommandController → Gazebo
真实: teleop → /steering_angle + /velocity → ChassisBridge → RS-485 串口 → 真实底盘
```

真实底盘内部已处理阿克曼几何（转向角度 + 油门），桥接节点只需做简单的线性映射。

---

## 实现步骤

### Step 1：创建 `scripts/chassis_bridge.py`（新文件）

独立 Python ROS2 节点，遵循项目现有 `scripts/` 目录惯例（同 `ackermann_keyboard_teleop.py` 一样直接运行，无需 ament 包）。

#### 核心类设计

```python
class ChassisBridge(Node):
    """订阅 /steering_angle + /velocity，通过 RS-485 串口发送 485 控制帧"""

    # 订阅
    #   /steering_angle (Float64, rad)
    #   /velocity       (Float64, m/s)
    #
    # 定时器：每 100ms 调用 send_frame()
    #   1. 检查命令超时（400ms 无新命令 → 自动归零）
    #   2. 线性映射为 CH1/CH2 值
    #   3. 构建 23 字节帧
    #   4. 写入串口
```

#### 线性映射公式

```
CH1(转向) = 1500 + (angle_rad / max_steering_angle) * 500
CH2(速度) = 1500 + (velocity_ms / max_velocity) * 500
```

| 输入 | 输出 | 含义 |
|------|------|------|
| angle = -0.5236 rad | CH1 = 1000 | 最大左转 |
| angle = 0 | CH1 = 1500 | 直行 |
| angle = +0.5236 rad | CH1 = 2000 | 最大右转 |
| velocity = -1.4 m/s | CH2 = 1000 | 最大后退 |
| velocity = 0 | CH2 = 1500 | 停止 |
| velocity = +1.4 m/s | CH2 = 2000 | 最大前进 |

#### 485 帧格式（23 字节固定）

```
[AA][55] [CH1_H][CH1_L] [CH2_H][CH2_L] [05 DC]×8 [A5]
 帧头      转向           速度          CH3~10=1500  帧尾
```

示例：前进 + 左转最大 → `AA 55 03 E8 07 D0 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC A5`

#### 安全机制

| 场景 | 处理方式 |
|------|---------|
| 节点启动 | steering=0, velocity=0 → CH1=1500, CH2=1500 → 安全静止 |
| 命令超时（400ms） | 自动发送停车帧 CH1=1500, CH2=1500 |
| Ctrl-C 关闭 | 先发停车帧，再关闭串口 |
| 底盘自身超时（500ms） | 底盘自动停车（双重保护） |
| 串口写入失败 | 记录日志但不崩溃，底盘会在 500ms 后自动停车 |

#### 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB1` | RS-485 适配器串口（ttyUSB0 给雷达用） |
| `baud_rate` | `115200` | 波特率 |
| `send_period` | `0.1` | 发帧间隔（秒） |
| `max_steering_angle` | `0.5236` | 最大转向角（rad，约 30°） |
| `max_velocity` | `1.4` | 最大速度（m/s） |
| `timeout_duration` | `0.4` | 命令超时（秒，须小于底盘 500ms 超时） |

#### 依赖

- `rclpy` — ROS2 Python 客户端
- `std_msgs` — Float64 消息类型
- `pyserial` — 串口通信（`pip install pyserial`）

---

### Step 2：修改 `launch/real_slam.launch.py`

添加底盘控制支持，通过 launch 参数控制开关：

```python
# 新增 launch 参数
DeclareLaunchArgument('chassis', default_value='true')
DeclareLaunchArgument('chassis_port', default_value='/dev/ttyUSB1')

# 底盘桥接节点（使用 ExecuteProcess，与项目中 load_controllers.py 运行方式一致）
chassis_bridge = ExecuteProcess(
    cmd=['python3', os.path.join(project_dir, 'scripts', 'chassis_bridge.py'),
         '--ros-args',
         '-p', ['serial_port:=', LaunchConfiguration('chassis_port')],
         ...],
    output='screen',
    condition=IfCondition(LaunchConfiguration('chassis')),
)
```

---

### Step 3：修改 `scripts/real_slam.sh`

扩展 shell 脚本支持底盘控制：

- 新增 `drive` 模式：SLAM + 底盘控制 + teleop（三合一）
- 双串口自动检测：ttyUSB0 给雷达，ttyUSB1 给底盘
- 添加 `--no-chassis` 参数禁用底盘控制（纯 SLAM 模式）

---

### Step 4：验证测试

1. **串口帧验证**：连接 USB-RS485 适配器，用 `ros2 topic pub` 发送测试值，串口监视器确认帧内容
   - `steering=0.3 rad` → CH1=1787
   - `velocity=0.5 m/s` → CH2=1679
2. **超时停车测试**：发送命令后停止发布，确认 400ms 后自动停车
3. **集成测试**：运行 `real_slam.sh drive` + 键盘 teleop，实际驱动小车并观察建图
4. **关机安全测试**：Ctrl-C 确认小车停车

---

## 关键文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `scripts/chassis_bridge.py` | **新建** | RS-485 桥接节点（核心交付物） |
| `launch/real_slam.launch.py` | 修改 | 添加底盘控制节点和 launch 参数 |
| `scripts/real_slam.sh` | 修改 | 添加 drive 模式和双串口支持 |
| `scripts/ackermann_keyboard_teleop.py` | 不改 | 复用，仿真和真实共用 |

---

## 待确认事项

1. **RS-485 适配器串口路径**：假设为 `/dev/ttyUSB1`（雷达占 ttyUSB0），连接适配器后需确认
2. **底盘最大转向角度和最大速度**：当前使用仿真参数（30° / 1.4 m/s），需实测校准与真实底盘是否匹配
3. **pyserial 是否已安装**：如未安装需 `pip install pyserial`
