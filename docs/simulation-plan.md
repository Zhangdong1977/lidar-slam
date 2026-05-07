# Gazebo Harmonic 仿真测试计划

## 目标

在连接 RPLIDAR S2L 实体硬件之前，使用 Gazebo Harmonic 仿真环境验证整个 SLAM 管线：

```
仿真 LiDAR 数据 → ros_gz_bridge → slam_toolbox 建图 → rviz2 可视化
```

## 软件栈

| 组件 | 版本/包名 | 安装状态 |
|------|-----------|----------|
| 仿真器 | Gazebo Harmonic (`gz-sim8` 8.11.0) | 已安装 |
| ROS2-Gazebo 启动 | `ros-jazzy-ros-gz-sim` 1.0.22 | 已安装 |
| 话题桥接 | `ros-jazzy-ros-gz-bridge` 1.0.22 | 已安装 |
| 键盘遥控 | `ros-jazzy-teleop-twist-keyboard` | 待安装 |
| 差速驱动 | Gazebo 内置 `DiffDrive` 插件 | 内置 |
| 2D LiDAR | Gazebo 内置 `gpu_lidar` 传感器 | 内置 |

## 仿真架构

```
Gazebo Harmonic
├── 仿真世界 (slam_test.sdf): 10m×10m 房间 + 障碍物
└── 机器人模型 (sim_robot):
    ├── 差速驱动底盘 (DiffDrive 插件)
    ├── gpu_lidar 传感器 (参数匹配 RPLIDAR S2L)
    └── 发布 odom + TF (odom→base_link)
        │
        ▼
ros_gz_bridge (话题桥接)
    /scan     → ROS2 /scan (sensor_msgs/LaserScan)
    /cmd_vel  → Gazebo DiffDrive (控制移动)
    /odom     → ROS2 /odom (nav_msgs/Odometry)
        │
        ▼
slam_toolbox (online_async, use_sim_time:=true)
    发布 /map (nav_msgs/OccupancyGrid) + TF (map→odom)
        │
        ▼
rviz2 可视化

TF 树:
  map → odom → base_link → laser_frame
        ▲         ▲
   slam_toolbox  Gazebo DiffDrive (仿真里程计)
```

> **与实体的差异：** 仿真环境通过 DiffDrive 插件提供里程计（odom→base_link），而实体环境无里程计，slam_toolbox 需通过 scan-matching 推算。切换实体时只需改 `use_sim_time` 和数据源，SLAM 配置基本不变。

## 新增文件结构

```
lidar-slam/
├── models/
│   └── sim_robot/
│       └── model.sdf                # 差速小车 + 2D LiDAR 模型
├── worlds/
│   └── slam_test.sdf                # 测试世界（房间 + 障碍物）
├── config/
│   └── slam_toolbox_sim.yaml        # 仿真 SLAM 参数 (use_sim_time: true)
└── launch/
    └── sim_slam.launch.py           # 仿真一键启动
```

---

## 实施步骤

### 步骤 1：安装依赖

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

### 步骤 2：创建目录结构

```bash
mkdir -p /home/pi/lidar-slam/models/sim_robot
mkdir -p /home/pi/lidar-slam/worlds
```

### 步骤 3：创建机器人模型 — models/sim_robot/model.sdf

一个简单的差速驱动小车，配备 2D LiDAR：

- **底盘 (chassis)**：矩形 0.3m × 0.3m × 0.15m，质量 5kg
- **左/右驱动轮**：圆柱体，半径 0.05m，宽 0.02m，间距 0.3m
- **后万向轮 (caster)**：球体，半径 0.025m
- **LiDAR 传感器**：`gpu_lidar`，参数匹配 RPLIDAR S2L
  - 水平扫描范围：360°（0° ~ 2π）
  - 采样数：3600（对应 0.1° 角分辨率，接近 S2L 的 0.12°）
  - 扫描频率：10 Hz
  - 有效范围：0.15m ~ 16.0m（与 slam_toolbox 配置一致）
  - 安装高度：底盘顶部 z = 0.15m
- **Gazebo 插件**：
  - `DiffDrive`：差速驱动，发布里程计到 `/model/sim_robot/odometry`，订阅 `/model/sim_robot/cmd_vel`
  - `Sensors`：驱动 `gpu_lidar` 传感器

### 步骤 4：创建测试世界 — worlds/slam_test.sdf

- 地面：平面 `<include> model://sun` + 地板
- 墙壁：四面 10m × 2.5m 的墙体围成封闭房间（0,0 为中心）
- 障碍物：
  - 2-3 个矩形箱子（0.5m × 0.5m × 0.5m）
  - 1-2 个圆柱体（半径 0.2m，高 0.8m）
  - 一个 L 形内墙（增加几何特征多样性，利于 scan-matching）
- 机器人初始位置：(0, 0, 0.075)

### 步骤 5：创建仿真 SLAM 配置 — config/slam_toolbox_sim.yaml

基于 `config/slam_toolbox_online.yaml`，仅修改以下参数：

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true              # 关键差异：使用仿真时间
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    # ... 其余参数与 slam_toolbox_online.yaml 相同
```

> 其余参数（扫描匹配、回环检测、Ceres 求解器等）保持与实体配置一致，确保仿真结果可迁移。

### 步骤 6：创建仿真 Launch 文件 — launch/sim_slam.launch.py

启动流程：

1. **IncludeLaunchDescription** `ros_gz_sim` 启动 Gazebo，加载 `slam_test.sdf` 世界
2. **ros_gz_bridge 节点**：桥接话题
   - `/scan`：Gazebo `sensor_msgs/LaserScan` → ROS2 `/scan`
   - `/cmd_vel`：ROS2 `/cmd_vel` → Gazebo DiffDrive
   - `/odom`：Gazebo odom → ROS2 `/odom`
   - `/tf`：Gazebo TF → ROS2 `/tf`
3. **static_transform_publisher**：`base_link → laser_frame`（z=0.15）
4. **IncludeLaunchDescription** `slam_toolbox` online_async，加载 `slam_toolbox_sim.yaml`
5. **rviz2** 节点

**使用方式：**

```bash
# 终端 1：启动仿真 SLAM
source /opt/ros/jazzy/setup.bash
source /home/pi/lidar-slam/install/setup.bash
ros2 launch /home/pi/lidar-slam/launch/sim_slam.launch.py

# 终端 2：键盘遥控
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 步骤 7：验证测试

启动仿真后，按以下检查清单逐项验证：

| # | 检查项 | 验证方法 |
|---|--------|----------|
| 1 | Gazebo 启动正常 | 看到 Gazebo 窗口，机器人模型和世界加载 |
| 2 | `/scan` 话题有数据 | `ros2 topic echo /scan --once` |
| 3 | `/odom` 话题有数据 | `ros2 topic echo /odom --once` |
| 4 | TF 树完整 | `ros2 run tf2_tools view_frames`，确认 map→odom→base_link→laser_frame |
| 5 | 键盘可控制小车 | 在 teleop 终端按方向键，Gazebo 中小车移动 |
| 6 | `/map` 话题有数据 | `ros2 topic echo /map --once` |
| 7 | rviz2 显示地图 | LaserScan 和 OccupancyGrid 正常渲染 |
| 8 | 绕房间一圈后地图完整 | 遥控小车绕房间行驶，观察地图是否完整闭合 |
| 9 | 回环检测生效 | 回到起点后地图自动修正 |

### 步骤 8：保存仿真地图

验证完成后，保存仿真生成的地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/pi/lidar-slam/maps/sim_test_map
```

---

## 仿真通过标准

- [ ] 小车能响应键盘指令移动
- [ ] `/scan` 数据格式与 RPLIDAR S2L 一致（LaserScan, 360°, ~10Hz）
- [ ] slam_toolbox 正常启动，无报错
- [ ] rviz2 中能看到实时激光扫描点和逐步构建的地图
- [ ] 绕房间一圈后，地图边界闭合无明显漂移
- [ ] 回环检测正常工作（返回起点时地图自动修正）

通过仿真验证后，继续执行原计划的实体雷达连接步骤。
