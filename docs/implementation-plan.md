# RPLIDAR S2L + ROS2 SLAM 实施计划

## 环境概要

| 项目 | 详情 |
|------|------|
| 操作系统 | Ubuntu 24.04 LTS (Noble), x86_64 |
| 内核 | 6.17.0-22-generic |
| ROS2 | Jazzy (`/opt/ros/jazzy`) |
| 激光雷达 | RPLIDAR S2L (USB 串口, 18m 测距) |
| SLAM | slam_toolbox 2.8.4 (apt) |
| 雷达驱动 | rplidar_ros (ros2 分支，源码编译) |

## 系统架构

```
RPLIDAR S2L (USB Serial /dev/ttyUSB0)
    │
    ▼
rplidar_ros 节点
    │ 发布 /scan (sensor_msgs/LaserScan)
    ▼
slam_toolbox (online_async)
    │ 发布 /map (nav_msgs/OccupancyGrid) + TF (map→odom)
    ▼
rviz2 可视化

TF 树:
  map → odom → base_link → laser_frame
         ▲              ▲
    slam_toolbox    static_transform
```

## 项目结构

```
lidar-slam/
├── src/
│   └── rplidar_ros/            # git submodule, ros2 分支
├── third-party/
│   ├── rplidar_sdk/            # git submodule, 参考
│   └── slam_toolbox/           # git submodule, 备用
├── config/
│   ├── rplidar_s2l.yaml        # RPLIDAR S2L 驱动参数
│   └── slam_toolbox_online.yaml # slam_toolbox 在线建图参数
├── launch/
│   └── slam_online.launch.py   # 一键启动 (rplidar + slam_toolbox + rviz)
├── maps/                       # 保存建图结果
├── docs/
│   └── implementation-plan.md  # 本文件
└── .gitmodules
```

---

## 实施步骤

### 阶段一：环境准备

#### 1.1 用户权限配置

将当前用户加入 `dialout` 组以访问串口设备：

```bash
sudo usermod -aG dialout $USER
# 注销并重新登录后生效
```

#### 1.2 验证 ROS2 环境

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep slam_toolbox   # 确认已安装
```

#### 1.3 安装编译依赖

```bash
sudo apt update
sudo apt install -y \
    libusb-1.0-0-dev \
    libceres-dev \
    liblapack-dev \
    suitesparse \
    libomp-dev \
    libtbb-dev \
    qtbase5-dev
```

### 阶段二：添加 rplidar_ros 子模块

> 注意：apt 包 `ros-jazzy-rplidar-ros` 缺少 S2/S2L 专用 launch 文件，必须从源码编译。rplidar_ros 无 S2L 专用 launch 文件，但 S2 的 launch 文件完全兼容 S2L（相同的波特率和通信协议）。

```bash
cd /home/pi/lidar-slam
git submodule add -b ros2 \
    https://github.com/Slamtec/rplidar_ros.git \
    src/rplidar_ros
```

执行 rplidar_ros 的 udev 规则脚本（创建 `/dev/rplidar` 软链接）：

```bash
cd src/rplidar_ros/scripts
sudo bash create_udev_rules.sh
```

### 阶段三：创建配置文件

#### 3.1 RPLIDAR S2L 驱动参数 — config/rplidar_s2l.yaml

> rplidar_ros 无 S2L 专用 launch 文件，S2 的 launch 文件（`rplidar_s2_launch.py`）兼容 S2L。
> S2L 规格：最大测距 18m，采样率 32000 samples/sec，角分辨率 0.12°，10Hz 扫描频率。

```yaml
rplidar_node:
  ros__parameters:
    channel_type: serial
    serial_port: /dev/ttyUSB0
    serial_baudrate: 1000000    # S2L 专用波特率 (1M)
    frame_id: laser_frame
    inverted: false
    angle_compensate: true
    scan_mode: DenseBoost        # S2L 推荐扫描模式
    scan_frequency: 10.0         # 10 Hz
```

#### 3.2 slam_toolbox 在线建图参数 — config/slam_toolbox_online.yaml

```yaml
slam_toolbox:
  ros__parameters:
    # 基础帧设置
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    # 时间与仿真
    use_sim_time: false
    debug_logging: false

    # 扫描处理
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05              # 5cm 栅格分辨率
    min_laser_range: 0.15         # S2L 盲区 0.05m, 留余量
    max_laser_range: 16.0         # S2L 最大 18m, 取 16m 保稳定
    minimum_time_interval: 0.1    # S2L 10Hz, 处理每帧
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # 扫描匹配
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3  # 降低以适应高密度扫描
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0

    # 回环检测
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # 相关性搜索
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # 回环搜索空间
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Ceres 求解器
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
```

### 阶段四：创建 Launch 文件

#### 4.1 一键启动 — launch/slam_online.launch.py

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 配置文件路径
    slam_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config', 'slam_toolbox_online.yaml'
    )

    # 1. RPLIDAR S2L 驱动节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
            'scan_frequency': 10.0,
        }],
    )

    # 2. base_link → laser_frame 静态 TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0.15',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser_frame'],
    )

    # 3. slam_toolbox online_async
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )

    # 4. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        rplidar_node,
        static_tf,
        slam_toolbox,
        rviz2,
    ])
```

### 阶段五：编译与运行

#### 5.1 编译 workspace

```bash
cd /home/pi/lidar-slam
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

#### 5.2 连接雷达并测试

```bash
# 确认设备已识别
ls /dev/ttyUSB0
# 或 (若已配置 udev 规则)
ls /dev/rplidar

# 检查雷达健康状态
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=1000000
# 在另一个终端查看:
ros2 topic echo /scan --once
```

#### 5.3 启动 SLAM 在线建图

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/lidar-slam/install/setup.bash
ros2 launch /home/pi/lidar-slam/launch/slam_online.launch.py
```

#### 5.4 RViz2 配置

在 RViz2 中添加以下显示项：
- **Map**: Topic `/map`, Type `nav_msgs/OccupancyGrid`
- **LaserScan**: Topic `/scan`, Type `sensor_msgs/LaserScan`
- **TF**: 显示坐标系关系

将 Fixed Frame 设为 `map`。

### 阶段六：保存地图

建图完成后，保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/pi/lidar-slam/maps/my_map
```

会生成两个文件：
- `my_map.pgm` — 栅格图像
- `my_map.yaml` — 地图元数据

---

## 常见问题排查

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| `Permission denied: /dev/ttyUSB0` | 用户无串口权限 | `sudo usermod -aG dialout $USER` 后重新登录 |
| `RPLIDAR internal error detected` | S2L 固件偶发问题 | 拔插雷达电源重试 |
| `/scan` 无数据 | 波特率错误 | 确认 S2L 使用 `1000000`，非 `115200` |
| slam_toolbox 不发布 map | `use_sim_time` 为 true | launch 中设 `use_sim_time:=false` |
| TF 树断裂 | 缺少静态变换 | 确认 `base_link → laser_frame` 已发布 |
| 地图漂移严重 | 扫描匹配参数不当 | 调小 `minimum_travel_distance`，检查雷达安装稳固性 |

## 已知限制

1. 本方案仅使用激光雷达，无里程计（odom），slam_toolbox 会通过 scan-matching 推算里程计。在快速移动或长廊等几何特征单一的环境中精度可能下降。
2. RPLIDAR S2L 固定 10Hz 扫描频率，不支持更高频率。
3. 若未来需要导航功能（Nav2），需要补充里程计源（轮式编码器 / IMU）。

## 参考资源

- [rplidar_ros (ros2 分支)](https://github.com/Slamtec/rplidar_ros)
- [slam_toolbox 文档 (ROS2 Jazzy)](https://docs.ros.org/en/jazzy/p/slam_toolbox/)
- [RPLIDAR S2L 数据手册 (PDF)](https://wiki.slamtec.com/download/attachments/83066883/SLAMTEC_rplidar_datasheet_S2L_v1.2_en.pdf)
- [slam_toolbox 配置参数详解](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
