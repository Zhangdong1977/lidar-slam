# RPLIDAR S2L + ROS2 SLAM 实施计划（实际环境）

## 环境概要

| 项目 | 详情 |
|------|------|
| 操作系统 | Ubuntu 24.04 LTS (Noble), x86_64 |
| 内核 | 6.17.0-23-generic |
| ROS2 | Jazzy (`/opt/ros/jazzy`) |
| 激光雷达 | RPLIDAR S2L (USB 串口, 18m 测距) |
| SLAM | slam_toolbox 2.8.4 (`ros-jazzy-slam-toolbox`, apt 安装) |
| 雷达驱动 | rplidar_ros 2.1.4 (ros2 分支，预编译 workspace) |
| 地图保存 | nav2_map_server 1.3.11 (`ros-jazzy-nav2-map-server`, apt 安装) |
| 可视化 | rviz2 14.1.20 (`ros-jazzy-rviz2`, apt 安装) |

## 系统架构

```
RPLIDAR S2L (USB Serial /dev/ttyUSB*)
    │
    ▼
rplidar_ros 节点 (预编译, third-party/rplidar_ws/)
    │ 发布 /scan (sensor_msgs/LaserScan), frame_id=laser
    ▼
static_transform_publisher (odom → laser 恒等变换)
    │
    ▼
slam_toolbox (online_async) (apt 安装)
    │ 发布 /map (nav_msgs/OccupancyGrid) + TF (map→odom)
    ▼
rviz2 可视化

TF 树 (实际硬件模式，无里程计):
  map → odom → laser
  ▲         ▲
  slam_   static_tf
  toolbox (恒等变换)

注意: 无里程计源，base_frame 设为 laser，odom→laser 为恒等变换，
      slam_toolbox 依赖纯 scan-matching 推算位姿。
```

## 项目结构（实际）

```
lidar-slam/
├── config/
│   ├── slam_toolbox_real.yaml    # 真实硬件 SLAM 参数
│   ├── slam_toolbox_sim.yaml     # 仿真 SLAM 参数
│   └── slam.rviz                 # RViz2 显示配置
├── launch/
│   ├── rplidar_s2l.launch.py     # RPLIDAR S2L 独立驱动 + rviz
│   ├── real_slam.launch.py       # 真实硬件 SLAM 全流程
│   └── sim_slam.launch.py        # Gazebo 仿真 SLAM 全流程
├── scripts/
│   ├── real_slam.sh              # 一键启动真实 SLAM (自动检测串口)
│   ├── real_save_map.sh          # 保存真实建图结果
│   ├── rplidar_s2_view.sh        # 仅查看雷达原始数据
│   ├── sim_slam.sh               # 启动仿真 SLAM
│   ├── sim_check.sh              # 仿真环境检查
│   ├── sim_teleop.sh / .py       # 键盘遥控
│   └── sim_save_map.sh           # 保存仿真地图
├── maps/                         # 保存建图结果
├── models/sim_robot/             # Gazebo 机器人模型
├── worlds/slam_test.sdf          # Gazebo 测试世界
├── third-party/
│   ├── rplidar_sdk/              # [git submodule] Slamtec RPLIDAR C++ SDK (参考)
│   ├── slam_toolbox/             # [git submodule] slam_toolbox v2.9.0 (参考)
│   ├── rplidar_ros-dev-ros2/     # rplidar_ros 源码 (v2.1.4, ros2 分支)
│   ├── rplidar_ws/               # 预编译 colcon workspace (含 rplidar_ros)
│   └── src/                      # Yahboomcar 及其他 ROS2 包
└── docs/
    ├── implementation-plan.md    # 本文件
    └── simulation-plan.md        # 仿真计划
```

---

## 实施步骤

### 阶段一：系统环境安装

#### 1.1 安装 ROS2 Jazzy

如果系统尚未安装 ROS2 Jazzy：

```bash
# 设置 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 apt 源
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Jazzy 桌面版
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

#### 1.2 安装编译依赖

```bash
sudo apt update
sudo apt install -y \
    libusb-1.0-0-dev \
    libceres-dev \
    liblapack-dev \
    libsuitesparse-dev \
    libomp-dev \
    libtbb-dev \
    qtbase5-dev
```

#### 1.3 安装 SLAM 和地图相关 ROS2 包

```bash
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server
```

#### 1.4 用户权限配置

将当前用户加入 `dialout` 组以访问串口设备：

```bash
sudo usermod -aG dialout $USER
# 注销并重新登录后生效
```

> 即使未加入 dialout 组，`real_slam.sh` 脚本会自动检测并以 `sudo chmod 666` 修复权限。

#### 1.5 验证环境

```bash
source /opt/ros/jazzy/setup.bash

# 验证 slam_toolbox
ros2 pkg list | grep slam_toolbox
# 预期输出: slam_toolbox

# 验证 rplidar_ros (预编译 workspace)
source /home/pi/lidar-slam/third-party/rplidar_ws/install/setup.bash
ros2 pkg list | grep rplidar_ros
# 预期输出: rplidar_ros

# 验证 nav2_map_server
ros2 pkg list | grep nav2_map_server
# 预期输出: nav2_map_server
```

### 阶段二：编译 rplidar_ros 雷达节点

> 项目已包含预编译的 workspace (`third-party/rplidar_ws/`)，通常无需重新编译。
> 如需从源码重新编译，按以下步骤操作。

#### 2.1 编译（如需）

```bash
# 创建 workspace
mkdir -p ~/rplidar_ws/src
cd ~/rplidar_ws/src

# 复制源码（或 git clone）
cp -r /home/pi/lidar-slam/third-party/rplidar_ros-dev-ros2 .

# 编译
cd ~/rplidar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 验证
source install/setup.bash
ros2 pkg list | grep rplidar_ros
```

编译完成后将 workspace 放到 `third-party/rplidar_ws/`，或将脚本中的 source 路径指向新 workspace。

#### 2.2 配置 udev 规则（可选）

安装 udev 规则可创建 `/dev/rplidar` 软链接，使串口设备名固定：

```bash
cd /home/pi/lidar-slam/third-party/rplidar_ros-dev-ros2/scripts
sudo bash create_udev_rules.sh
```

### 阶段三：配置文件说明

配置文件已预置在 `config/` 目录，无需手动创建。

#### 3.1 SLAM 参数 — config/slam_toolbox_real.yaml

关键配置项：

| 参数 | 值 | 说明 |
|------|-----|------|
| `base_frame` | `laser` | 无里程计时直接用激光帧作为 base |
| `use_sim_time` | `false` | 真实硬件不使用仿真时间 |
| `max_laser_range` | `18.0` | S2L 最大测距 18m |
| `min_laser_range` | `0.15` | S2L 盲区 0.05m, 留余量 |
| `resolution` | `0.05` | 5cm 栅格分辨率 |
| `solver_plugin` | `CeresSolver` | Ceres 非线性优化求解器 |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | 稀疏 Cholesky 分解 |

#### 3.2 RPLIDAR 驱动参数 — launch/rplidar_s2l.launch.py

| 参数 | 值 | 说明 |
|------|-----|------|
| `serial_baudrate` | `1000000` | S2L 专用波特率 (1M) |
| `scan_mode` | `DenseBoost` | S2L 推荐扫描模式 |
| `frame_id` | `laser` | 激光帧 ID |

> rplidar_ros 无 S2L 专用 launch 文件，S2 的 launch 文件 (`rplidar_s2_launch.py`) 完全兼容 S2L
> （相同的波特率和通信协议）。

### 阶段四：Launch 文件说明

#### 4.1 雷达独立启动 — launch/rplidar_s2l.launch.py

仅启动 rplidar_node + rviz2，用于调试和查看原始扫描数据。

```bash
ros2 launch /home/pi/lidar-slam/launch/rplidar_s2l.launch.py serial_port:=/dev/ttyUSB0
```

或使用脚本：

```bash
./scripts/rplidar_s2_view.sh          # 带 GUI
./scripts/rplidar_s2_view.sh --no-gui # 不带 GUI
```

#### 4.2 SLAM 全流程启动 — launch/real_slam.launch.py

依次启动：
1. `rplidar_s2l.launch.py`（带 `gui:=false`）— RPLIDAR 驱动节点
2. `static_transform_publisher` — 发布 `odom → laser` 恒等变换
3. `slam_toolbox online_async` — 使用 `slam_toolbox_real.yaml` 参数
4. `rviz2` — 使用 `config/slam.rviz` 配置（可通过 `rviz:=false` 禁用）

### 阶段五：运行 SLAM 建图

#### 5.1 连接雷达

将 RPLIDAR S2L 通过 USB 连接到电脑，确认设备识别：

```bash
ls /dev/ttyUSB*
# 预期输出: /dev/ttyUSB0 或 /dev/ttyUSB1 等
```

#### 5.2 一键启动建图

```bash
cd /home/pi/lidar-slam

# 启动 SLAM 建图（带 RViz 可视化）
./scripts/real_slam.sh

# 或不启动 GUI（如通过 SSH 运行）
./scripts/real_slam.sh --no-gui
```

`real_slam.sh` 脚本会自动执行以下操作：
1. 扫描 `/dev/ttyUSB*` 自动检测串口设备
2. 检查串口写入权限，无权限时自动 `sudo chmod 666` 修复
3. 加载 ROS2 Jazzy 环境和预编译的 rplidar_ws workspace
4. 启动 `real_slam.launch.py` 并传入检测到的串口号

如果是在桌面环境中运行（非 SSH），脚本会自动设置 `DISPLAY=:0` 和 Xauthority。

#### 5.3 通过 ROS2 命令启动（手动方式）

如不想使用脚本，也可手动启动：

```bash
# 加载环境
source /opt/ros/jazzy/setup.bash
source /home/pi/lidar-slam/third-party/rplidar_ws/install/setup.bash

# 启动 SLAM
ros2 launch /home/pi/lidar-slam/launch/real_slam.launch.py \
    serial_port:=/dev/ttyUSB0
```

#### 5.4 验证建图运行

```bash
# 在另一个终端中检查话题
source /opt/ros/jazzy/setup.bash
source /home/pi/lidar-slam/third-party/rplidar_ws/install/setup.bash

# 检查雷达数据
ros2 topic echo /scan --once

# 检查地图数据
ros2 topic echo /map --once

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### 阶段六：保存地图

建图完成后，在另一个终端运行：

```bash
# 使用脚本保存（默认名称 real_map）
./scripts/real_save_map.sh

# 或指定地图名称
./scripts/real_save_map.sh my_office_map

# 或手动保存
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/pi/lidar-slam/maps/my_map
```

会在 `maps/` 目录下生成：
- `my_map.pgm` — 栅格图像
- `my_map.yaml` — 地图元数据（分辨率、原点、阈值等）

---

## 常见问题排查

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| `未找到串口设备 /dev/ttyUSB*` | 雷达未连接或未识别 | 检查 USB 连接，运行 `dmesg \| grep ttyUSB` 查看内核日志 |
| `Permission denied: /dev/ttyUSB0` | 用户无串口权限 | 脚本会自动修复；或手动 `sudo chmod 666 /dev/ttyUSB0`；或加入 dialout 组后重新登录 |
| `RPLIDAR internal error detected` | S2L 固件偶发问题 | 拔插雷达 USB 重试 |
| `/scan` 无数据 | 波特率错误 | 确认 S2L 使用 `1000000`，非 `115200` |
| slam_toolbox 不发布 /map | `use_sim_time` 为 true | 确认使用 `slam_toolbox_real.yaml`（`use_sim_time: false`） |
| TF 树断裂 | 缺少 odom→laser 变换 | 确认 `real_slam.launch.py` 中 static_tf 节点正常运行 |
| 地图漂移严重 | 无里程计 + 环境特征少 | 减慢移动速度；检查雷达安装稳固性；调整 `minimum_travel_distance` |
| RViz 无法启动 (SSH) | 无 DISPLAY 环境变量 | 使用 `--no-gui` 参数，或配置 X11 转发 |

## 已知限制

1. **无里程计**: 本方案仅使用激光雷达，slam_toolbox 通过 scan-matching 推算位姿。在快速移动或长廊等几何特征单一的环境中精度会下降。
2. **固定扫描频率**: RPLIDAR S2L 固定 10Hz 扫描频率。
3. **无导航功能**: 若需 Nav2 导航，需补充里程计源（轮式编码器 / IMU）。

## 参考资源

- [rplidar_ros (ros2 分支)](https://github.com/Slamtec/rplidar_ros)
- [slam_toolbox 文档 (ROS2 Jazzy)](https://docs.ros.org/en/jazzy/p/slam_toolbox/)
- [RPLIDAR S2L 数据手册 (PDF)](https://wiki.slamtec.com/download/attachments/83066883/SLAMTEC_rplidar_datasheet_S2L_v1.2_en.pdf)
- [slam_toolbox 配置参数详解](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
