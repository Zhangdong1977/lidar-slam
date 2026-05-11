# Ackermann 底盘 ros2_control 架构迁移设计

## 目标

将当前基于 Gazebo 内置 `gz-sim-ackermann-steering-system` 插件的一体化控制方案，
替换为基于 `ros2_control` + 自定义 `VehicleController` 的分层架构。
目标是：自定义 Ackermann 运动学、学习 ros2_control、为真实硬件 RS-485 控制做准备。

## 硬件参数（BX-S40 底盘）

| 参数 | 值 | 说明 |
|------|-----|------|
| 外形尺寸 | 0.9 × 0.6 × 0.4 m | 长 × 宽 × 高 |
| 轮径 | 0.16 m | 半径（直径 32 cm） |
| 轴距 (wheel_base) | 0.59 m | 前后轮轴心距 |
| 轮距 (track_width) | 0.51 m | 左右轮中心距 |
| 自重 | 76 kg | |
| 载重 | 500 kg | |
| 最大速度 | 5 km/h (~1.4 m/s) | |
| 控制接口 | RS-485 | 协议见 docs/底盘485通信协议.md |

## 架构总览

```
键盘手柄 → [teleop] → /steering_angle (Float64)
                        /velocity       (Float64)
                              │
                    VehicleController (C++)
                      - Ackermann 几何
                      - 后轮差速
                              │
              /forward_position_controller/commands
              /forward_velocity_controller/commands
                              │
                       ros2_control
                  (ForwardCommandController)
                              │
              ┌───────────────┴───────────────┐
              │ 仿真                           │ 真实硬件 (未来)
              │ gz_ros2_control/              │ RS-485 Serial
              │ GazeboSimSystem               │ Hardware Interface
              └───────────────────────────────┘
```

## 包结构

```
/home/pi/lidar-slam/
├── src/ackermann_control/                 # 新建 ROS2 C++ 包
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/ackermann_control/
│   │   ├── vehicle_controller.hpp
│   │   └── visibility_control.h
│   ├── src/
│   │   └── vehicle_controller.cpp
│   ├── config/
│   │   ├── gz_ros2_control.yaml
│   │   └── ackermann_params.yaml
│   └── launch/
│       └── ackermann_control.launch.py
├── models/ackermann/
│   ├── ackermann.xacro                    # 新建，基于参考项目修改
│   └── model.config
├── launch/sim_ackermann.launch.py         # 重写
├── scripts/ackermann_keyboard_teleop.py   # 重写
├── worlds/ackermann_test.sdf              # 删除机器人 include
├── config/slam_toolbox_ackermann.yaml     # 更新 base_frame
└── maps/                                  # 不变
```

## 各组件设计

### 1. Xacro 模型 (models/ackermann/ackermann.xacro)

基于参考项目 vehicle.xacro，修改：

- **参数对齐 BX-S40**：body_length=0.9, body_width=0.6, body_height=0.4,
  wheel_radius=0.16, wheel_width=0.08, max_steering_angle=0.5236, max_velocity=1.4,
  mass=76
- **添加 LiDAR**：gpu_lidar 传感器，3600 采样，10Hz，范围 0.15-16m，
  安装在 chassis 顶部（z=0.22m），topic=/scan
- **删除 camera**：去掉 camera_link / camera_stick_link 及关联 joints
- **保留 IMU**：100Hz，noise 参数对齐当前值
- **关节结构**：
  - `front_left_steering_joint`：Z 轴 revolute，body → steering_link，position 控制
  - `front_left_wheel_joint`：Y 轴 continuous，steering_link → wheel_link，被动滚动
  - 右侧同理 `front_right_steering_joint` + `front_right_wheel_joint`
  - `rear_left_wheel_joint`：Y 轴 continuous，body → wheel_link，velocity 控制
  - `rear_right_wheel_joint`：Y 轴 continuous，body → wheel_link，velocity 控制

### 2. ros2_control 配置 (config/gz_ros2_control.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController
    forward_velocity_controller:
      type: forward_command_controller/ForwardCommandController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

forward_position_controller:
  ros__parameters:
    joints:
      - front_left_steering_joint
      - front_right_steering_joint
    interface_name: position
    command_interfaces: [position]
    state_interfaces: [position, velocity]

forward_velocity_controller:
  ros__parameters:
    joints:
      - rear_left_wheel_joint
      - rear_right_wheel_joint
    interface_name: velocity
    command_interfaces: [velocity]
    state_interfaces: [position, velocity]
```

### 3. VehicleController (src/ackermann_control/)

从参考项目移植，改动点：
- 参数从 `ackermann_params.yaml` 加载
- 超时时间 800ms
- 输入限幅：steering_angle → ±max_steering_angle，velocity → ±max_velocity
- Ackermann 几何：根据 wheel_base 和 track_width 计算左右前轮不同转向角
- 后轮差速：根据 turning_radius 计算内外轮不同速度，超 max_velocity 等比缩放

### 4. 键盘遥控 (scripts/ackermann_keyboard_teleop.py)

从当前 sim_ackermann_teleop.py 改造：
- 发布 `/steering_angle` (Float64) + `/velocity` (Float64) 替代 `/cmd_vel` (Twist)
- 键盘映射、参数动态调节、超时停止、select 非阻塞读取 → 全部保留
- 默认参数：speed=0.5 m/s, steer=0.3 rad

### 5. Launch 流程 (launch/sim_ackermann.launch.py)

1. SetEnvironmentVariable → GZ_SIM_RESOURCE_PATH 指向 models/
2. Gazebo Harmonic → 加载 ackermann_test.sdf world（不含机器人）
3. ros_gz_bridge → LiDAR, IMU, clock, tf
4. robot_state_publisher → 加载 ackermann.xacro，发布 robot_description
5. ros2_control controller_manager → 启动 forward_position_controller,
   forward_velocity_controller, joint_state_broadcaster
6. ros_gz_sim create → 在 Gazebo 中动态 spawn 机器人 URDF
7. slam_toolbox online_async → 加载 slam_toolbox_ackermann.yaml
8. RViz2

### 6. World 文件变更 (worlds/ackermann_test.sdf)

- 删除第 256-260 行的 `<include model="ackermann">` 块
- 其余房间墙壁障碍物保持不变

## 数据流

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/steering_angle` | Float64 | teleop → VehicleController | 期望转向角 (rad) |
| `/velocity` | Float64 | teleop → VehicleController | 期望线速度 (m/s) |
| `/forward_position_controller/commands` | Float64MultiArray | VehicleController → ros2_control | 前轮位置 |
| `/forward_velocity_controller/commands` | Float64MultiArray | VehicleController → ros2_control | 后轮速度 |
| `/joint_states` | JointState | ros2_control → SLAM/RViz | 关节状态 |
| `/scan` | LaserScan | Gazebo → ros_gz_bridge → SLAM | 激光数据 |
| `/imu` | IMU | Gazebo → ros_gz_bridge → SLAM | IMU 数据 |
| `/odom` | Odometry | OdometryPublisher → ros_gz_bridge → SLAM | 里程计 |
| `/tf` | TFMessage | OdometryPublisher → ros_gz_bridge | 坐标变换 |

## 删除文件

- `models/ackermann/ackermann.sdf`（不再使用）
- `scripts/sim_ackermann_teleop.py`（被 ackermann_keyboard_teleop.py 替代）

## 新增依赖

- `ros-jazzy-gz-ros2-control`
- `ros-jazzy-forward-command-controller`
- `ros-jazzy-ros2-controllers`
- `ros-jazzy-joint-state-broadcaster`
- `ros-jazzy-robot-state-publisher`
