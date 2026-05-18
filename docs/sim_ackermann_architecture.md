# sim_ackermann.launch.py — 纯SLAM建图架构

## 节点与话题数据流

```mermaid
flowchart LR
  subgraph Gazebo["🔧 Gazebo Harmonic (factory.sdf + ackermann.xacro)"]
    lidar["LiDAR Sensor<br/>gpu_lidar<br/>720 samples, 10Hz"]
    imu["IMU Sensor<br/>100Hz"]
    odom_pub["Odometry Publisher<br/>publish_tf=false"]
    jsp["JointState Publisher"]
    cm["Controller Manager<br/>(gz_ros2_control)"]
  end

  subgraph Bridge["🌉 ros_gz_bridge"]
    br_scan["parameter_bridge<br/>gz→ros"]
    br_odom["parameter_bridge<br/>gz→ros"]
    br_imu["parameter_bridge<br/>gz→ros"]
    br_clock["parameter_bridge<br/>gz→ros"]
  end

  subgraph Filter["🔍 扫描过滤"]
    srf["scan_range_filter<br/>(lidar_slam_nodes)<br/>NaN→inf<br/>strip namespace"]
  end

  subgraph SLAM["🗺️ SLAM + 定位"]
    st["slam_toolbox<br/>online_async<br/>mode: mapping"]
    ekf["ekf_filter_node<br/>(robot_localization)<br/>2D mode, 100Hz"]
  end

  subgraph Robot["🤖 机器人模型"]
    rsp["robot_state_publisher<br/>(URDF/Xacro TF)"]
    vc["vehicle_controller<br/>(ackermann_control)<br/>Ackermann运动学"]
    stf["static_transform_publisher<br/>body_link→body_link/lidar<br/>(z=0.22)"]
  end

  subgraph Viz["👁️ 可视化"]
    rv["rviz2"]
  end

  lidar -->|"/scan_raw<br/>(gz.msgs.LaserScan)"| br_scan
  imu -->|"/imu<br/>(gz.msgs.IMU)"| br_imu
  odom_pub -->|"/odom<br/>(gz.msgs.Odometry)"| br_odom

  br_scan -->|"/scan_raw<br/>(sensor_msgs/LaserScan)"| srf
  br_odom -->|"/odom<br/>(nav_msgs/Odometry)"| ekf
  br_imu -->|"/imu<br/>(sensor_msgs/Imu)"| ekf
  br_clock -->|"/clock"| ekf
  br_clock -->|"/clock"| st
  br_clock -->|"/clock"| srf

  srf -->|"/scan<br/>(LaserScan, cleaned)"| st
  srf -->|"/scan"| rv

  st -->|"/map<br/>(OccupancyGrid)"| rv
  st -->|"/map_updates"| rv

  jsp -->|"/joint_states"| rsp

  vc -->|"/forward_position_controller/commands<br/>(Float64MultiArray)"| cm
  vc -->|"/forward_velocity_controller/commands<br/>(Float64MultiArray)"| cm
```

## TF 树

```mermaid
flowchart TD
  map["🗺️ map<br/>(world fixed)"]
  odom["📍 odom<br/>(drift frame)"]
  body["🚗 body_link<br/>(robot base)"]
  lidar["📡 body_link/lidar<br/>(laser frame)"]

  fls["front_left_steering_link"]
  flw["front_left_wheel_link"]
  frs["front_right_steering_link"]
  frw["front_right_wheel_link"]
  rlw["rear_left_wheel_link"]
  rrw["rear_right_wheel_link"]

  map -->|"slam_toolbox<br/>动态修正"| odom
  odom -->|"ekf_filter_node<br/>100Hz, 2D<br/>融合 /odom + /imu"| body
  body -->|"static_tf<br/>(z=0.22)"| lidar
  body -->|"robot_state_publisher<br/>(xacro固定关节)"| fls
  body -->|"robot_state_publisher"| frs
  body -->|"robot_state_publisher"| rlw
  body -->|"robot_state_publisher"| rrw
  fls -->|"xacro"| flw
  frs -->|"xacro"| frw
```

### TF 发布者汇总

| TF 变换 | 发布节点 | 频率 | 说明 |
|---------|---------|------|------|
| map → odom | slam_toolbox | 动态(50Hz) | SLAM定位修正 |
| odom → body_link | ekf_filter_node | 100Hz | 融合 /odom + /imu, 2D模式 |
| body_link → body_link/lidar | static_transform_publisher | static | 固定偏移 (x:0, y:0, z:0.22) |
| body_link → wheel/steering links | robot_state_publisher | static | 从 URDF/Xacro 读取固定关节 |

## 启动时序

```mermaid
gantt
    title sim_ackermann.launch.py 启动时序
    dateFormat X
    axisFormat %s s

    section 基础层
    Gazebo + Bridge    :a1, 0, 2000
    laser_tf           :a2, 0, 2000
    rviz2              :a3, 0, 2000

    section 机器人层
    ackermann_control  :b1, 2000, 10000

    section 感知层
    scan_range_filter   :b2, 2000, 1000

    section 定位与建图
    ekf_filter         :c1, 5000, 1000
    slam_toolbox       :c2, 5000, 1000
```

### ackermann_control 子启动时序

```mermaid
gantt
    title ackermann_control 内部启动时序
    dateFormat X
    axisFormat %s s

    section 机器人模型
    robot_state_publisher :a0, 0, 1000
    spawn_robot           :a1, 5000, 2000
    load_controllers      :a2, 10000, 3000
    vehicle_controller    :a3, 12000, 1000
```

## 完整节点列表

| 序号 | 节点名 | 包 | 可执行文件 | 启动延迟 | 说明 |
|------|--------|-----|-----------|---------|------|
| 1 | gz_sim (include) | ros_gz_sim | gz_sim.launch.py | 0s | Gazebo Harmonic 仿真世界 |
| 2 | bridge | ros_gz_bridge | parameter_bridge | 0s | Gazebo→ROS2 传感器桥接 |
| 3 | scan_range_filter | lidar_slam_nodes | scan_range_filter | 2s | NaN→inf, 去掉命名空间前缀 |
| 4 | robot_state_publisher | robot_state_publisher | robot_state_publisher | 2s(include内) | 发布机器人关节TF |
| 5 | spawn_robot | ros_gz_sim | create | 7s(include内) | 在Gazebo中生成机器人 |
| 6 | load_controllers | lidar_slam_nodes | load_controllers | 12s(include内) | 加载并激活控制器 |
| 7 | vehicle_controller | ackermann_control | vehicle_controller | 14s(include内) | Ackermann运动学转换 |
| 8 | laser_tf | tf2_ros | static_transform_publisher | 0s | body_link→lidar 静态TF |
| 9 | ekf_filter_node | robot_localization | ekf_node | 5s | 融合 /odom + /imu, 发布 odom→body_link TF |
| 10 | slam_toolbox (include) | slam_toolbox | online_async_launch.py | 5s | SLAM建图, 发布 /map 和 map→odom TF |
| 11 | rviz2 | rviz2 | rviz2 | 0s | 可视化 |

## 话题列表

| 话题 | 消息类型 | 发布者 | 订阅者 |
|------|---------|--------|--------|
| /scan_raw (gz) | gz.msgs.LaserScan | Gazebo LiDAR sensor | ros_gz_bridge |
| /scan_raw (ros) | sensor_msgs/LaserScan | ros_gz_bridge | scan_range_filter |
| /scan | sensor_msgs/LaserScan | scan_range_filter | slam_toolbox, rviz2 |
| /odom | nav_msgs/Odometry | Gazebo OdometryPublisher → ros_gz_bridge | ekf_filter_node |
| /imu | sensor_msgs/Imu | Gazebo IMU sensor → ros_gz_bridge | ekf_filter_node |
| /clock | rosgraph_msgs/Clock | ros_gz_bridge | 所有节点 |
| /joint_states | sensor_msgs/JointState | Gazebo JointStatePublisher | robot_state_publisher |
| /map | nav_msgs/OccupancyGrid | slam_toolbox | rviz2 |
| /map_updates | nav_msgs/OccupancyGrid | slam_toolbox | rviz2 |
| /steering_angle | std_msgs/Float64 | (手动/外部) | vehicle_controller |
| /velocity | std_msgs/Float64 | (手动/外部) | vehicle_controller |
| /forward_position_controller/commands | std_msgs/Float64MultiArray | vehicle_controller | Controller Manager (Gazebo) |
| /forward_velocity_controller/commands | std_msgs/Float64MultiArray | vehicle_controller | Controller Manager (Gazebo) |
