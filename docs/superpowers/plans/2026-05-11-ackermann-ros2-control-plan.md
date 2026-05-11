# Ackermann ros2_control Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Gazebo built-in AckermannSteering plugin with ros2_control + VehicleController layered architecture.

**Architecture:** Four-layer control stack: teleop → VehicleController (Ackermann math) → ros2_control ForwardCommandController → gz_ros2_control/GazeboSimSystem. New ROS2 C++ package `ackermann_control`, Xacro model based on reference project, updated keyboard teleop publishing Float64 pair instead of Twist.

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic, ros2_control, gz_ros2_control, slam_toolbox, C++17, Python 3

---

### Task 1: Install system dependencies

**Files:** None

- [ ] **Step 1: Install ros2_control and related packages**

```bash
sudo apt install -y \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-forward-command-controller \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-state-publisher
```

- [ ] **Step 2: Verify packages are installed**

```bash
ros2 pkg list | grep -E "gz_ros2_control|forward_command_controller|joint_state_broadcaster|robot_state_publisher"
```

Expected: all five package names appear in output.

- [ ] **Step 3: Add colcon build outputs to .gitignore**

Read `.gitignore` and add these lines if not present:
```
build/
install/
log/
```

- [ ] **Step 4: Commit**

```bash
git add .gitignore
git commit -m "chore: add colcon build outputs to .gitignore"
```

---

### Task 2: Create ROS2 package scaffolding

**Files:**
- Create: `src/ackermann_control/package.xml`
- Create: `src/ackermann_control/CMakeLists.txt`
- Create: `src/ackermann_control/include/ackermann_control/visibility_control.h`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/ackermann_control/include/ackermann_control
mkdir -p src/ackermann_control/src
mkdir -p src/ackermann_control/config
mkdir -p src/ackermann_control/launch
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ackermann_control</name>
  <version>0.1.0</version>
  <description>Ackermann steering vehicle controller using ros2_control</description>
  <maintainer email="zhangdong1977@example.com">Zhang Dong</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>forward_command_controller</depend>
  <depend>gz_ros2_control</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_broadcaster</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(ackermann_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_include_directories(vehicle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

install(TARGETS vehicle_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 4: Write visibility_control.h**

```cpp
#ifndef ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_
#define ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
  #ifdef ACKERMANN_CONTROL_BUILDING_DLL
    #define ACKERMANN_CONTROL_PUBLIC __declspec(dllexport)
  #else
    #define ACKERMANN_CONTROL_PUBLIC __declspec(dllimport)
  #endif
#else
  #define ACKERMANN_CONTROL_PUBLIC
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_
```

- [ ] **Step 5: Commit**

```bash
git add src/ackermann_control/package.xml \
        src/ackermann_control/CMakeLists.txt \
        src/ackermann_control/include/ackermann_control/visibility_control.h
git commit -m "feat: add ackermann_control ROS2 package scaffolding"
```

---

### Task 3: Create configuration files

**Files:**
- Create: `src/ackermann_control/config/gz_ros2_control.yaml`
- Create: `src/ackermann_control/config/ackermann_params.yaml`

- [ ] **Step 1: Write gz_ros2_control.yaml**

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
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

forward_velocity_controller:
  ros__parameters:
    joints:
      - rear_left_wheel_joint
      - rear_right_wheel_joint
    interface_name: velocity
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity
```

- [ ] **Step 2: Write ackermann_params.yaml**

```yaml
vehicle_controller:
  ros__parameters:
    body_length: 0.9
    body_width: 0.6
    wheel_radius: 0.16
    wheel_width: 0.08
    max_steering_angle: 0.5236
    max_velocity: 1.4
```

- [ ] **Step 3: Commit**

```bash
git add src/ackermann_control/config/gz_ros2_control.yaml \
        src/ackermann_control/config/ackermann_params.yaml
git commit -m "feat: add ros2_control and vehicle parameter configs"
```

---

### Task 4: Create Xacro robot model

**Files:**
- Create: `models/ackermann/ackermann.xacro`
- Modify: `models/ackermann/model.config`

- [ ] **Step 1: Write ackermann.xacro**

Based on the reference `third-party/gazebo_ackermann_steering_vehicle/model/vehicle.xacro` with: BX-S40 parameters, LiDAR added, camera removed, IMU retained.

```xml
<?xml version="1.0"?>
<robot name="ackermann_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- BX-S40 Parameters -->
  <xacro:property name="body_length" value="0.9"/>
  <xacro:property name="body_width" value="0.6"/>
  <xacro:property name="body_height" value="0.4"/>
  <xacro:property name="body_density" value="350.0"/>
  <xacro:property name="wheel_radius" value="0.16"/>
  <xacro:property name="wheel_width" value="0.08"/>
  <xacro:property name="wheel_density" value="900.0"/>
  <xacro:property name="max_steering_angle" value="0.5236"/>
  <xacro:property name="max_steering_angular_velocity" value="1.570796"/>
  <xacro:property name="max_steering_effort" value="50.0"/>
  <xacro:property name="max_velocity" value="1.4"/>
  <xacro:property name="max_effort" value="50.0"/>
  <xacro:property name="PI" value="3.14159265"/>

  <!-- Derived properties -->
  <xacro:property name="body_mass" value="${body_density * body_length * body_height * body_width}"/>
  <xacro:property name="body_inertia_x" value="${1.0/12.0 * body_mass * (body_height*body_height + body_width*body_width)}"/>
  <xacro:property name="body_inertia_y" value="${1.0/12.0 * body_mass * (body_length*body_length + body_height*body_height)}"/>
  <xacro:property name="body_inertia_z" value="${1.0/12.0 * body_mass * (body_length*body_length + body_width*body_width)}"/>
  <xacro:property name="wheel_separation" value="${body_width + wheel_width}"/>
  <xacro:property name="wheel_offset" value="${body_length/2 - wheel_radius}"/>
  <xacro:property name="wheel_mass" value="${wheel_density * PI * wheel_radius * wheel_radius * wheel_width}"/>
  <xacro:property name="wheel_inertia_x" value="${1.0/12.0 * wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"/>
  <xacro:property name="wheel_inertia_y" value="${1.0/12.0 * wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"/>
  <xacro:property name="wheel_inertia_z" value="${1.0/2.0 * wheel_mass * wheel_radius * wheel_radius}"/>
  <xacro:property name="max_wheel_angular_velocity" value="${max_velocity / wheel_radius}"/>

  <!-- ========== Links ========== -->

  <!-- Body Link -->
  <link name="body_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="body_material">
        <color rgba="0.25 0.25 0.25 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${body_mass}"/>
      <inertia ixx="${body_inertia_x}" ixy="0.0" ixz="0.0" iyy="${body_inertia_y}" iyz="0.0" izz="${body_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Front Left Steering Link -->
  <link name="front_left_steering_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
      <material name="steering_material">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Front Left Wheel Link -->
  <link name="front_left_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0.0" izz="${wheel_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Front Right Steering Link -->
  <link name="front_right_steering_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
      <material name="steering_material">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Front Right Wheel Link -->
  <link name="front_right_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0.0" izz="${wheel_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Rear Left Wheel Link -->
  <link name="rear_left_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0.0" izz="${wheel_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Rear Right Wheel Link -->
  <link name="rear_right_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0.0" izz="${wheel_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- ========== Joints ========== -->

  <!-- Front Left Steering Joint (position controlled, Z-axis) -->
  <joint name="front_left_steering_joint" type="revolute">
    <origin xyz="${wheel_offset} ${wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="front_left_steering_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="${max_steering_effort}" lower="${-max_steering_angle}" upper="${max_steering_angle}" velocity="${max_steering_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Left Wheel Joint (passive, Y-axis) -->
  <joint name="front_left_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="front_left_steering_link"/>
    <child link="front_left_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Right Steering Joint (position controlled, Z-axis) -->
  <joint name="front_right_steering_joint" type="revolute">
    <origin xyz="${wheel_offset} ${-wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="front_right_steering_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="${max_steering_effort}" lower="${-max_steering_angle}" upper="${max_steering_angle}" velocity="${max_steering_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Right Wheel Joint (passive, Y-axis) -->
  <joint name="front_right_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="front_right_steering_link"/>
    <child link="front_right_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Rear Left Wheel Joint (velocity controlled, Y-axis) -->
  <joint name="rear_left_wheel_joint" type="continuous">
    <origin xyz="-${wheel_offset} ${wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="rear_left_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Rear Right Wheel Joint (velocity controlled, Y-axis) -->
  <joint name="rear_right_wheel_joint" type="continuous">
    <origin xyz="-${wheel_offset} ${-wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="rear_right_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- ========== Gazebo Surface Friction ========== -->
  <gazebo reference="body_link">
    <mu1>0.0001</mu1>
    <mu2>0.0001</mu2>
  </gazebo>
  <gazebo reference="front_left_wheel_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  <gazebo reference="front_right_wheel_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  <gazebo reference="rear_left_wheel_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  <gazebo reference="rear_right_wheel_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

  <!-- ========== ros2_control ========== -->
  <ros2_control name="control" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <joint name="front_left_steering_joint">
      <command_interface name="position">
        <param name="min">${-max_steering_angle}</param>
        <param name="max">${max_steering_angle}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="front_right_steering_joint">
      <command_interface name="position">
        <param name="min">${-max_steering_angle}</param>
        <param name="max">${max_steering_angle}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">${-max_wheel_angular_velocity}</param>
        <param name="max">${max_wheel_angular_velocity}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">${-max_wheel_angular_velocity}</param>
        <param name="max">${max_wheel_angular_velocity}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <!-- ========== Gazebo Plugins ========== -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>front_left_steering_joint</joint_name>
      <joint_name>front_right_steering_joint</joint_name>
      <joint_name>front_left_wheel_joint</joint_name>
      <joint_name>front_right_wheel_joint</joint_name>
      <joint_name>rear_left_wheel_joint</joint_name>
      <joint_name>rear_right_wheel_joint</joint_name>
    </plugin>

    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find ackermann_control)/config/gz_ros2_control.yaml</parameters>
    </plugin>

    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    </plugin>

    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>body_link</robot_base_frame>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>

  <!-- ========== Sensors ========== -->

  <!-- 2D LiDAR on body_link -->
  <gazebo reference="body_link">
    <sensor name="lidar" type="gpu_lidar">
      <pose>0 0 0.22 0 0 0</pose>
      <topic>/scan</topic>
      <update_rate>10</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>3600</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28318</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.15</min>
          <max>16.0</max>
        </range>
      </lidar>
      <visualize>true</visualize>
    </sensor>

    <!-- IMU on body_link -->
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <topic>/imu</topic>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.00017</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.00017</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.00017</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

</robot>
```

- [ ] **Step 2: Update model.config**

Read the existing `models/ackermann/model.config`. Replace its content:

```xml
<?xml version="1.0"?>
<model>
  <name>ackermann_robot</name>
  <version>1.0</version>
  <sdf version="1.9">ackermann.xacro</sdf>
  <description>BX-S40 Ackermann steering robot</description>
</model>
```

- [ ] **Step 3: Commit**

```bash
git add models/ackermann/ackermann.xacro models/ackermann/model.config
git commit -m "feat: add Xacro robot model for BX-S40 with ros2_control"
```

---

### Task 5: Write VehicleController C++ header

**Files:**
- Create: `src/ackermann_control/include/ackermann_control/vehicle_controller.hpp`

- [ ] **Step 1: Write vehicle_controller.hpp**

```cpp
#ifndef ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
#define ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_

#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ackermann_control
{

class VehicleController : public rclcpp::Node
{
public:
  explicit VehicleController(
    const double timer_period = 0.01,
    const double timeout_duration = 8e8  // 800ms in nanoseconds
  );

private:
  std::pair<double, double> ackermann_steering_angle();
  std::pair<double, double> rear_differential_velocity();

  void timer_callback();
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double timeout_duration_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  double steering_angle_;
  double velocity_;

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ackermann_control

#endif  // ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
```

- [ ] **Step 2: Commit**

```bash
git add src/ackermann_control/include/ackermann_control/vehicle_controller.hpp
git commit -m "feat: add VehicleController C++ header"
```

---

### Task 6: Write VehicleController C++ implementation

**Files:**
- Create: `src/ackermann_control/src/vehicle_controller.cpp`

- [ ] **Step 1: Write vehicle_controller.cpp**

```cpp
#include "ackermann_control/vehicle_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace ackermann_control
{

VehicleController::VehicleController(
  const double timer_period,
  const double timeout_duration)
: Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  track_width_ = body_width_ + (2 * wheel_width_ / 2);
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10,
    std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));

  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  if (std::abs(steering_angle_) > 1e-3) {
    const double sin_angle = std::sin(std::abs(steering_angle_));
    const double cos_angle = std::cos(std::abs(steering_angle_));

    if (steering_angle_ > 0.0) {
      left_wheel_angle = std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
      right_wheel_angle = std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      left_wheel_angle = -std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
      right_wheel_angle = -std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  if (std::abs(steering_angle_) > 1e-3) {
    const double turning_radius = wheel_base_ / std::tan(std::abs(steering_angle_));
    const double vehicle_angular_velocity = velocity_ / turning_radius;
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0) {
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    } else {
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    const double max_wheel_velocity = std::max(
      std::abs(left_wheel_velocity), std::abs(right_wheel_velocity));
    if (max_wheel_velocity > max_velocity_) {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed{(current_time - last_velocity_time_).nanoseconds()};
  const auto steering_elapsed{(current_time - last_steering_time_).nanoseconds()};

  if (velocity_elapsed > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }
  if (steering_elapsed > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  last_steering_time_ = get_clock()->now();

  if (msg->data > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (msg->data < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  } else {
    steering_angle_ = msg->data;
  }

  const auto wheel_angles{ackermann_steering_angle()};
  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  last_velocity_time_ = get_clock()->now();

  if (msg->data > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->data < -max_velocity_) {
    velocity_ = -max_velocity_;
  } else {
    velocity_ = msg->data;
  }

  const auto wheel_velocity{rear_differential_velocity()};
  wheel_angular_velocity_ = {
    wheel_velocity.first / wheel_radius_,
    wheel_velocity.second / wheel_radius_};
}

}  // namespace ackermann_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ackermann_control::VehicleController>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/ackermann_control/src/vehicle_controller.cpp
git commit -m "feat: add VehicleController C++ implementation"
```

---

### Task 7: Build and verify compilation

**Files:** None (build step)

- [ ] **Step 1: Build the colcon workspace**

```bash
cd /home/pi/lidar-slam && colcon build --packages-select ackermann_control
```

Expected: Build succeeds with no errors.

- [ ] **Step 2: Source the workspace and verify executable exists**

```bash
source /home/pi/lidar-slam/install/setup.bash && ros2 pkg list | grep ackermann_control
```

Expected: `ackermann_control` appears in the output.

- [ ] **Step 3: Run C++ linter check (optional, informational)**

```bash
cd /home/pi/lidar-slam && colcon test --packages-select ackermann_control
```

If linting fails, fix reported issues.

---

### Task 8: Rewrite keyboard teleop script

**Files:**
- Create: `scripts/ackermann_keyboard_teleop.py`

- [ ] **Step 1: Write ackermann_keyboard_teleop.py**

```python
#!/usr/bin/env python3
"""Ackermann keyboard teleop publishing /steering_angle and /velocity (Float64).

Uses select-based non-blocking keyboard reads.
"""

import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import Float64


MSG = """
Ackermann Keyboard Teleop
---------------------------
  u    i    o
  j    k    l
  m    ,    .

i / ,  : forward / backward
j / l  : steer left / right
u / o  : forward+left / forward+right
m / .  : backward+left / backward+right
k / SPACE : brake

q/z : increase/decrease both speed & steer 10%
w/x : increase/decrease speed 10%
e/c : increase/decrease steer 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
    'k': (0, 0),
    ' ': (0, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def status_str(speed, steer):
    return f'speed: {speed:.2f} m/s  steer: {steer:.2f} rad ({steer * 57.3:.0f} deg)'


def main():
    rclpy.init()
    node = rclpy.create_node('ackermann_teleop')

    speed = node.declare_parameter('speed', 0.5).value
    steer = node.declare_parameter('steer', 0.3).value

    pub_steer = node.create_publisher(Float64, '/steering_angle', 10)
    pub_vel = node.create_publisher(Float64, '/velocity', 10)

    lin = 0.0
    ang = 0.0
    status = 0
    stale_count = 0
    stale_limit = 8

    print(MSG)
    print(status_str(speed, steer), flush=True)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
                if key in MOVE_BINDINGS:
                    lin = MOVE_BINDINGS[key][0]
                    ang = MOVE_BINDINGS[key][1]
                    stale_count = 0
                elif key in SPEED_BINDINGS:
                    speed *= SPEED_BINDINGS[key][0]
                    steer *= SPEED_BINDINGS[key][1]
                    sys.stdout.write(status_str(speed, steer) + '\r\n')
                    if status == 14:
                        sys.stdout.write(MSG.replace('\n', '\r\n') + '\r\n')
                    status = (status + 1) % 15
                    stale_count = 0
                elif key == '\x03':
                    break
                else:
                    lin = 0.0
                    ang = 0.0
            else:
                stale_count += 1
                if stale_count >= stale_limit:
                    lin = 0.0
                    ang = 0.0

            msg_s = Float64()
            msg_s.data = ang * steer
            msg_v = Float64()
            msg_v.data = lin * speed
            pub_steer.publish(msg_s)
            pub_vel.publish(msg_v)
            rclpy.spin_once(node, timeout_sec=0.001)

    except Exception as e:
        sys.stdout.write(str(e) + '\r\n')
    finally:
        msg_s = Float64()
        msg_s.data = 0.0
        msg_v = Float64()
        msg_v.data = 0.0
        pub_steer.publish(msg_s)
        pub_vel.publish(msg_v)
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Make it executable**

```bash
chmod +x /home/pi/lidar-slam/scripts/ackermann_keyboard_teleop.py
```

- [ ] **Step 3: Commit**

```bash
git add scripts/ackermann_keyboard_teleop.py
git commit -m "feat: rewrite keyboard teleop for Float64 steering/velocity"
```

---

### Task 9: Create ackermann_control launch file

**Files:**
- Create: `src/ackermann_control/launch/ackermann_control.launch.py`

- [ ] **Step 1: Write ackermann_control.launch.py**

This launch file loads the robot description, starts robot_state_publisher and ros2_control, then after a delay spawns the robot in Gazebo and loads controllers.

```python
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('ackermann_control')
    xacro_file = os.path.join(
        '/home/pi/lidar-slam', 'models', 'ackermann', 'ackermann.xacro')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # 1. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 2. ros2_control node (controller_manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'gz_ros2_control.yaml']),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # 3. Spawn robot in Gazebo (one-shot, delayed until Gazebo is ready)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.17'],
        output='screen',
    )

    # 4. Load and activate controllers (one-shot commands)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen',
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_velocity_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        # Spawn robot after Gazebo + ros2_control are up (5s delay)
        TimerAction(period=5.0, actions=[spawn_robot]),
        # Load controllers after spawn (6s delay)
        TimerAction(period=6.0, actions=[
            load_joint_state_broadcaster,
            load_position_controller,
            load_velocity_controller,
        ]),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/ackermann_control/launch/ackermann_control.launch.py
git commit -m "feat: add ackermann_control launch file"
```

---

### Task 10: Rewrite sim_ackermann.launch.py

**Files:**
- Modify: `launch/sim_ackermann.launch.py`
- Modify: `config/slam_toolbox_ackermann.yaml`

- [ ] **Step 1: Read and note current content**

Read `launch/sim_ackermann.launch.py` and `scripts/sim_ackermann_slam.sh` to understand current world path and bridge configuration.

- [ ] **Step 2: Rewrite launch/sim_ackermann.launch.py**

```python
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = '/home/pi/lidar-slam'
    world_file = os.path.join(project_dir, 'worlds', 'ackermann_test.sdf')
    slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_ackermann.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models'),
    )

    # 1. Gazebo Harmonic with world (no robot in world file)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. ros_gz_bridge: sensors only (no /cmd_vel)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{
            'qos_overrides./tf.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # 3. ackermann_control (robot_state_publisher + controller_manager + spawn)
    ackermann_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ackermann_control'), '/launch/ackermann_control.launch.py'
        ]),
    )

    # 4. slam_toolbox online_async
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # 5. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,
        # Delay ackermann_control until Gazebo is running
        TimerAction(
            period=2.0,
            actions=[ackermann_control],
        ),
        slam_toolbox,
        rviz2,
    ])
```

- [ ] **Step 3: Update config/slam_toolbox_ackermann.yaml**

Read the current file, then update the frame parameters:

```bash
# Ensure these key parameters are set:
# odom_frame: odom
# base_frame: body_link
# scan_topic: /scan
# use_sim_time: true
```

Use `sed` or the Edit tool to change:
- `base_frame: chassis` → `base_frame: body_link`

- [ ] **Step 4: Commit**

```bash
git add launch/sim_ackermann.launch.py config/slam_toolbox_ackermann.yaml
git commit -m "feat: rewrite sim launch for ros2_control architecture"
```

---

### Task 11: Update world file and static TF

**Files:**
- Modify: `worlds/ackermann_test.sdf`
- Delete static_tf publisher from launch (no longer needed)

- [ ] **Step 1: Remove robot include from world file**

In `worlds/ackermann_test.sdf`, delete lines 255-260 (the `<include>` block for ackermann_robot).

- [ ] **Step 2: Verify world file is valid**

```bash
gz sdf --check /home/pi/lidar-slam/worlds/ackermann_test.sdf
```

Expected: No errors.

- [ ] **Step 3: Commit**

```bash
git add worlds/ackermann_test.sdf
git commit -m "refactor: remove robot include from world (now spawned via ros_gz_sim create)"
```

---

### Task 12: Delete deprecated files

**Files:**
- Delete: `models/ackermann/ackermann.sdf`
- Delete/Rename: `scripts/sim_ackermann_teleop.py`

- [ ] **Step 1: Delete old SDF model**

```bash
rm /home/pi/lidar-slam/models/ackermann/ackermann.sdf
```

- [ ] **Step 2: Delete old teleop**

```bash
rm /home/pi/lidar-slam/scripts/sim_ackermann_teleop.py
```

- [ ] **Step 3: Update any shell scripts referencing old files**

Search for references to deleted files:

```bash
grep -r "sim_ackermann_teleop" /home/pi/lidar-slam/scripts/ 2>/dev/null || true
grep -r "ackermann\.sdf" /home/pi/lidar-slam/ 2>/dev/null | grep -v '.git/' || true
```

If `scripts/sim_ackermann_slam.sh` or `scripts/sim_ackermann_teleop.sh` reference the old teleop, update them to use `ackermann_keyboard_teleop.py`.

- [ ] **Step 4: Commit**

```bash
git rm models/ackermann/ackermann.sdf scripts/sim_ackermann_teleop.py
# If shell scripts were updated:
git add scripts/sim_ackermann_slam.sh scripts/sim_ackermann_teleop.sh
git commit -m "refactor: remove deprecated SDF model and old teleop script"
```

---

### Task 13: Integration test in simulation

**Files:** None

- [ ] **Step 1: Full colcon build**

```bash
cd /home/pi/lidar-slam && colcon build
```

Expected: All packages build successfully.

- [ ] **Step 2: Source workspace**

```bash
source /home/pi/lidar-slam/install/setup.bash
```

- [ ] **Step 3: Launch simulation**

```bash
ros2 launch /home/pi/lidar-slam/launch/sim_ackermann.launch.py
```

Expected results to verify:
- Gazebo opens with the ackermann_test world (walls + obstacles visible)
- The robot spawns at (0, 0) in Gazebo
- `ros2 topic list` shows: `/steering_angle`, `/velocity`, `/joint_states`, `/scan`, `/imu`, `/odom`, `/tf`, `/clock`
- In another terminal, run `ackermann_keyboard_teleop.py` and verify:
  - Pressing `i` moves robot forward
  - Pressing `j`/`l` steers left/right
  - Pressing `k` stops the robot
  - `/steering_angle` and `/velocity` topics receive values
- `ros2 topic echo /joint_states` shows changing joint positions/velocities when moving
- SLAM toolbox starts and RViz2 shows laser scans and map building

- [ ] **Step 4: Test timeout safety**

Stop the teleop node while robot is moving. Verify robot stops within ~1s (the 800ms timeout).

- [ ] **Step 5: Fix any issues discovered**

If frame_id mismatches cause SLAM to not work, check:
- `ros2 topic echo /scan --once` to see the frame_id
- If needed, add a `gz.frame_id` remap in the bridge configuration or adjust SLAM config

---

## Task Dependency Graph

```
Task 1 (deps)
   │
Task 2 (package scaffold)
   │
Task 3 (config files)
   │
   ├── Task 4 (xacro model)
   ├── Task 5 (C++ header)
   │       │
   │     Task 6 (C++ impl)
   │       │
   │     Task 7 (build)
   │       │
   ├── Task 8 (teleop)
   │
Task 9 (control launch)
   │
Task 10 (sim launch)
   │
Task 11 (world update)
   │
Task 12 (cleanup)
   │
Task 13 (integration test)
```
