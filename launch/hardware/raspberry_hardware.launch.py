"""Raspberry Pi car hardware layer: RPLIDAR C1 + car_base_node (STM32).

Starts:
  - rplidar_node (RPLIDAR C1, /dev/lidar, 460800 baud)
  - car_base_node (底盘驱动+里程计+IMU, /dev/ttyAMA0)
  - imu_filter_madgwick_node (/imu/data_raw -> /imu/data)
  - static TF: base_link -> lidar_link
  - static TF: base_link -> imu_link
  - battery_bridge (/PowerVoltage -> /battery_state)

car_base_node directly subscribes to /cmd_vel (Twist) and sends to STM32 via UART.
STM32 internally handles Ackermann kinematics — no cmd_vel_bridge or rs485_bridge needed.

Launch arguments:
  use_sim_time     (default: False)
  use_respawn      (default: True)
  lidar_serial     (default: /dev/lidar)
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_params(yaml_path):
    """Load ros__parameters from YAML, bypassing node-name key matching.

    ROS2 Jazzy matches YAML keys against the fully qualified node name.
    A key like ``imu_filter:`` only matches ``/imu_filter`` (root namespace),
    not ``/c30_1/imu_filter`` (sub-namespace).  This helper extracts the
    ``ros__parameters`` dict regardless of the top-level key so that
    parameters work under any namespace.
    """
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    for v in doc.values():
        if isinstance(v, dict) and 'ros__parameters' in v:
            return v['ros__parameters']
    return {}


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')

    # 1. RPLIDAR C1 (YeahBot uses RPLIDAR C1 on /dev/lidar at 460800)
    #    Defensive remapping: ensure 'scan' topic resolves under namespace
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial'),
            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('/scan', 'scan')],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 2. car_base_node (底盘驱动 + 传感器)
    #    订阅 /cmd_vel → UART → STM32 → 电机
    #    发布 /odom (编码器里程计), /imu/data_raw, /joint_states, /PowerVoltage
    #    Defensive remappings: absolute topics → relative for namespace support
    car_base = Node(
        package='car_base',
        executable='car_base_node',
        name='car_base_node',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom',
            'robot_frame_id': 'base_link',
            'gyro_frame_id': 'imu_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/odom', 'odom'),
            ('/imu/data_raw', 'imu/data_raw'),
            ('/cmd_vel', 'cmd_vel'),
            ('/joint_states', 'joint_states'),
            ('/PowerVoltage', 'PowerVoltage'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 3. static TF: base_link -> lidar_link (激光雷达安装位置)
    #    TF remapping for namespace isolation
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf_static', 'tf_static')],
    )

    # 3b. static TF: base_link -> imu_link (IMU 安装位置)
    #     C30 的 IMU 集成在 STM32 板上，位于底盘中心
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.02',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf_static', 'tf_static')],
    )

    # 3c. IMU Madgwick filter: /imu/data_raw -> /imu/data
    #     Fuses accelerometer + gyroscope for stable orientation estimate
    #     Reference: vendor's car_base.launch.py uses the same node+config
    imu_config = os.path.join(project_dir, 'config', 'imu.yaml')
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[_load_params(imu_config), {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data_raw', 'imu/data_raw'),
            ('/imu/data', 'imu/data'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # 4. battery_bridge: /PowerVoltage (Float32) -> /battery_state (BatteryState)
    #    Defensive remappings for namespace support
    battery_bridge = Node(
        package='lidar_slam_nodes',
        executable='battery_bridge',
        name='battery_bridge',
        output='screen',
        parameters=[{
            'voltage_full': 24.0,
            'voltage_empty': 20.0,
            'publish_rate': 1.0,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/PowerVoltage', 'PowerVoltage'),
            ('/battery_state', 'battery_state'),
        ],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument('lidar_serial', default_value='/dev/lidar'),

        # Hardware nodes
        rplidar,
        car_base,
        laser_tf,
        imu_tf,
        imu_filter,
        battery_bridge,
    ])
