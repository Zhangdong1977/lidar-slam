"""Hardware layer: Standalone RPLIDAR S2L (no chassis).

For walking-around SLAM mapping with laptop + RPLIDAR S2L.
No IMU, no wheel odometry — uses rf2o laser odometry instead of EKF.

Starts:
  - rplidar_node (RPLIDAR S2L driver)
  - static TF: base_link → laser

Launch arguments:
  serial_port    (default: /dev/ttyUSB0)
  use_sim_time   (default: False)
  use_respawn    (default: True)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')

    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')

    # 1. RPLIDAR S2L driver
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': 1000000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 2. Static TF: base_link → laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial port of RPLIDAR S2L'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='False'),
        DeclareLaunchArgument(
            'use_respawn', default_value='True'),

        # Nodes
        rplidar,
        static_tf,
    ])
