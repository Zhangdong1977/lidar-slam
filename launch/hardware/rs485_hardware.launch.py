"""RS-485 hardware layer: RPLIDAR + car_base_node (odom+IMU) + RS-485 chassis bridge.

Starts:
  - rplidar_node (RPLIDAR S2L, /dev/ttyUSB0)
  - car_base_node (里程计+IMU from STM32, /dev/ttyAMA0)  [optional, if available]
  - static TF: base_link -> laser
  - robot_state_publisher (simplified URDF)
  - rs485_chassis_bridge (writes RS-485 frames to /dev/ttyUSB1)
  - cmd_vel_bridge (Twist -> steering + velocity)

Note: In this profile, car_base_node provides odom+IMU from its STM32.
The RS-485 chassis bridge sends steering+velocity to a separate MCU.
If the RS-485 MCU also provides odom+IMU, car_base_node is not needed.

Launch arguments:
  use_sim_time     (default: False)
  use_respawn      (default: True)
  lidar_serial     (default: /dev/ttyUSB0)
  chassis_serial   (default: /dev/ttyUSB1)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    rs485_config = os.path.join(project_dir, 'config', 'rs485_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')

    # 1. RPLIDAR S2L
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial'),
            'serial_baudrate': 1000000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 2. static TF: base_link -> laser
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.22',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 3. car_base_node (provides /odom + /imu/data_raw from STM32)
    #    Override frame IDs to match our convention
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
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 4. cmd_vel_bridge: /cmd_vel -> /steering_angle + /velocity
    cmd_vel_bridge = Node(
        package='lidar_slam_nodes',
        executable='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_base': 0.58,
            'max_steering_angle': 0.5236,
            'max_velocity': 1.4,
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # 5. rs485_chassis_bridge: writes RS-485 frames to serial
    rs485_bridge = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_bridge',
        output='screen',
        parameters=[rs485_config, {
            'use_sim_time': use_sim_time,
            'serial_port': LaunchConfiguration('chassis_serial'),
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # rs485_bridge starts with a delay to let other nodes settle
    rs485_bridge_delayed = TimerAction(
        period=3.0,
        actions=[rs485_bridge],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument('lidar_serial', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('chassis_serial', default_value='/dev/ttyUSB1'),

        # Hardware nodes
        rplidar,
        laser_tf,
        car_base,
        cmd_vel_bridge,
        rs485_bridge_delayed,
    ])
