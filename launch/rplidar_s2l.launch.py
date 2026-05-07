import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rplidar_dir = get_package_share_directory('rplidar_ros')
    rviz_config = os.path.join(rplidar_dir, 'rviz', 'rplidar_ros.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial port of RPLIDAR S2L'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='1000000',
            description='Baudrate for RPLIDAR S2L'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Whether to launch rviz2'),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
