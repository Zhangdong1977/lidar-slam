import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    project_dir = '/home/pi/Desktop/code/lidar-slam'
    slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_real.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'slam.rviz')

    # 1. RPLIDAR S2 驱动
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(project_dir, 'launch', 'rplidar_s2l.launch.py')),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'gui': 'false',
        }.items(),
    )

    # 2. rf2o 激光里程计: odom -> base_link
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'laser_scan_topic': '/scan',
            'init_pose_from_topic': '',
        }],
        output='screen',
    )

    # 3. 静态 TF: base_link -> laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    # 4. slam_toolbox online_async
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )

    # 5. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial port of RPLIDAR'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Whether to launch rviz2'),
        rplidar,
        rf2o,
        static_tf,
        slam_toolbox,
        rviz2,
    ])
