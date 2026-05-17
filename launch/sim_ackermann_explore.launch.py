import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    nav2_params = os.path.join(project_dir, 'config', 'nav2_params_exploration.yaml')
    slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_ackermann.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'explore.rviz')
    ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models') + ':' + os.path.join(
            project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
    )

    # 1. Gazebo Harmonic with world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. ros_gz_bridge: sensors only
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{}],
        output='screen',
    )

    # 2b. scan_range_filter: replace inf/NaN → valid range so Karto traces free space
    scan_filter = Node(
        package='lidar_slam_nodes',
        executable='scan_range_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 3. ackermann_control (robot_state_publisher + controller_manager + spawn)
    ackermann_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ackermann_control'), '/launch/ackermann_control.launch.py'
        ]),
    )

    # 4. Static TF: body_link -> laser frame
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.22',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'body_link',
            '--child-frame-id', 'body_link/lidar',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 5. EKF: fuse /odom + /imu -> odom->body_link TF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # 6. slam_toolbox online_async (provides /map topic AND map->odom TF)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # 7. Nav2 navigation ONLY (NO AMCL, NO map_server)
    #    slam_toolbox provides localization (map->odom TF) and /map topic
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    # 8. cmd_vel_bridge: Nav2 Twist -> Ackermann steering_angle/velocity
    cmd_vel_bridge = Node(
        package='lidar_slam_nodes',
        executable='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 9. Wait for TF tree: body_link -> map (slam_toolbox localization)
    wait_tf = Node(
        package='lidar_slam_nodes',
        executable='wait_for_tf',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'target_frame': 'map'},
            {'source_frame': 'body_link'},
            {'timeout': 120.0},
            {'check_period': 0.5},
        ],
    )

    # 10. m-explore-ros2 frontier explorer (starts after TF is available)
    explore_params = os.path.join(project_dir, 'config', 'explore_lite_params.yaml')
    frontier_explorer = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params,
            {'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # 10b. Map-saver watcher: auto-save on exploration completion
    map_saver_watcher = Node(
        package='lidar_slam_nodes',
        executable='map_saver_watcher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 11. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,
        TimerAction(period=2.0, actions=[ackermann_control]),
        TimerAction(period=2.0, actions=[scan_filter]),
        laser_tf,
        TimerAction(period=5.0, actions=[ekf]),
        TimerAction(period=5.0, actions=[slam_toolbox]),
        TimerAction(period=25.0, actions=[navigation]),
        cmd_vel_bridge,
        rviz2,
        # Wait for TF tree, then launch explore + map_saver
        TimerAction(period=30.0, actions=[wait_tf]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_tf,
                on_exit=[
                    frontier_explorer,
                    map_saver_watcher,
                ],
            ),
        ),
    ])
