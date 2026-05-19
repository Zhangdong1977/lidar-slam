import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    nav2_params = os.path.join(project_dir, 'config', 'nav2_params_ackermann.yaml')
    map_file = os.path.join(project_dir, 'maps', 'ackermann_map.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'nav.rviz')
    ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models'),
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

    # 2. ros_gz_bridge: sensors + tf
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

    # 5. EKF: fuse /odom + /imu -> publish odom->body_link TF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # 6. Nav2 localization (map_server + AMCL)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/localization_launch.py'
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    # 7. Nav2 navigation (planner, controller, etc.)
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

    # 9. openTCS-Nav2 bridge: /goal_pose -> NavigateToPose, TF -> /amcl_pose
    opentcs_bridge = Node(
        package='lidar_slam_nodes',
        executable='opentcs_nav2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_publish_rate': 10.0,
        }],
    )

    # 10. RViz2
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
        TimerAction(
            period=2.0,
            actions=[ackermann_control],
        ),
        laser_tf,
        TimerAction(
            period=5.0,
            actions=[ekf],
        ),
        TimerAction(
            period=15.0,
            actions=[localization],
        ),
        TimerAction(
            period=25.0,
            actions=[navigation],
        ),
        cmd_vel_bridge,
        # Bridge starts after Nav2 action server is ready
        TimerAction(
            period=30.0,
            actions=[opentcs_bridge],
        ),
        rviz2,
    ])
