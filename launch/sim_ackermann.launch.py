import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_ackermann.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'slam.rviz')
    ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models') + ':' + os.path.join(project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
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

    # 4. EKF: fuse /odom + /imu → publish odom→body_link TF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # 5. slam_toolbox online_async
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # 6. Static TF: body_link → laser frame
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

    # 7. Lifecycle starter: activate vehicle_controller LifecycleNode
    lifecycle_starter = Node(
        package='lidar_slam_nodes',
        executable='lifecycle_starter',
        name='lifecycle_starter',
        parameters=[{
            'node_names': ['vehicle_controller'],
            'timeout': 5.0,
            'retries': 3,
        }],
        output='screen',
    )

    # 8. RViz2
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
            actions=[scan_filter],
        ),
        # Delay ackermann_control until Gazebo is running
        TimerAction(
            period=2.0,
            actions=[ackermann_control],
        ),
        # EKF starts after bridge delivers /odom and /imu
        TimerAction(
            period=5.0,
            actions=[ekf],
        ),
        # slam_toolbox starts after robot is spawned and TF/scan are flowing
        TimerAction(
            period=5.0,
            actions=[slam_toolbox],
        ),
        laser_tf,
        # Activate vehicle_controller after ackermann_control has started it (12s delay inside)
        TimerAction(
            period=15.0,
            actions=[lifecycle_starter],
        ),
        rviz2,
    ])
