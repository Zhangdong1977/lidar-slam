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
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{}],
        output='screen',
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

    # 6. Static TF: body_link → laser frame (Gazebo uses scoped name as frame_id)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.22',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'body_link',
            '--child-frame-id', 'ackermann_robot/body_link/lidar',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 7. RViz2
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
        rviz2,
    ])
