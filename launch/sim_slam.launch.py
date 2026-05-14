import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'slam_test.sdf')
    slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_sim.yaml')

    # Ensure Gazebo can find the robot model
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models'),
    )

    # 1. Launch Gazebo Harmonic with the test world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Bridge Gazebo <-> ROS2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{
            'qos_overrides./tf.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # 3. Static TF: base_link -> scan frame
    # Gazebo sets scan frame_id to "sim_robot/base_link/lidar"
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'sim_robot/base_link/lidar',
        ],
        parameters=[{'use_sim_time': True}],
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
        static_tf,
        slam_toolbox,
        rviz2,
    ])
