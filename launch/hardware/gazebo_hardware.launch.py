"""Gazebo hardware layer: simulator + RS-485 virtual loopback.

Starts:
  - socat virtual serial port pair
  - Gazebo Harmonic simulator
  - ros_gz_bridge (scan, odom, imu, tf, clock)
  - robot_state_publisher (ackermann.xacro)
  - static TF: body_link -> lidar frame
  - rs485_chassis_receiver (reads virtual serial)
  - vehicle_controller (ros2_control interface)
  - rs485_chassis_bridge (writes virtual serial, 1s delay)

Launch arguments:
  use_sim_time     (default: True)
  use_respawn      (default: True)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    rs485_config = os.path.join(project_dir, 'config', 'rs485_bridge.yaml')
    xacro_file = os.path.join(project_dir, 'models', 'ackermann', 'ackermann.xacro')

    pkg_share = FindPackageShare('ackermann_control')
    params_yaml = PathJoinSubstitution([pkg_share, 'config', 'ackermann_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')

    # Gazebo resource paths
    gz_model_paths = ':'.join([
        os.path.join(project_dir, 'models'),
        os.path.join(project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
    ])

    # A1. socat: virtual serial port pair
    socat = ExecuteProcess(
        cmd=[
            'socat',
            'pty,link=/tmp/chassis_cmd,raw,echo=0',
            'pty,link=/tmp/chassis_recv,raw,echo=0',
        ],
        output='screen',
    )

    # A2. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # A3. ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{
            'qos_overrides./tf.publisher.durability': 'transient_local',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/scan_raw', 'scan'),
            ('/odom', 'odom'),
            ('/imu', 'imu'),
            ('/tf', 'tf'),
            # /clock stays absolute (global for sim time synchronization)
        ],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # A4. Robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # A5. static TF: body_link -> lidar
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.22',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'body_link',
            '--child-frame-id', 'ackermann_robot/body_link/lidar',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Spawn robot ---
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.24'],
        output='screen',
    )

    # --- Controller loading ---
    wait_cm = Node(
        package='lidar_slam_nodes',
        executable='wait_for_service',
        output='screen',
        parameters=[{
            'service_name': 'controller_manager/list_controllers',
            'service_type': 'controller_manager_msgs/srv/ListControllers',
            'timeout': 15.0,
            'use_sim_time': use_sim_time,
        }],
    )

    load_controllers = Node(
        package='lidar_slam_nodes',
        executable='load_controllers',
        output='screen',
    )

    wait_joints = Node(
        package='lidar_slam_nodes',
        executable='wait_for_topic',
        output='screen',
        parameters=[{
            'topic_name': 'joint_states',
            'min_publishers': 1,
            'timeout': 30.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # --- RS-485 chain (sim loopback) ---
    rs485_receiver = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_receiver',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': use_sim_time}],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    vehicle_controller = Node(
        package='ackermann_control',
        executable='vehicle_controller',
        name='vehicle_controller',
        parameters=[params_yaml],
        remappings=[
            ('steering_angle', 'rs485/steering_angle'),
            ('velocity', 'rs485/velocity'),
        ],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    rs485_bridge = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_bridge',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': use_sim_time}],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # --- Event chains ---
    chain_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[wait_cm],
        )
    )
    chain_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_cm,
            on_exit=[load_controllers],
        )
    )
    chain_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=load_controllers,
            on_exit=[wait_joints],
        )
    )
    chain_rs485_sim = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_joints,
            on_exit=[
                rs485_receiver,
                vehicle_controller,
                TimerAction(period=1.0, actions=[rs485_bridge]),
            ],
        )
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument('namespace', default_value=''),

        # Infrastructure
        socat,
        gz_sim,
        bridge,
        robot_state_publisher,
        laser_tf,

        # Spawn chain
        spawn_robot,
        chain_spawn,
        chain_controllers,
        chain_after_controllers,
        chain_rs485_sim,
    ])
