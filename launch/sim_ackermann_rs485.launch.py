"""Simulation launch with RS-485 chassis protocol bridge.

Data flow:
  Nav2 /cmd_vel → cmd_vel_bridge → /steering_angle, /velocity
    → rs485_chassis_bridge → serial (/tmp/chassis_cmd)
    → [socat virtual serial pair]
    → rs485_chassis_receiver → /rs485/steering_angle, /rs485/velocity
    → vehicle_controller (remapped) → ros2_control → Gazebo
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    nav2_params = os.path.join(project_dir, 'config', 'nav2_params_opentcs.yaml')
    map_file = os.path.join(project_dir, 'maps', 'auto_exploration_map.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'nav.rviz')
    ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')
    rs485_config = os.path.join(project_dir, 'config', 'rs485_bridge.yaml')
    opentcs_vehicle_config = os.path.join(project_dir, 'config', 'opentcs_vehicle.yaml')
    material_action_config = os.path.join(project_dir, 'config', 'material_action.yaml')

    xacro_file = os.path.join(project_dir, 'models', 'ackermann', 'ackermann.xacro')

    pkg_share = FindPackageShare('ackermann_control')
    controller_yaml = PathJoinSubstitution([pkg_share, 'config', 'gz_ros2_control.yaml'])
    params_yaml = PathJoinSubstitution([pkg_share, 'config', 'ackermann_params.yaml'])

    # --- Gazebo environment ---
    gz_model_paths = ':'.join([
        os.path.join(project_dir, 'models'),
        os.path.join(project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
    ])
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_model_paths,
    )

    # 1. socat: virtual serial port pair
    socat = ExecuteProcess(
        cmd=[
            'socat',
            'pty,link=/tmp/chassis_cmd,raw,echo=0',
            'pty,link=/tmp/chassis_recv,raw,echo=0',
        ],
        output='screen',
    )

    # 2. Gazebo Harmonic with world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 3. ros_gz_bridge: sensors
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
        }],
        remappings=[
            ('/scan_raw', '/scan'),
        ],
        output='screen',
    )

    # --- Replicate ackermann_control nodes with RS-485 remapping ---

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # 4. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 5. Spawn robot in Gazebo
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.24'],
        output='screen',
    )

    # 6. Load controllers
    load_controllers = Node(
        package='lidar_slam_nodes',
        executable='load_controllers',
        output='screen',
    )

    # 7. VehicleController — remapped to /rs485/ topics
    vehicle_controller = Node(
        package='ackermann_control',
        executable='vehicle_controller',
        name='vehicle_controller',
        parameters=[params_yaml],
        remappings=[
            ('/steering_angle', '/rs485/steering_angle'),
            ('/velocity', '/rs485/velocity'),
        ],
        output='screen',
    )

    # 8. RS-485 bridge nodes
    rs485_receiver = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_receiver',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': True}],
    )

    rs485_bridge = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_bridge',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': True}],
    )

    # --- Standard navigation nodes ---

    # 9. Static TF: body_link → laser frame
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

    # 10. EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # 11. Nav2 localization (custom: passes configured_params to lifecycle_manager)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(project_dir, 'launch', 'localization_custom.launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
            'vehicle_name': LaunchConfiguration('vehicle_name'),
        }.items(),
    )

    # 12. Nav2 navigation (custom launch: lifecycle_manager gets service_call_timeout)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(project_dir, 'launch', 'navigation_custom.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    # 13. cmd_vel_bridge
    cmd_vel_bridge = Node(
        package='lidar_slam_nodes',
        executable='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 14. Route graph loader (receives GeoJSON from Sidecar, loads into route_server)
    route_graph_loader = Node(
        package='lidar_slam_nodes',
        executable='route_graph_loader',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'graph_save_path': '/tmp/route_graph.geojson',
        }],
    )

    # 15. openTCS vehicle state node (publishes /robot_state, /battery_state, /amcl_pose)
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='ackermann_robot',
        description='Vehicle name for openTCS identification')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS2 namespace for multi-vehicle support')

    vehicle_namespace = LaunchConfiguration('namespace')
    opentcs_vehicle = Node(
        package='lidar_slam_nodes',
        executable='opentcs_vehicle_node',
        namespace=vehicle_namespace,
        output='screen',
        parameters=[opentcs_vehicle_config, {
            'use_sim_time': True,
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'namespace': vehicle_namespace,
        }],
    )

    # 15. RViz2 (was 15, now 16)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
    )

    # 17. Material action simulation GUI (JVS-VGA控制台)
    material_action_gui = Node(
        package='jvs_agv_material_actions',
        executable='material_action_gui',
        namespace=vehicle_namespace,
        output='screen',
        parameters=[material_action_config, {
            'use_sim_time': True,
            'vehicle_name': LaunchConfiguration('vehicle_name'),
        }],
    )

    return LaunchDescription([
        set_gz_resource_path,
        vehicle_name_arg,
        namespace_arg,
        # Virtual serial port pair (must start first)
        socat,
        gz_sim,
        bridge,
        # Robot model + spawn + controllers (same timing as ackermann_control.launch.py)
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=10.0, actions=[load_controllers]),
        laser_tf,
        # EKF
        TimerAction(period=5.0, actions=[ekf]),
        # RS-485 receiver + vehicle_controller (after controllers loaded)
        TimerAction(period=12.0, actions=[rs485_receiver, vehicle_controller]),
        # RS-485 bridge (1s after receiver, so it's ready to read)
        TimerAction(period=13.0, actions=[rs485_bridge]),
        # Nav2 (delayed: Gazebo needs time to stabilize under heavy CPU load)
        TimerAction(period=25.0, actions=[localization]),
        TimerAction(period=30.0, actions=[navigation]),
        # UI / teleop
        cmd_vel_bridge,
        # openTCS vehicle state node (after Nav2 action server is ready)
        TimerAction(period=30.0, actions=[opentcs_vehicle]),
        # Route graph loader (after route_server is up)
        TimerAction(period=35.0, actions=[route_graph_loader]),
        # Material action GUI (after Nav2 + opentcs_vehicle ready)
        TimerAction(period=32.0, actions=[material_action_gui]),
        rviz2,
    ])
