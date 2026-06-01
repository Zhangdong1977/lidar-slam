"""Unified sim/real navigation launch with RS-485 chassis protocol bridge.

Data flow (simulation):
  Nav2 /cmd_vel → cmd_vel_bridge → /steering_angle, /velocity
    → rs485_chassis_bridge → serial (/tmp/chassis_cmd)
    → [socat virtual serial pair]
    → rs485_chassis_receiver → /rs485/steering_angle, /rs485/velocity
    → vehicle_controller (remapped) → ros2_control → Gazebo

Data flow (real hardware):
  Nav2 /cmd_vel → cmd_vel_bridge → /steering_angle, /velocity
    → rs485_chassis_bridge → serial (/dev/ttyUSB1) → MCU

Lifecycle architecture:
  lifecycle_starter_localization  → map_server, amcl (custom starter with timeout+retry)
  lifecycle_manager_navigation    → controller_server, planner_server, ... (10 Nav2 C++ nodes)
  lifecycle_starter_custom        → cmd_vel_bridge, rs485_bridge, rs485_receiver,
                                    opentcs_vehicle_node, route_graph_loader,
                                    vehicle_controller (custom starter, two-pass)

Launch arguments:
  simulation       (default: True)   — True=sim, False=real hardware
  use_sim_time     (default: True)   — propagated to all nodes
  use_respawn      (default: True)   — respawn crashed nodes
  vehicle_name     (default: ackermann_robot)
  namespace        (default: '')
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
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
    nav2_params = os.path.join(project_dir, 'config', 'nav2_params_opentcs.yaml')
    map_file = os.path.join(project_dir, 'maps', 'auto_exploration_map.yaml')
    rviz_config = os.path.join(project_dir, 'config', 'nav.rviz')
    ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')
    rs485_config = os.path.join(project_dir, 'config', 'rs485_bridge.yaml')
    opentcs_vehicle_config = os.path.join(project_dir, 'config', 'opentcs_vehicle.yaml')
    material_action_config = os.path.join(project_dir, 'config', 'material_action.yaml')
    watchdog_config = os.path.join(project_dir, 'config', 'watchdog.yaml')

    xacro_file = os.path.join(project_dir, 'models', 'ackermann', 'ackermann.xacro')

    pkg_share = FindPackageShare('ackermann_control')
    controller_yaml = PathJoinSubstitution([pkg_share, 'config', 'gz_ros2_control.yaml'])
    params_yaml = PathJoinSubstitution([pkg_share, 'config', 'ackermann_params.yaml'])

    # --- Launch arguments ---
    sim_arg = DeclareLaunchArgument(
        'simulation', default_value='True',
        description='True=simulation (Gazebo+socat), False=real hardware')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation clock')
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Enable automatic respawn of crashed nodes')
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='ackermann_robot',
        description='Vehicle name for openTCS identification')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS2 namespace for multi-vehicle support')

    sim = LaunchConfiguration('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    vehicle_namespace = LaunchConfiguration('namespace')

    # --- Gazebo environment ---
    gz_model_paths = ':'.join([
        os.path.join(project_dir, 'models'),
        os.path.join(project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
    ])
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', gz_model_paths,
    )

    # =========================================================================
    # Group A: Infrastructure (start immediately)
    # =========================================================================

    # A1. socat: virtual serial port pair (sim only)
    socat = ExecuteProcess(
        cmd=[
            'socat',
            'pty,link=/tmp/chassis_cmd,raw,echo=0',
            'pty,link=/tmp/chassis_recv,raw,echo=0',
        ],
        output='screen',
        condition=IfCondition(sim),
    )

    # A2. Gazebo Harmonic (sim only, on_exit_shutdown triggers full shutdown)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(sim),
    )

    # A3. ros_gz_bridge: sensors (sim only)
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
        remappings=[('/scan_raw', '/scan')],
        output='screen',
        condition=IfCondition(sim),
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # A4. Robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # A5. robot_state_publisher
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

    # A6. Static TF: body_link → laser frame
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

    # A7. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
    )

    # A8. Watchdog (start immediately, monitors everything)
    watchdog = Node(
        package='lidar_slam_nodes',
        executable='node_watchdog',
        name='node_watchdog',
        output='screen',
        parameters=[watchdog_config, {'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=5.0,
    )

    # =========================================================================
    # Group B: Robot spawn + EKF (after /scan ready)
    # =========================================================================

    # Wait for /scan topic to be ready (from Gazebo bridge or real LiDAR)
    wait_scan = Node(
        package='lidar_slam_nodes',
        executable='wait_for_topic',
        output='screen',
        parameters=[{
            'topic_name': '/scan',
            'min_publishers': 1,
            'timeout': 120.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # B1. Spawn robot in Gazebo (sim only, after /scan is available)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.24'],
        output='screen',
        condition=IfCondition(sim),
    )

    # B2. EKF sensor fusion (after /scan ready ensures /odom and /imu are likely up too)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # =========================================================================
    # Group C: Controllers (after controller_manager service ready, sim only)
    # =========================================================================

    # Wait for controller_manager service
    wait_cm = Node(
        package='lidar_slam_nodes',
        executable='wait_for_service',
        output='screen',
        parameters=[{
            'service_name': '/ackermann_robot/controller_manager/list_controllers',
            'service_type': 'controller_manager_msgs/srv/ListControllers',
            'timeout': 60.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(sim),
    )

    load_controllers = Node(
        package='lidar_slam_nodes',
        executable='load_controllers',
        output='screen',
        condition=IfCondition(sim),
    )

    # =========================================================================
    # Group D: RS-485 chain (after controllers active, sim only)
    # =========================================================================

    # Wait for /joint_states (signals controllers are active)
    wait_joints = Node(
        package='lidar_slam_nodes',
        executable='wait_for_topic',
        output='screen',
        parameters=[{
            'topic_name': '/joint_states',
            'min_publishers': 1,
            'timeout': 30.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(sim),
    )

    # D1. RS-485 receiver (sim only)
    rs485_receiver = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_receiver',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(sim),
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # D2. Vehicle controller — remapped to /rs485/ topics (sim only)
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
        condition=IfCondition(sim),
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # D3. RS-485 bridge (1s after receiver, starts both sim and real)
    #     In sim: writes to /tmp/chassis_cmd
    #     In real: writes to /dev/ttyUSB1 (configured via rs485_bridge.yaml)
    rs485_bridge = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_bridge',
        output='screen',
        parameters=[rs485_config, {'use_sim_time': use_sim_time}],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # =========================================================================
    # Group E: Localization (after EKF TF: odom→body_link is available)
    # =========================================================================

    # Wait for EKF TF
    wait_ekf_tf = Node(
        package='lidar_slam_nodes',
        executable='wait_for_tf',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_frame': 'odom'},
            {'source_frame': 'body_link'},
            {'timeout': 60.0},
            {'check_period': 0.5},
        ],
    )

    # E1. map_server (LifecycleNode, managed by lifecycle_starter_localization)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # E2. AMCL (LifecycleNode, managed by lifecycle_starter_localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('initialpose', ['/', LaunchConfiguration('vehicle_name'), '/initialpose']),
        ],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # E3. Lifecycle starter for localization (replaces Nav2 lifecycle_manager_localization)
    #     Provides configurable timeout + retry logic for reliable lifecycle transitions
    #     Note: monitor_period=0 to avoid nested spin_once() crash during rclpy.spin()
    lifecycle_starter_localization = Node(
        package='lidar_slam_nodes',
        executable='lifecycle_starter',
        name='lifecycle_starter_localization',
        output='screen',
        parameters=[{
            'node_names': ['map_server', 'amcl'],
            'configure_timeout': 30.0,
            'activate_timeout': 30.0,
            'max_retries': 5,
            'retry_delay': 2.0,
            'startup_delay': 2.0,
            'monitor_period': 0.0,      # Disable: nested spin_once crashes in rclpy.spin()
            'starter_name': 'localization',
        }],
    )

    # =========================================================================
    # Group F: Navigation + Custom lifecycle (after lifecycle_starter_localization/ready)
    # =========================================================================

    # Wait for lifecycle_starter_localization to signal all localization nodes are active
    # (This is faster than waiting for /amcl_pose, which requires AMCL convergence)
    wait_localization_ready = Node(
        package='lidar_slam_nodes',
        executable='wait_for_topic',
        output='screen',
        parameters=[{
            'topic_name': '/lifecycle_starter_localization/ready',
            'min_publishers': 1,
            'timeout': 120.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # F1. Nav2 navigation (custom: lifecycle_manager gets configured_params)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(project_dir, 'launch', 'navigation_custom.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': LaunchConfiguration('use_respawn'),
        }.items(),
    )

    # F2. Lifecycle starter for custom nodes (replaces Nav2 lifecycle_manager_custom)
    #     Two-pass bringup: handles nodes that start at different times
    lifecycle_starter_custom = Node(
        package='lidar_slam_nodes',
        executable='lifecycle_starter',
        name='lifecycle_starter_custom',
        output='screen',
        parameters=[{
            'node_names': [
                'cmd_vel_bridge',
                'rs485_chassis_bridge',
                'rs485_chassis_receiver',
                'opentcs_vehicle_node',
                'route_graph_loader',
                'vehicle_controller',
            ],
            'configure_timeout': 30.0,
            'activate_timeout': 30.0,
            'max_retries': 5,
            'retry_delay': 2.0,
            'startup_delay': 5.0,
            'monitor_period': 0.0,       # Disable: avoid nested spin_once crash
            'starter_name': 'custom',
        }],
    )

    # F3. cmd_vel_bridge (LifecycleNode, managed by lifecycle_manager_custom)
    cmd_vel_bridge = Node(
        package='lidar_slam_nodes',
        executable='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # F4. openTCS vehicle state node (LifecycleNode, managed by lifecycle_manager_custom)
    opentcs_vehicle = Node(
        package='lidar_slam_nodes',
        executable='opentcs_vehicle_node',
        namespace=vehicle_namespace,
        output='screen',
        parameters=[opentcs_vehicle_config, {
            'use_sim_time': use_sim_time,
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'namespace': vehicle_namespace,
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # =========================================================================
    # Group G: Application nodes (after navigation is fully up)
    # =========================================================================

    # Wait for route_server service
    wait_route = Node(
        package='lidar_slam_nodes',
        executable='wait_for_service',
        output='screen',
        parameters=[{
            'service_name': '/route_server/set_route_graph',
            'service_type': 'nav2_msgs/srv/SetRouteGraph',
            'timeout': 60.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # G1. Route graph loader (LifecycleNode, managed by lifecycle_manager_custom)
    route_graph_loader = Node(
        package='lidar_slam_nodes',
        executable='route_graph_loader',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'graph_save_path': '/tmp/route_graph.geojson',
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # G2. Material action GUI (non-critical, no respawn)
    material_action_gui = Node(
        package='jvs_agv_material_actions',
        executable='material_action_gui',
        namespace=vehicle_namespace,
        output='screen',
        parameters=[material_action_config, {
            'use_sim_time': use_sim_time,
            'vehicle_name': LaunchConfiguration('vehicle_name'),
        }],
    )

    # =========================================================================
    # Event-driven startup chains (replacing TimerAction delays)
    #
    # Critical: each wait_for_* must start ONLY after its prerequisite completes.
    # This prevents premature timeouts when dependencies aren't ready yet.
    # =========================================================================

    # Chain 1: /scan ready → spawn robot (sim) + EKF
    chain_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_scan,
            on_exit=[spawn_robot, ekf],
        )
    )

    # Chain 2: spawn_robot exits → start waiting for controller_manager (sim only)
    #          spawn_robot is a one-shot process, it exits after creating the entity
    chain_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[wait_cm],
        )
    )

    # Chain 3: controller_manager ready → load controllers (sim only)
    chain_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_cm,
            on_exit=[load_controllers],
        )
    )

    # Chain 4: load_controllers exits → start waiting for /joint_states (sim only)
    #          load_controllers is a one-shot node, it exits after activating controllers
    chain_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=load_controllers,
            on_exit=[wait_joints],
        )
    )

    # Chain 5: /joint_states ready → rs485 receiver + vehicle_controller (sim)
    #          Then 1s later → rs485 bridge (both sim and real)
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

    # For real hardware: start rs485_bridge with a short delay after EKF
    # (no controller_manager needed, MCU handles wheel control)
    rs485_bridge_real = TimerAction(
        period=3.0,
        actions=[rs485_bridge],
        condition=UnlessCondition(sim),
    )

    # Chain 6: EKF TF ready → map_server + amcl + lifecycle_starter_localization
    chain_localization = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_ekf_tf,
            on_exit=[map_server, amcl, lifecycle_starter_localization],
        )
    )

    # Chain 7: lifecycle_starter_localization ready → navigation + lifecycle_starter_custom + cmd_vel_bridge + opentcs
    chain_navigation = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_localization_ready,
            on_exit=[
                navigation,
                lifecycle_starter_custom,
                cmd_vel_bridge,
                opentcs_vehicle,
            ],
        )
    )

    # Chain 8: route_server ready → route_graph_loader
    chain_route = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_route,
            on_exit=[route_graph_loader],
        )
    )

    # Material action GUI: start after a delay (non-critical, doesn't need tight sequencing)
    material_delayed = TimerAction(
        period=5.0,
        actions=[material_action_gui],
    )

    # =========================================================================
    # Final launch description
    # =========================================================================

    return LaunchDescription([
        # Arguments
        sim_arg,
        sim_time_arg,
        respawn_arg,
        vehicle_name_arg,
        namespace_arg,

        # Group A: Infrastructure (immediate)
        set_gz_resource_path,
        socat,
        gz_sim,
        bridge,
        robot_state_publisher,
        laser_tf,
        rviz2,
        watchdog,

        # Group B: readiness gate → spawn + EKF
        wait_scan,
        chain_spawn,

        # Group C-F: chained gates (each starts only after prerequisite completes)
        #   spawn_robot → wait_cm → load_controllers → wait_joints → rs485_chain
        chain_after_spawn,
        chain_controllers,
        chain_after_controllers,
        chain_rs485_sim,
        rs485_bridge_real,

        # Group E: readiness gate → localization
        wait_ekf_tf,
        chain_localization,

        # Group F: readiness gate → navigation + custom lifecycle
        wait_localization_ready,
        chain_navigation,

        # Group G: readiness gate → application
        wait_route,
        chain_route,
        material_delayed,
    ])
