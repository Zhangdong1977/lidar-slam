"""Unified navigation launch supporting three hardware profiles.

Hardware profiles:
  gazebo    — Gazebo full simulation (chassis + sensors simulated)
  rs485     — RS-485 physical chassis + RPLIDAR S2L + car_base_node (odom+IMU)
  raspberry — Raspberry Pi car (car_base_node drives STM32, RPLIDAR C1)

Usage:
  ros2 launch nav_main.launch.py hardware_profile:=gazebo
  ros2 launch nav_main.launch.py hardware_profile:=rs485
  ros2 launch nav_main.launch.py hardware_profile:=raspberry map_file:=/path/map.yaml

The hardware profile determines:
  - Which hardware sub-launch is included (sensors + chassis drivers)
  - EKF frame names (body_link vs base_link)
  - IMU topic name (/imu vs /imu/data_raw)
  - Whether cmd_vel_bridge + rs485_bridge are needed
  - lifecycle_starter_custom node list

Architecture:
  [Hardware Layer]     → /scan, /odom, /imu, TF: odom→base
  [EKF Fusion]         → TF: odom→base (refined)
  [Localization]       → TF: map→odom (AMCL or slam_toolbox)
  [Nav2 Navigation]    → /cmd_vel
  [Application Layer]  → opentcs, explore, route, watchdog
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def load_profile_yaml(profile_name_str, project_dir):
    """Load profile config yaml as dict."""
    import yaml
    path = os.path.join(project_dir, 'config', 'profiles', f'{profile_name_str}.yaml')
    if os.path.exists(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    return {}


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')

    # --- Launch arguments ---
    profile_arg = DeclareLaunchArgument(
        'hardware_profile', default_value='gazebo',
        description='Hardware profile: gazebo | rs485 | raspberry')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(project_dir, 'maps', 'auto_exploration_map.yaml'),
        description='Map YAML file for AMCL localization')
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Enable automatic respawn of crashed nodes')
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='ackermann_robot',
        description='Vehicle name for openTCS identification')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS2 namespace for multi-vehicle support')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(project_dir, 'config', 'nav.rviz'),
        description='RViz config file')

    # Use OpaqueFunction to access launch configuration at runtime
    def launch_setup(context):
        profile = LaunchConfiguration('hardware_profile').perform(context)
        use_respawn = LaunchConfiguration('use_respawn')
        vehicle_namespace = LaunchConfiguration('namespace')

        # Load profile config
        profile_cfg = load_profile_yaml(profile, project_dir)
        use_sim_time = profile_cfg.get('use_sim_time', profile == 'gazebo')
        base_frame = profile_cfg.get('frames', {}).get('base_frame', 'body_link')
        odom_frame = profile_cfg.get('frames', {}).get('odom_frame', 'odom')
        imu_topic = profile_cfg.get('sensors', {}).get('imu', {}).get('topic', '/imu')
        chassis_type = profile_cfg.get('chassis', {}).get('type', 'gazebo')

        # Determine if cmd_vel_bridge + rs485_bridge are needed
        needs_cmd_vel_bridge = chassis_type in ('gazebo', 'rs485')
        needs_rs485_bridge = chassis_type in ('gazebo', 'rs485')

        # Config file paths
        nav2_params = os.path.join(project_dir, 'config', 'nav2_params_opentcs.yaml')
        ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')
        opentcs_vehicle_config = os.path.join(project_dir, 'config', 'opentcs_vehicle.yaml')
        watchdog_config = os.path.join(project_dir, 'config', 'watchdog.yaml')
        material_action_config = os.path.join(project_dir, 'config', 'material_action.yaml')
        rviz_config = LaunchConfiguration('rviz_config').perform(context)
        map_file = LaunchConfiguration('map_file').perform(context)

        # --- Build EKF parameters with profile overrides ---
        ekf_params = [ekf_config, {
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'imu0': imu_topic,
        }]

        # --- Determine lifecycle_starter_custom node list ---
        custom_lifecycle_nodes = ['opentcs_vehicle_node', 'route_graph_loader']
        if needs_cmd_vel_bridge:
            custom_lifecycle_nodes.insert(0, 'cmd_vel_bridge')
        if needs_rs485_bridge:
            custom_lifecycle_nodes.insert(
                1 if needs_cmd_vel_bridge else 0, 'rs485_chassis_bridge')
        if chassis_type == 'gazebo':
            custom_lifecycle_nodes.extend([
                'rs485_chassis_receiver', 'vehicle_controller'])

        actions = []

        # =====================================================================
        # Group A: Hardware layer (profile-specific)
        # =====================================================================

        hardware_dir = os.path.join(project_dir, 'launch', 'hardware')

        is_gazebo = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'gazebo'"])
        is_rs485 = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'rs485'"])
        is_raspberry = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'raspberry'"])

        hardware_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'gazebo_hardware.launch.py')),
            condition=IfCondition(is_gazebo),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
            }.items(),
        )

        hardware_rs485 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'rs485_hardware.launch.py')),
            condition=IfCondition(is_rs485),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
            }.items(),
        )

        hardware_raspberry = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'raspberry_hardware.launch.py')),
            condition=IfCondition(is_raspberry),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
            }.items(),
        )

        actions.extend([hardware_gazebo, hardware_rs485, hardware_raspberry])

        # =====================================================================
        # Group A (common): RViz2 + Watchdog
        # =====================================================================

        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
        )

        watchdog = Node(
            package='lidar_slam_nodes',
            executable='node_watchdog',
            name='node_watchdog',
            output='screen',
            parameters=[watchdog_config, {'use_sim_time': use_sim_time}],
            respawn=True,
            respawn_delay=5.0,
        )

        actions.extend([rviz2, watchdog])

        # =====================================================================
        # Group B: EKF (after /scan ready)
        # =====================================================================

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

        ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=ekf_params,
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        # =====================================================================
        # Group E: Localization (after EKF TF ready)
        # =====================================================================

        wait_ekf_tf = Node(
            package='lidar_slam_nodes',
            executable='wait_for_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': odom_frame,
                'source_frame': base_frame,
                'timeout': 60.0,
                'check_period': 0.5,
            }],
        )

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

        amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('initialpose', ['/', LaunchConfiguration('vehicle_name'), '/initialpose']),
            ],
            respawn=use_respawn,
            respawn_delay=2.0,
        )

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
                'monitor_period': 0.0,
                'starter_name': 'localization',
            }],
        )

        # =====================================================================
        # Group F: Navigation + Custom lifecycle (after localization ready)
        # =====================================================================

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

        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(project_dir, 'launch', 'navigation_custom.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'autostart': 'True',
                'params_file': nav2_params,
                'use_composition': 'False',
                'use_respawn': 'True',
            }.items(),
        )

        lifecycle_starter_custom = Node(
            package='lidar_slam_nodes',
            executable='lifecycle_starter',
            name='lifecycle_starter_custom',
            output='screen',
            parameters=[{
                'node_names': custom_lifecycle_nodes,
                'configure_timeout': 30.0,
                'activate_timeout': 30.0,
                'max_retries': 5,
                'retry_delay': 2.0,
                'startup_delay': 5.0,
                'monitor_period': 0.0,
                'starter_name': 'custom',
            }],
        )

        # cmd_vel_bridge (only for gazebo and rs485 profiles)
        cmd_vel_bridge_node = Node(
            package='lidar_slam_nodes',
            executable='cmd_vel_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('hardware_profile'), "' != 'raspberry'"
                ])),
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        opentcs_vehicle = Node(
            package='lidar_slam_nodes',
            executable='opentcs_vehicle_node',
            namespace=vehicle_namespace,
            output='screen',
            parameters=[opentcs_vehicle_config, {
                'use_sim_time': use_sim_time,
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'namespace': vehicle_namespace,
                'base_frame': base_frame,
            }],
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        # =====================================================================
        # Group G: Application (after route_server ready)
        # =====================================================================

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

        # =====================================================================
        # Event-driven startup chains
        # =====================================================================

        # Chain 1: /scan ready → EKF
        chain_ekf = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=[ekf],
            )
        )

        # Chain 2: EKF TF ready → localization
        chain_localization = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_ekf_tf,
                on_exit=[map_server, amcl, lifecycle_starter_localization],
            )
        )

        # Chain 3: localization ready → navigation + custom lifecycle
        chain_navigation = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_localization_ready,
                on_exit=[
                    navigation,
                    lifecycle_starter_custom,
                    cmd_vel_bridge_node,
                    opentcs_vehicle,
                ],
            )
        )

        # Chain 4: route_server ready → route_graph_loader
        chain_route = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_route,
                on_exit=[route_graph_loader],
            )
        )

        # Material action GUI: delayed start
        from launch.actions import TimerAction
        material_delayed = TimerAction(
            period=5.0,
            actions=[material_action_gui],
        )

        actions.extend([
            # Chain gates
            wait_scan,
            chain_ekf,
            wait_ekf_tf,
            chain_localization,
            wait_localization_ready,
            chain_navigation,
            wait_route,
            chain_route,
            material_delayed,
        ])

        return actions

    return LaunchDescription([
        # Arguments
        profile_arg,
        map_file_arg,
        respawn_arg,
        vehicle_name_arg,
        namespace_arg,
        rviz_config_arg,
        # Opaque function for runtime profile evaluation
        OpaqueFunction(function=launch_setup),
    ])
