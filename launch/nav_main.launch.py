"""Application layer: Dispatch integration with openTCS fleet management.

Layer: Application (Scene 3 — Dispatch Integration)

Hardware profiles:
  gazebo    — Gazebo full simulation (chassis + sensors simulated)
  rs485     — RS-485 physical chassis + RPLIDAR S2L + car_base_node (odom+IMU)
  raspberry — Raspberry Pi car (car_base_node drives STM32, RPLIDAR C1)

Startup chain (event-driven):
  [Hardware Layer]     → /scan, /odom, /imu, TF: odom→base
  [EKF Fusion]         → TF: odom→base (refined)
  [Localization]       → TF: map→odom (AMCL + map_server)
  [Nav2 Navigation]    → /cmd_vel, route planning, behavior tree
  [Application Layer]  → opentcs_vehicle, route_graph_loader, material_action_gui

Cross-layer services: watchdog + lifecycle_starter + rviz2

Usage:
  ros2 launch nav_main.launch.py hardware_profile:=gazebo
  ros2 launch nav_main.launch.py hardware_profile:=rs485
  ros2 launch nav_main.launch.py hardware_profile:=raspberry map_file:=/path/map.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def load_profile_yaml(profile_name_str, project_dir):
    """Load profile config yaml as dict."""
    import yaml
    path = os.path.join(project_dir, 'config', 'profiles', f'{profile_name_str}.yaml')
    if os.path.exists(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    return {}


def load_node_params_yaml(path, node_name):
    """Load ros__parameters for a node from a ROS2 parameter YAML file."""
    import yaml
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get(node_name, {}).get('ros__parameters', {})


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
        description='Vehicle name (ROS2 namespace and Gazebo spawn name)')
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0',
        description='Gazebo spawn X position')
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0',
        description='Gazebo spawn Y position')
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z', default_value='0.24',
        description='Gazebo spawn Z position')
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0',
        description='AMCL initial pose X')
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0',
        description='AMCL initial pose Y')
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0',
        description='AMCL initial pose yaw')
    start_gazebo_arg = DeclareLaunchArgument(
        'start_gazebo', default_value='True',
        description='Start Gazebo simulator process')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(project_dir, 'config', 'nav_multi.rviz'),
        description='RViz config file')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz2 for visualization')

    # Use OpaqueFunction to access launch configuration at runtime
    def launch_setup(context):
        profile = LaunchConfiguration('hardware_profile').perform(context)
        use_respawn_str = LaunchConfiguration('use_respawn').perform(context)
        use_respawn = use_respawn_str.lower() in ('true', '1', 'yes')
        vehicle_namespace = LaunchConfiguration('namespace').perform(context)
        use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
        spawn_x = LaunchConfiguration('spawn_x').perform(context)
        spawn_y = LaunchConfiguration('spawn_y').perform(context)
        spawn_z = LaunchConfiguration('spawn_z').perform(context)
        initial_pose_x = LaunchConfiguration('initial_pose_x').perform(context)
        initial_pose_y = LaunchConfiguration('initial_pose_y').perform(context)
        initial_pose_yaw = LaunchConfiguration('initial_pose_yaw').perform(context)
        start_gazebo = LaunchConfiguration('start_gazebo').perform(context)

        # Load profile config
        profile_cfg = load_profile_yaml(profile, project_dir)
        use_sim_time = profile_cfg.get('use_sim_time', profile == 'gazebo')
        base_frame = profile_cfg.get('frames', {}).get('base_frame', 'body_link')
        odom_frame = profile_cfg.get('frames', {}).get('odom_frame', 'odom')
        odom_topic = profile_cfg.get('sensors', {}).get('odom', {}).get('topic', 'odom')
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
        # Pass the extracted ros__parameters directly so they apply under namespaces.
        ekf_node_params = load_node_params_yaml(ekf_config, 'ekf_filter_node')
        ekf_node_params.update({
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'odom0': odom_topic,
            'imu0': imu_topic,
        })
        ekf_params = [ekf_node_params]

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
        # Hardware Layer (profile-specific)
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
                'namespace': vehicle_namespace,
                'spawn_x': spawn_x,
                'spawn_y': spawn_y,
                'spawn_z': spawn_z,
                'start_gazebo': start_gazebo,
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
        # Cross-layer: Watchdog + RViz2
        # =====================================================================

        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            condition=IfCondition(
                PythonExpression(["'", use_rviz_str, "' == 'True'"])),
        )

        watchdog = Node(
            package='lidar_slam_nodes',
            executable='node_watchdog',
            name='node_watchdog',
            output='screen',
            parameters=[watchdog_config, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            respawn=True,
            respawn_delay=5.0,
        )

        actions.extend([rviz2, watchdog])

        # =====================================================================
        # Sensing Layer: EKF (after /scan ready)
        # =====================================================================

        wait_scan = Node(
            package='lidar_slam_nodes',
            executable='wait_for_topic',
            name='wait_scan',
            output='screen',
            parameters=[{
                'topic_name': 'scan',
                'min_publishers': 1,
                'timeout': 30.0,
                'exit_on_timeout': False,
                'use_sim_time': use_sim_time,
            }],
        )

        ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=ekf_params,
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        # =====================================================================
        # Localization Layer (after EKF TF ready)
        # =====================================================================

        wait_ekf_tf = Node(
            package='lidar_slam_nodes',
            executable='wait_for_tf',
            name='wait_ekf_tf',
            output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': odom_frame,
                'source_frame': base_frame,
                'timeout': 20.0,
                'check_period': 0.5,
                'exit_on_timeout': False,
            }],
        )

        # Use localization sub-launch instead of inline nodes
        localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(project_dir, 'launch', 'localization.launch.py')),
            launch_arguments={
                'map': map_file,
                'namespace': vehicle_namespace,
                'use_sim_time': str(use_sim_time).lower(),
                'params_file': nav2_params,
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': str(use_respawn).lower(),
                'vehicle_name': LaunchConfiguration('vehicle_name').perform(context),
                'initial_pose_x': initial_pose_x,
                'initial_pose_y': initial_pose_y,
                'initial_pose_yaw': initial_pose_yaw,

            }.items(),
        )

        # Wait for localization to be ready (use /map topic — reliable DDS discovery)
        wait_localization_ready = Node(
            package='lidar_slam_nodes',
            executable='wait_for_topic',
            name='wait_localization_ready',
            output='screen',
            parameters=[{
                'topic_name': 'map',
                'min_publishers': 1,
                'timeout': 30.0,
                'exit_on_timeout': False,
                'use_sim_time': use_sim_time,
            }],
        )

        # =====================================================================
        # Navigation + Application Layer (after localization ready)
        # =====================================================================

        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(project_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'namespace': vehicle_namespace,
                'use_sim_time': str(use_sim_time).lower(),
                'autostart': 'True',
                'params_file': nav2_params,
                'use_composition': 'False',
                'use_respawn': str(use_respawn).lower(),
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
            output='screen',
            parameters=[opentcs_vehicle_config, {
                'use_sim_time': use_sim_time,
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'namespace': vehicle_namespace,
                'base_frame': base_frame,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        # =====================================================================
        # Application: route_graph_loader + material_action_gui
        # =====================================================================

        wait_route = Node(
            package='lidar_slam_nodes',
            executable='wait_for_service',
            name='wait_route',
            output='screen',
            parameters=[{
                'service_name': 'route_server/set_route_graph',
                'service_type': 'nav2_msgs/srv/SetRouteGraph',
                'timeout': 30.0,
                'exit_on_timeout': False,
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
                'route_graph_topic': 'route_graph_json',
            }],
            respawn=use_respawn,
            respawn_delay=2.0,
        )

        material_action_gui = Node(
            package='jvs_agv_material_actions',
            executable='material_action_gui',
            output='screen',
            parameters=[material_action_config, {
                'use_sim_time': use_sim_time,
                'vehicle_name': LaunchConfiguration('vehicle_name'),
            }],
        )

        # =====================================================================
        # Event-driven startup chains
        # =====================================================================

        # Helper: wrap actions in GroupAction with PushRosNamespace.
        # Required because RegisterEventHandler → OnProcessExit does NOT inherit
        # PushRosNamespace from the enclosing GroupAction (ROS2 launch limitation).
        def _ns_wrap(*nodes):
            if not vehicle_namespace:
                return list(nodes)
            return [GroupAction(actions=[PushRosNamespace(vehicle_namespace), *nodes])]

        # Chain 1: /scan ready → EKF + wait_ekf_tf (gate starts WITH dependency)
        chain_ekf = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=_ns_wrap(ekf, wait_ekf_tf),
            )
        )

        # Chain 2: EKF TF ready → localization + wait_localization_ready
        chain_localization = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_ekf_tf,
                on_exit=[
                    localization,  # IncludeLaunchDescription with own PushRosNamespace
                    *_ns_wrap(wait_localization_ready),
                ],
            )
        )

        # Chain 3: localization ready → navigation + custom lifecycle + wait_route
        chain_navigation = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_localization_ready,
                on_exit=[
                    navigation,  # IncludeLaunchDescription with own PushRosNamespace
                    *_ns_wrap(
                        lifecycle_starter_custom,
                        cmd_vel_bridge_node,
                        opentcs_vehicle,
                        wait_route,
                    ),
                ],
            )
        )

        # Chain 4: route_server ready → route_graph_loader
        chain_route = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_route,
                on_exit=_ns_wrap(route_graph_loader),
            )
        )

        # Material action GUI: delayed start
        material_delayed = TimerAction(
            period=5.0,
            actions=[material_action_gui],
        )

        actions.extend([
            # Chain gates (only wait_scan starts at T=0; others start with their dependency)
            wait_scan,
            chain_ekf,
            chain_localization,
            chain_navigation,
            chain_route,
            material_delayed,
        ])

        # Wrap all actions in GroupAction with namespace for multi-vehicle support.
        # When namespace='', PushRosNamespace('') is a no-op → backward compatible.
        return [GroupAction(actions=[
            PushRosNamespace(vehicle_namespace),
            *actions,
        ])]

    return LaunchDescription([
        # Arguments
        profile_arg,
        map_file_arg,
        respawn_arg,
        vehicle_name_arg,
        namespace_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        start_gazebo_arg,
        rviz_config_arg,
        use_rviz_arg,
        # Opaque function for runtime profile evaluation
        OpaqueFunction(function=launch_setup),
    ])
