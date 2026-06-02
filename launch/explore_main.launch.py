"""Application layer: Autonomous exploration mapping with frontier_explorer.

Layer: Application (Scene 2 — Auto Exploration)

Hardware profiles:
  gazebo    — Gazebo full simulation
  rs485     — RS-485 physical chassis + RPLIDAR S2L
  raspberry — Raspberry Pi car (STM32 + RPLIDAR C1)

Startup chain (event-driven):
  [Hardware Layer] → /scan, /odom, /imu, TF
  [EKF Fusion]     → TF: odom→base (refined)
  [SLAM]           → slam_toolbox online_async → /map, TF map→odom
  [Nav2]           → Navigation stack (NO AMCL — slam provides localization)
  [Explorer]       → frontier_explorer → NavigateToPose → Nav2 → /cmd_vel

Cross-layer services: watchdog + lifecycle_starter + rviz2

Usage:
  ros2 launch explore_main.launch.py hardware_profile:=gazebo
  ros2 launch explore_main.launch.py hardware_profile:=rs485
  ros2 launch explore_main.launch.py hardware_profile:=raspberry
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value='',
        description='SLAM params file (empty = auto-select by profile)')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file', default_value='',
        description='Nav2 params file (empty = auto-select exploration params)')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='',
        description='RViz config file (empty = auto-select)')
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Enable automatic respawn of crashed nodes')

    def launch_setup(context):
        profile = LaunchConfiguration('hardware_profile').perform(context)
        use_respawn_raw = LaunchConfiguration('use_respawn').perform(context)
        use_respawn = use_respawn_raw.lower() == 'true'

        # Load profile config
        profile_cfg = load_profile_yaml(profile, project_dir)
        use_sim_time = profile_cfg.get('use_sim_time', profile == 'gazebo')
        base_frame = profile_cfg.get('frames', {}).get('base_frame', 'body_link')
        odom_frame = profile_cfg.get('frames', {}).get('odom_frame', 'odom')
        imu_topic = profile_cfg.get('sensors', {}).get('imu', {}).get('topic', '/imu')
        chassis_type = profile_cfg.get('chassis', {}).get('type', 'gazebo')
        lidar_frame = profile_cfg.get('frames', {}).get('lidar_frame', 'laser')
        lidar_height = profile_cfg.get('vehicle', {}).get('lidar_height', 0.22)

        # Auto-select SLAM params
        slam_params_input = LaunchConfiguration('slam_params_file').perform(context)
        if slam_params_input:
            slam_params = slam_params_input
        else:
            if profile == 'gazebo':
                slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_ackermann.yaml')
            else:
                slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_real.yaml')

        # Auto-select Nav2 params
        nav2_params_input = LaunchConfiguration('nav2_params_file').perform(context)
        if nav2_params_input:
            nav2_params = nav2_params_input
        else:
            nav2_params = os.path.join(project_dir, 'config', 'nav2_params_exploration.yaml')

        # Auto-select RViz config
        rviz_config_input = LaunchConfiguration('rviz_config').perform(context)
        if rviz_config_input:
            rviz_config = rviz_config_input
        else:
            rviz_config = os.path.join(project_dir, 'config', 'explore.rviz')

        # Config file paths
        ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')
        watchdog_config = os.path.join(project_dir, 'config', 'watchdog.yaml')
        frontier_params = os.path.join(project_dir, 'config', 'frontier_explorer_params.yaml')

        # Determine lifecycle nodes for explore scenario
        lifecycle_nodes_explore = []
        if chassis_type in ('gazebo', 'rs485'):
            lifecycle_nodes_explore.append('cmd_vel_bridge')
        if chassis_type in ('gazebo', 'rs485'):
            lifecycle_nodes_explore.append('rs485_chassis_bridge')
        if chassis_type == 'gazebo':
            lifecycle_nodes_explore.extend([
                'rs485_chassis_receiver', 'vehicle_controller'])

        # Build EKF parameters with profile overrides
        ekf_params = [ekf_config, {
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'imu0': imu_topic,
        }]

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

        watchdog = Node(
            package='lidar_slam_nodes',
            executable='node_watchdog',
            name='node_watchdog',
            output='screen',
            parameters=[watchdog_config, {'use_sim_time': use_sim_time}],
            respawn=True,
            respawn_delay=5.0,
        )

        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
        )

        actions.extend([watchdog, rviz2])

        # =====================================================================
        # Sensing Layer: EKF (after /scan ready)
        # =====================================================================

        wait_scan = Node(
            package='lidar_slam_nodes',
            executable='wait_for_topic',
            output='screen',
            parameters=[{
                'topic_name': '/scan',
                'min_publishers': 1,
                'timeout': 30.0,
                'use_sim_time': use_sim_time,
            }],
        )

        scan_filter = Node(
            package='lidar_slam_nodes',
            executable='scan_range_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(is_gazebo),
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

        # Static TF: base → laser (non-Gazebo)
        laser_tf_non_gazebo = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', str(lidar_height),
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', base_frame,
                '--child-frame-id', lidar_frame,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' != 'gazebo'"])),
        )

        # Wait for EKF to provide odom→base TF
        wait_odom_tf = Node(
            package='lidar_slam_nodes',
            executable='wait_for_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': odom_frame,
                'source_frame': base_frame,
                'timeout': 20.0,
                'check_period': 0.5,
            }],
        )

        # Wait for SLAM to provide map→base TF
        wait_map_tf = Node(
            package='lidar_slam_nodes',
            executable='wait_for_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'map',
                'source_frame': base_frame,
                'timeout': 30.0,
                'check_period': 0.5,
            }],
        )

        # Wait for Nav2 lifecycle_manager to signal readiness
        wait_nav2_ready = Node(
            package='lidar_slam_nodes',
            executable='wait_for_service',
            output='screen',
            parameters=[{
                'service_name': '/lifecycle_manager_navigation/is_active',
                'service_type': 'std_srvs/srv/Trigger',
                'timeout': 30.0,
                'use_sim_time': use_sim_time,
            }],
        )

        # Chain 1: /scan ready → EKF + scan_filter + wait_odom_tf
        chain_sensing = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=[scan_filter, ekf, wait_odom_tf],
            )
        )

        actions.extend([wait_scan, chain_sensing, laser_tf_non_gazebo])

        # =====================================================================
        # SLAM Layer: slam_toolbox (after EKF TF ready)
        # =====================================================================

        slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(FindPackageShare('slam_toolbox').find('slam_toolbox'),
                             'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': str(use_sim_time).lower(),
            }.items(),
        )

        chain_slam = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_odom_tf,
                on_exit=[slam_toolbox, wait_map_tf],
            )
        )

        actions.extend([chain_slam])

        # =====================================================================
        # Navigation Layer (after SLAM provides map→odom TF)
        # =====================================================================

        # Nav2 navigation ONLY — no AMCL, no map_server
        # slam_toolbox provides localization (map→odom TF) and /map topic
        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(project_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'autostart': 'True',
                'params_file': nav2_params,
                'use_composition': 'False',
                'use_respawn': use_respawn_raw.lower(),
            }.items(),
        )

        # cmd_vel_bridge: Nav2 Twist → Ackermann (LifecycleNode)
        cmd_vel_bridge = Node(
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

        # lifecycle_starter for explore nodes
        lifecycle_starter_explore = Node(
            package='lidar_slam_nodes',
            executable='lifecycle_starter',
            name='lifecycle_starter_explore',
            output='screen',
            parameters=[{
                'node_names': lifecycle_nodes_explore,
                'configure_timeout': 30.0,
                'activate_timeout': 30.0,
                'max_retries': 5,
                'retry_delay': 2.0,
                'startup_delay': 5.0,
                'monitor_period': 0.0,
                'starter_name': 'explore',
            }],
        )

        chain_navigation = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_map_tf,
                on_exit=[
                    navigation,
                    cmd_vel_bridge,
                    lifecycle_starter_explore,
                    wait_nav2_ready,
                ],
            )
        )

        actions.extend([chain_navigation])

        # =====================================================================
        # Application: frontier_explorer (after Nav2 is ready)
        # =====================================================================

        frontier_explorer = Node(
            package='lidar_slam_nodes',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[frontier_params, {
                'use_sim_time': use_sim_time,
            }],
        )

        chain_explorer = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_nav2_ready,
                on_exit=[frontier_explorer],
            )
        )

        actions.extend([chain_explorer])

        return actions

    return LaunchDescription([
        # Arguments
        profile_arg,
        slam_params_arg,
        nav2_params_arg,
        rviz_config_arg,
        respawn_arg,
        # Opaque function for runtime profile evaluation
        OpaqueFunction(function=launch_setup),
    ])
