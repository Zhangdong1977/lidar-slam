"""Application layer: Manual SLAM mapping with teleop control.

Layer: Application (Scene 1 — Manual Mapping)

Hardware profiles:
  gazebo      — Gazebo full simulation
  rs485       — RS-485 physical chassis + RPLIDAR S2L
  raspberry   — Raspberry Pi car (STM32 + RPLIDAR C1)
  rplidar_s2l — Standalone RPLIDAR S2L (no chassis, rf2o odometry)

Startup chain (event-driven):
  [Hardware Layer] → /scan, /odom, /imu, TF
  [Sensing]        → EKF (odom+imu fusion) or rf2o (laser odometry)
  [SLAM]           → slam_toolbox online_async → /map, TF map→odom
  [Teleop]         → ackermann_keyboard_teleop (optional)

Cross-layer services: watchdog + lifecycle_starter + rviz2

Usage:
  ros2 launch slam_main.launch.py hardware_profile:=gazebo
  ros2 launch slam_main.launch.py hardware_profile:=rs485
  ros2 launch slam_main.launch.py hardware_profile:=raspberry
  ros2 launch slam_main.launch.py hardware_profile:=rplidar_s2l use_teleop:=False
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def load_profile_yaml(profile_name_str, project_dir):
    """Load profile config yaml as dict."""
    path = os.path.join(project_dir, 'config', 'profiles', f'{profile_name_str}.yaml')
    if os.path.exists(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    return {}


def _load_params(yaml_path):
    """Load ros__parameters from YAML, bypassing node-name key matching.

    ROS2 Jazzy matches YAML keys against the fully qualified node name.
    A key like ``ekf_filter_node:`` only matches ``/ekf_filter_node``
    (root namespace), not ``/c30_1/ekf_filter_node`` (sub-namespace).
    This helper extracts the ``ros__parameters`` dict regardless of the
    top-level key so that parameters work under any namespace.
    """
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    for v in doc.values():
        if isinstance(v, dict) and 'ros__parameters' in v:
            return v['ros__parameters']
    return {}


def generate_launch_description():
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')

    # --- Launch arguments ---
    profile_arg = DeclareLaunchArgument(
        'hardware_profile', default_value='gazebo',
        description='Hardware profile: gazebo | rs485 | raspberry | rplidar_s2l')
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='False',
        description='Launch keyboard teleop for manual mapping')
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick', default_value='False',
        description='Launch joystick teleop for manual mapping (publishes /cmd_vel)')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz2 for visualization')
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value='',
        description='SLAM params file (empty = auto-select by profile)')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='',
        description='RViz config file (empty = auto-select)')
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Enable automatic respawn of crashed nodes')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Vehicle name (ROS2 namespace and Gazebo spawn name)')

    def launch_setup(context):
        profile = LaunchConfiguration('hardware_profile').perform(context)
        use_respawn = LaunchConfiguration('use_respawn')
        use_teleop_str = LaunchConfiguration('use_teleop').perform(context)
        use_joystick_str = LaunchConfiguration('use_joystick').perform(context)
        use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
        vehicle_namespace = LaunchConfiguration('namespace').perform(context)

        # Load profile config
        profile_cfg = load_profile_yaml(profile, project_dir)
        use_sim_time = profile_cfg.get('use_sim_time', profile == 'gazebo')
        base_frame = profile_cfg.get('frames', {}).get('base_frame', 'body_link')
        odom_frame = profile_cfg.get('frames', {}).get('odom_frame', 'odom')
        imu_topic = profile_cfg.get('sensors', {}).get('imu', {}).get('topic', '/imu')
        sensing_type = profile_cfg.get('sensing', {}).get('type', 'ekf')
        lidar_frame = profile_cfg.get('frames', {}).get('lidar_frame', 'laser')
        lidar_height = profile_cfg.get('vehicle', {}).get('lidar_height', 0.22)

        # Auto-select SLAM params based on profile
        slam_params_input = LaunchConfiguration('slam_params_file').perform(context)
        if slam_params_input:
            slam_params = slam_params_input
        else:
            if profile == 'gazebo':
                slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_ackermann.yaml')
            else:
                slam_params = os.path.join(project_dir, 'config', 'slam_toolbox_real.yaml')

        # Auto-select RViz config
        rviz_config_input = LaunchConfiguration('rviz_config').perform(context)
        if rviz_config_input:
            rviz_config = rviz_config_input
        else:
            rviz_config = os.path.join(project_dir, 'config', 'slam.rviz')

        # Config file paths
        ekf_config = os.path.join(project_dir, 'config', 'ekf.yaml')
        watchdog_config = os.path.join(project_dir, 'config', 'watchdog.yaml')

        actions = []

        # =====================================================================
        # Hardware Layer (profile-specific)
        # =====================================================================

        hardware_dir = os.path.join(project_dir, 'launch', 'hardware')

        is_gazebo = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'gazebo'"])
        is_rs485 = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'rs485'"])
        is_raspberry = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'raspberry'"])
        is_rplidar_s2l = PythonExpression(["'", LaunchConfiguration('hardware_profile'), "' == 'rplidar_s2l'"])

        hardware_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'gazebo_hardware.launch.py')),
            condition=IfCondition(is_gazebo),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
                'namespace': vehicle_namespace,
            }.items(),
        )

        hardware_rs485 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'rs485_hardware.launch.py')),
            condition=IfCondition(is_rs485),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
                'namespace': vehicle_namespace,
            }.items(),
        )

        hardware_raspberry = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'raspberry_hardware.launch.py')),
            condition=IfCondition(is_raspberry),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
                'namespace': vehicle_namespace,
            }.items(),
        )

        hardware_rplidar_s2l = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_dir, 'rplidar_s2l_hardware.launch.py')),
            condition=IfCondition(is_rplidar_s2l),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'use_respawn': 'True',
                'namespace': vehicle_namespace,
            }.items(),
        )

        actions.extend([hardware_gazebo, hardware_rs485, hardware_raspberry, hardware_rplidar_s2l])

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
            condition=IfCondition(
                PythonExpression(["'", use_rviz_str, "' == 'True'"])),
        )

        actions.extend([watchdog, rviz2])

        # =====================================================================
        # Sensing Layer: EKF or rf2o
        # =====================================================================

        # Wait for /scan topic before starting sensing
        wait_scan = Node(
            package='lidar_slam_nodes',
            executable='wait_for_topic',
            name='wait_scan',
            output='screen',
            parameters=[{
                'topic_name': 'scan',
                'topic_type': 'sensor_msgs/msg/LaserScan',
                'min_publishers': 1,
                'timeout': 30.0,
                'use_sim_time': use_sim_time,
            }],
        )

        # scan_range_filter: replace inf/NaN → valid range (Gazebo needs this)
        scan_filter = Node(
            package='lidar_slam_nodes',
            executable='scan_range_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(is_gazebo),
        )

        # EKF: fuses /odom + /imu → refined odom→base TF
        ekf_params = [_load_params(ekf_config), {
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'imu0': imu_topic,
        }]

        ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=ekf_params,
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            respawn=use_respawn,
            respawn_delay=2.0,
            condition=IfCondition(
                PythonExpression(["'", sensing_type, "' == 'ekf'"])),
        )

        # rf2o: laser odometry → odom→base TF (no IMU, no wheel odom)
        rf2o = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'laser_scan_topic': 'scan',
                'init_pose_from_topic': '',
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(
                PythonExpression(["'", sensing_type, "' == 'rf2o'"])),
        )

        # Static TF: base_link → laser frame (non-Gazebo profiles need this)
        # Gazebo profile has its own laser TF in gazebo_hardware.launch.py
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
                PythonExpression([
                    "'", LaunchConfiguration('hardware_profile'), "' != 'gazebo'",
                    " and '", LaunchConfiguration('hardware_profile'), "' != 'raspberry'"
                ])),
        )

        actions.extend([wait_scan, scan_filter, ekf, rf2o, laser_tf_non_gazebo])

        # =====================================================================
        # SLAM Layer: slam_toolbox online_async
        # =====================================================================

        # Wait for odom→base TF before starting SLAM
        wait_odom_tf = Node(
            package='lidar_slam_nodes',
            executable='wait_for_tf',
            name='wait_odom_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': odom_frame,
                'source_frame': base_frame,
                'timeout': 20.0,
                'check_period': 0.5,
            }],
        )

        # Use custom slam_toolbox launch with remappings for absolute topic names.
        # Pass namespace directly — the custom launch sets it on the LifecycleNode.
        # chain_slam is placed OUTSIDE the outer GroupAction to prevent
        # PushRosNamespace from stacking with the LifecycleNode's own namespace.
        slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(project_dir, 'launch', 'slam_toolbox_namespaced.py')),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': str(use_sim_time).lower(),
                'namespace': vehicle_namespace,
            }.items(),
        )

        # lifecycle_starter: activate vehicle_controller (Gazebo) or other LifecycleNodes
        lifecycle_nodes_slam = []
        chassis_type = profile_cfg.get('chassis', {}).get('type', 'gazebo')
        if chassis_type in ('gazebo', 'rs485'):
            lifecycle_nodes_slam.append('rs485_chassis_bridge')
        if chassis_type == 'gazebo':
            lifecycle_nodes_slam.extend([
                'rs485_chassis_receiver',
                'vehicle_controller',
            ])

        lifecycle_starter_slam = Node(
            package='lidar_slam_nodes',
            executable='lifecycle_starter',
            name='lifecycle_starter_slam',
            output='screen',
            parameters=[{
                'node_names': lifecycle_nodes_slam,
                'configure_timeout': 30.0,
                'activate_timeout': 30.0,
                'max_retries': 5,
                'retry_delay': 2.0,
                'startup_delay': 2.0,
                'monitor_period': 0.0,
                'starter_name': 'slam',
            }],
            condition=IfCondition(
                PythonExpression([str(len(lifecycle_nodes_slam) > 0)])),
        )

        # =====================================================================
        # Teleop (optional): keyboard control for manual mapping
        # =====================================================================

        teleop = Node(
            package='lidar_slam_nodes',
            executable='ackermann_keyboard_teleop',
            name='ackermann_keyboard_teleop',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression(["'", use_teleop_str, "' == 'True'"])),
        )

        # Joystick teleop: joy_node + teleop_twist_joy_node → /cmd_vel
        joystick_config = os.path.join(project_dir, 'config', 'joystick_c30.yaml')

        joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(
                PythonExpression(["'", use_joystick_str, "' == 'True'"])),
        )

        teleop_joystick = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joystick_config, {
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(
                PythonExpression(["'", use_joystick_str, "' == 'True'"])),
        )

        # =====================================================================
        # Event-driven startup chains
        # =====================================================================

        # Chain 1: /scan ready → EKF/rf2o + scan_filter
        chain_sensing = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=[],
            )
        )
        # EKF/rf2o and scan_filter start in parallel after /scan is ready
        # They are already in actions list, we just need the gate
        # Actually, let's make sensing depend on /scan via event
        # Move ekf/rf2o/scan_filter to only start after wait_scan exits

        # Remove ekf, rf2o, scan_filter from direct actions, add to chain
        actions.remove(ekf)
        actions.remove(rf2o)
        actions.remove(scan_filter)

        # Helper: wrap actions in GroupAction with namespace so that
        # event-triggered nodes also inherit PushRosNamespace.
        def ns_wrap(*acts):
            return GroupAction(actions=[PushRosNamespace(vehicle_namespace), *acts])

        chain_sensing = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=[ns_wrap(scan_filter, ekf, rf2o, wait_odom_tf)],
            )
        )

        # Chain 2: odom→base TF ready → SLAM
        # slam_toolbox uses custom launch with remappings; namespace comes from
        # the parent's PushRosNamespace. Do NOT wrap in ns_wrap.
        chain_slam = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_odom_tf,
                on_exit=[
                    slam_toolbox,
                    ns_wrap(lifecycle_starter_slam),
                ],
            )
        )

        # Chain 3: SLAM ready → teleop (delay slightly for slam to stabilize)
        from launch.actions import TimerAction
        teleop_delayed = TimerAction(
            period=3.0,
            actions=[ns_wrap(teleop, joy_node, teleop_joystick)],
        )

        actions.extend([
            chain_sensing,
            teleop_delayed,
        ])

        # Wrap most actions in GroupAction with namespace.
        # chain_slam is placed OUTSIDE because slam_toolbox sets namespace
        # directly on its LifecycleNode; PushRosNamespace would double it.
        return [
            GroupAction(actions=[
                PushRosNamespace(vehicle_namespace),
                *actions,
            ]),
            chain_slam,
        ]

    return LaunchDescription([
        # Arguments
        profile_arg,
        use_teleop_arg,
        use_joystick_arg,
        use_rviz_arg,
        slam_params_arg,
        rviz_config_arg,
        respawn_arg,
        namespace_arg,
        # Opaque function for runtime profile evaluation
        OpaqueFunction(function=launch_setup),
    ])
