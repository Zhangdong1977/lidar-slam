"""Gazebo hardware layer: simulator + RS-485 virtual loopback.

Starts:
  - socat virtual serial port pair
  - Gazebo Harmonic simulator
  - ros_gz_bridge (scan, odom, imu, clock)
  - robot_state_publisher (ackermann.xacro)
  - static TF: body_link -> lidar frame
  - rs485_chassis_receiver (reads virtual serial)
  - vehicle_controller (ros2_control interface)
  - rs485_chassis_bridge (writes virtual serial, 1s delay)

Launch arguments:
  use_sim_time         (default: True)
  use_respawn          (default: True)
  namespace            (default: '')  -- vehicle name / ROS2 namespace
  start_gazebo         (default: True)
  spawn_x              (default: '0')
  spawn_y              (default: '0')
  spawn_z              (default: '0.24')
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
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
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def load_node_params_yaml(path, node_name):
    """Load ros__parameters for a node from a ROS2 parameter YAML file."""
    import yaml
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get(node_name, {}).get('ros__parameters', {})


def launch_setup(context):
    project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
    world_file = os.path.join(project_dir, 'worlds', 'factory.sdf')
    rs485_config = os.path.join(project_dir, 'config', 'rs485_bridge.yaml')
    xacro_file = os.path.join(project_dir, 'models', 'ackermann', 'ackermann.xacro')

    ackermann_share = FindPackageShare('ackermann_control').perform(context)
    params_yaml = os.path.join(ackermann_share, 'config', 'ackermann_params.yaml')
    vehicle_controller_params = load_node_params_yaml(params_yaml, 'vehicle_controller')

    # Resolve substitutions eagerly — chain nodes (RegisterEventHandler → OnProcessExit)
    # execute outside the GroupAction context, where LaunchConfiguration lookups fail.
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'True'
    use_respawn = LaunchConfiguration('use_respawn').perform(context).lower() in ('true', '1', 'yes')

    # Use namespace as Gazebo model name; fallback for single-vehicle (no namespace)
    namespace = LaunchConfiguration('namespace').perform(context)
    gazebo_name = namespace if namespace else 'ackermann_robot'

    start_gazebo = LaunchConfiguration('start_gazebo').perform(context).lower() in (
        'true', '1', 'yes')
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    chassis_cmd_port = f'/tmp/{gazebo_name}_chassis_cmd'
    chassis_recv_port = f'/tmp/{gazebo_name}_chassis_recv'
    ros_topic_prefix = f'/{namespace}' if namespace else ''
    scan_topic = f'{ros_topic_prefix}/scan' if ros_topic_prefix else 'scan'
    odom_topic = f'{ros_topic_prefix}/odom' if ros_topic_prefix else 'odom'
    imu_topic = f'{ros_topic_prefix}/imu' if ros_topic_prefix else 'imu'
    robot_description_topic = (
        f'{ros_topic_prefix}/robot_description'
        if ros_topic_prefix else '/robot_description')

    # Gazebo resource paths
    gz_model_paths = ':'.join([
        os.path.join(project_dir, 'models'),
        os.path.join(project_dir, 'third-party', 'aws-robomaker-small-warehouse-world', 'models'),
    ])

    # A1. socat: virtual serial port pair
    socat = ExecuteProcess(
        cmd=[
            'socat',
            f'pty,link={chassis_cmd_port},raw,echo=0',
            f'pty,link={chassis_recv_port},raw,echo=0',
        ],
        output='screen',
    )

    # A2. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('start_gazebo')),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # A3. ros_gz_bridge — bridge Gazebo topics to relative ROS2 names
    # Gazebo topics are model-scoped (/model/{name}/odom, etc.) so each vehicle
    # publishes to its own topic. Bridge subscribes to the model-scoped paths and
    # remaps to relative names so PushRosNamespace adds the vehicle prefix.
    # /clock is global; bridge it only from the launch instance that starts Gazebo.
    bridge_names = ['scan', 'odom', 'imu']
    bridges = {
        'scan': {
            'ros_topic_name': scan_topic,
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_topic_name': f'/model/{gazebo_name}/scan_raw',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS',
        },
        'odom': {
            'ros_topic_name': odom_topic,
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_topic_name': f'/model/{gazebo_name}/odom',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS',
        },
        'imu': {
            'ros_topic_name': imu_topic,
            'ros_type_name': 'sensor_msgs/msg/Imu',
            'gz_topic_name': f'/model/{gazebo_name}/imu',
            'gz_type_name': 'gz.msgs.IMU',
            'direction': 'GZ_TO_ROS',
        },
    }
    if start_gazebo:
        bridge_names.append('clock')
        bridges['clock'] = {
            'ros_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_topic_name': '/clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        }

    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='ros_gz_bridge',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'use_sim_time': use_sim_time,
            'bridge_names': bridge_names,
            'bridges': bridges,
        }],
    )

    # A4. Robot description — pass namespace and model name to xacro
    xacro_args = (
        f' gz_ros2_control_ns:={namespace}'
        f' gz_model_name:={gazebo_name}'
        f' robot_description_topic:={robot_description_topic}')
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, xacro_args
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        # Keep robot_description per vehicle so each Gazebo controller_manager
        # consumes the URDF generated for its own namespace/model.
        remappings=[
            ('robot_description', robot_description_topic),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # A5. static TF: body_link -> lidar (Gazebo model prefix in child frame)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.22',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'body_link',
            '--child-frame-id', f'{gazebo_name}/body_link/lidar',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # --- Spawn robot (namespace as name, parameterized position) ---
    # ExecuteProcess does NOT inherit PushRosNamespace.
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', gazebo_name,
             '-topic', robot_description_topic,
             '-x', spawn_x, '-y', spawn_y, '-z', spawn_z],
        output='screen',
    )

    # --- Controller loading ---
    wait_cm = Node(
        package='lidar_slam_nodes',
        executable='wait_for_service',
        name='wait_for_cm',
        output='screen',
        parameters=[{
            'service_name': f'{gazebo_name}/controller_manager/list_controllers',
            'service_type': 'controller_manager_msgs/srv/ListControllers',
            'timeout': 15.0,
            'exit_on_timeout': False,
            'use_sim_time': use_sim_time,
        }],
    )

    load_controllers = Node(
        package='lidar_slam_nodes',
        executable='load_controllers',
        output='screen',
        parameters=[{
            'gazebo_model_name': gazebo_name,
        }],
    )

    wait_joints = Node(
        package='lidar_slam_nodes',
        executable='wait_for_topic',
        name='wait_for_joints',
        output='screen',
        parameters=[{
            'topic_name': 'joint_states',
            'min_publishers': 1,
            'timeout': 30.0,
            'exit_on_timeout': False,
            'use_sim_time': use_sim_time,
        }],
    )

    # --- RS-485 chain (sim loopback) ---
    rs485_receiver = Node(
        package='lidar_slam_nodes',
        executable='rs485_chassis_receiver',
        output='screen',
        parameters=[rs485_config, {
            'serial_port': chassis_recv_port,
            'use_sim_time': use_sim_time,
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    vehicle_controller = Node(
        package='ackermann_control',
        executable='vehicle_controller',
        name='vehicle_controller',
        parameters=[vehicle_controller_params],
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
        parameters=[rs485_config, {
            'serial_port': chassis_cmd_port,
            'use_sim_time': use_sim_time,
        }],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # --- Event chains ---
    # Helper: wrap actions in GroupAction with PushRosNamespace.
    # Required because RegisterEventHandler → OnProcessExit does NOT inherit
    # PushRosNamespace from the parent GroupAction (ROS2 launch limitation).
    # NOTE: wait_cm / load_controllers use gazebo_name-prefixed service paths
    # (absolute), so they must NOT be wrapped — otherwise the paths double-prefix.
    def _ns_wrap(*nodes):
        if not namespace:
            return list(nodes)
        return [GroupAction(actions=[PushRosNamespace(namespace), *nodes])]

    chain_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[wait_cm],  # uses absolute service paths via gazebo_name
        )
    )
    chain_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_cm,
            on_exit=[load_controllers],  # uses absolute service paths via gazebo_name
        )
    )
    chain_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=load_controllers,
            on_exit=_ns_wrap(wait_joints),
        )
    )
    chain_rs485_sim = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_joints,
            on_exit=[
                *_ns_wrap(rs485_receiver, vehicle_controller),
                TimerAction(period=1.0, actions=_ns_wrap(rs485_bridge)),
            ],
        )
    )

    return [
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
    ]


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Vehicle name (used as Gazebo spawn name and ROS2 namespace)'),
        DeclareLaunchArgument(
            'start_gazebo', default_value='True',
            description='Start Gazebo simulator process'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0',
            description='Spawn X position in Gazebo'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0',
            description='Spawn Y position in Gazebo'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.24',
            description='Spawn Z position in Gazebo'),
        # Opaque function for runtime parameter evaluation
        OpaqueFunction(function=launch_setup),
    ])
