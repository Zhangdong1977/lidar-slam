import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('ackermann_control')
    xacro_file = os.path.join(
        '/home/pi/lidar-slam', 'models', 'ackermann', 'ackermann.xacro')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # 1. robot_state_publisher
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

    # 2. ros2_control node (controller_manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'gz_ros2_control.yaml']),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # 3. Spawn robot in Gazebo (one-shot, delayed until Gazebo is ready)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.17'],
        output='screen',
    )

    # 4. Load and activate controllers (one-shot commands)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen',
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_velocity_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        # Spawn robot after Gazebo + ros2_control are up (5s delay)
        TimerAction(period=5.0, actions=[spawn_robot]),
        # Load controllers after spawn (6s delay)
        TimerAction(period=6.0, actions=[
            load_joint_state_broadcaster,
            load_position_controller,
            load_velocity_controller,
        ]),
    ])
