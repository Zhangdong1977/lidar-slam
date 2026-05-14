import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('ackermann_control')
    xacro_file = os.path.join(
        os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam'), 'models', 'ackermann', 'ackermann.xacro')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    controller_yaml = PathJoinSubstitution([pkg_share, 'config', 'gz_ros2_control.yaml'])
    params_yaml = PathJoinSubstitution([pkg_share, 'config', 'ackermann_params.yaml'])

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

    # 2. Spawn robot in Gazebo (includes internal controller_manager via gz_ros2_control-system plugin)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'ackermann_robot',
             '-topic', 'robot_description',
             '-x', '0', '-y', '0', '-z', '0.24'],
        output='screen',
    )

    # 3. Load controllers into Gazebo-internal controller_manager (delayed)
    load_controllers = ExecuteProcess(
        cmd=['python3', os.path.join(os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam'), 'scripts', 'load_controllers.py')],
        output='screen',
    )

    # 4. VehicleController: bridges /steering_angle + /velocity → forward command controllers
    vehicle_controller = Node(
        package='ackermann_control',
        executable='vehicle_controller',
        name='vehicle_controller',
        parameters=[params_yaml],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        # Spawn robot after Gazebo + robot_state_publisher are up (5s delay)
        TimerAction(period=5.0, actions=[spawn_robot]),
        # Load controllers after spawn + Gazebo controller_manager init (10s delay)
        TimerAction(period=10.0, actions=[load_controllers]),
        # Start VehicleController after controllers are activated (12s delay)
        TimerAction(period=12.0, actions=[vehicle_controller]),
    ])
