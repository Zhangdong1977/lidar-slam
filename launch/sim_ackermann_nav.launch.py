import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_dir = '/home/pi/lidar-slam'
    world_file = os.path.join(project_dir, 'worlds', 'ackermann_test.sdf')
    nav2_params = os.path.join(project_dir, 'config', 'nav2_params_ackermann.yaml')
    map_file = os.path.join(project_dir, 'maps', 'ackermann_map.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(project_dir, 'models'),
    )

    # 1. Gazebo Harmonic with world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. ros_gz_bridge: sensors only
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{
            'qos_overrides./tf.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # 3. ackermann_control (robot_state_publisher + controller_manager + spawn)
    ackermann_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ackermann_control'), '/launch/ackermann_control.launch.py'
        ]),
    )

    # 4. Static TF: body_link → laser frame
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

    # 5. Nav2 bringup (map_server + AMCL + navigation stack)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    # 6. cmd_vel_bridge: Nav2 Twist → Ackermann steering_angle/velocity
    cmd_vel_bridge = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(project_dir, 'scripts', 'cmd_vel_bridge.py'),
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        output='screen',
    )

    # 7. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,
        # Delay ackermann_control until Gazebo is running
        TimerAction(
            period=2.0,
            actions=[ackermann_control],
        ),
        laser_tf,
        # Delay Nav2 until robot is spawned and controllers are active
        TimerAction(
            period=15.0,
            actions=[nav2_bringup],
        ),
        cmd_vel_bridge,
        rviz2,
    ])
