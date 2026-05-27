"""Custom localization launch with proper lifecycle_manager configuration.

Based on nav2_bringup/localization_launch.py (ROS2 Jazzy).
Changes: lifecycle_manager receives configured_params including service_call_timeout,
so it can survive transient Fast-DDS RMW timeouts under heavy system load.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    vehicle_name = LaunchConfiguration('vehicle_name')

    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            # map_server: no map file provided (use yaml config)
            Node(
                condition=IfCondition(
                    EqualsSubstitution(map_yaml_file, '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # map_server: map file provided via launch argument
            Node(
                condition=IfCondition(
                    NotEqualsSubstitution(map_yaml_file, '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # AMCL: remap initialpose to /{vehicle_name}/initialpose
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('initialpose', ['/', vehicle_name, '/initialpose']),
                ],
            ),
            # KEY FIX: lifecycle_manager receives configured_params (from yaml)
            # so service_call_timeout from nav2_params yaml is actually applied.
            # The standard localization_launch.py does NOT pass configured_params
            # to lifecycle_manager, causing RMW timeouts to kill the entire startup.
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    configured_params,
                    {'autostart': autostart, 'node_names': lifecycle_nodes},
                ],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to map yaml file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('vehicle_name', default_value='ackermann_robot'),
        load_nodes,
    ])
