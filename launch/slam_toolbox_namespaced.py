"""Custom slam_toolbox launch with namespace-aware remappings.

Based on upstream online_async_launch.py.
The upstream file hardcodes namespace='' on the LifecycleNode, which prevents
PushRosNamespace from applying to topics like /map, /scan, /tf.

This file adds remappings to redirect absolute topic names (/map, /scan, /tf)
to relative names that resolve under the parent's PushRosNamespace.

NOTE: The parent launch must wrap this in a GroupAction with PushRosNamespace
for the namespace to take effect. Do NOT set namespace= on the LifecycleNode
here — that would stack with PushRosNamespace and cause double namespace
(e.g. /c30_1/c30_1/map).

Usage (from parent launch):
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource('slam_toolbox_namespaced.py'),
        launch_arguments={
            'slam_params_file': ...,
            'use_sim_time': 'false',
        }.items(),
    )
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _load_params(yaml_path):
    """Load ros__parameters from YAML, bypassing node-name key matching.

    ROS2 Jazzy matches YAML keys against the fully qualified node name.
    A key like ``slam_toolbox:`` only matches ``/slam_toolbox`` (root namespace),
    not ``/c30_1/slam_toolbox`` (sub-namespace).  This helper extracts the
    ``ros__parameters`` dict regardless of the top-level key so that
    parameters work under any namespace.
    """
    if not yaml_path or not os.path.isfile(yaml_path):
        return {}
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    if not doc:
        return {}
    for v in doc.values():
        if isinstance(v, dict) and 'ros__parameters' in v:
            return v['ros__parameters']
    return {}


def generate_launch_description():

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value='',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # slam_toolbox uses absolute topic names (/map, /scan, /tf) internally.
    # Remappings redirect them to relative names that resolve under namespace.
    # The 'namespace' parameter is read from the launch argument and set directly
    # on the LifecycleNode. The parent launch must NOT wrap this in PushRosNamespace
    # (would cause /c30_1/c30_1/map double namespace).
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot namespace for multi-vehicle support')

    def make_slam_node(context):
        """Resolve slam_params_file at launch time and parse YAML as dict."""
        params_path = slam_params_file.perform(context)
        slam_yaml_params = _load_params(params_path)

        node = LifecycleNode(
            parameters=[
                slam_yaml_params,
                {
                    'use_lifecycle_manager': use_lifecycle_manager,
                    'use_sim_time': use_sim_time,
                },
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen',
            remappings=[
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
                ('/scan', 'scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )

        configure_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(
                AndSubstitution(autostart,
                                NotSubstitution(use_lifecycle_manager))),
        )

        activate_event = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    LogInfo(
                        msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )),
                ],
            ),
            condition=IfCondition(
                AndSubstitution(autostart,
                                NotSubstitution(use_lifecycle_manager))),
        )

        return [node, configure_event, activate_event]

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(OpaqueFunction(function=make_slam_node))

    return ld
