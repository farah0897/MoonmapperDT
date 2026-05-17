"""Kun slam_toolbox (forutsetter /scan, /odom, /tf).

async_slam_toolbox_node er en LifecycleNode: uten Configure+Activate publiseres ikke /map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("moonmapper_mapping")
    default_params = os.path.join(pkg_share, "config", "slam_toolbox_online_async.yaml")

    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "use_lifecycle_manager": ParameterValue(
                    use_lifecycle_manager, value_type=bool
                ),
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[moonmapper_mapping] slam_toolbox: activate (mapping)."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Synkroniser med Gazebo /clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Lifecycle: configure + activate automatisk.",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="false: bruk EmitEvent/RegisterEventHandler som slam_toolbox upstream.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params,
                description="YAML med slam_toolbox-ros__parameters.",
            ),
            slam,
            configure_event,
            activate_event,
        ]
    )
