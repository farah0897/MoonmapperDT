"""Short NavigateThroughPoses mission on static-map Nav2 (odom localization default)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")
    launch_dir = os.path.join(nav2_pkg, "launch")

    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")
    default_wp = os.path.join(nav2_pkg, "config", "nav2_static_map_short_waypoints.yaml")
    mission_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2_mission.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization_mode = LaunchConfiguration("localization_mode")
    map_file = LaunchConfiguration("map_file")
    waypoints_file = LaunchConfiguration("waypoints_file")
    rviz = LaunchConfiguration("rviz")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    def impl(context):
        mode = localization_mode.perform(context).strip().lower()
        map_resolved = map_file.perform(context)
        wp_resolved = waypoints_file.perform(context)

        nav2_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_static_map.launch.py")),
            launch_arguments={
                "localization_mode": localization_mode,
                "use_sim_time": use_sim_time,
                "map_file": map_file,
                "rviz": "false",
                "tf_log": "true",
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
            }.items(),
        )

        mission = Node(
            package="moonmapper_nav2",
            executable="nav2_mission_client_node",
            name="nav2_mission_client_node",
            output="screen",
            parameters=[
                ParameterFile(wp_resolved, allow_substs=False),
                {
                    "map_yaml_path": map_resolved,
                    "localization_mode": mode if mode in ("odom", "amcl") else "odom",
                },
            ],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2_static_map_mission",
            output="screen",
            arguments=["-d", mission_rviz],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(rviz),
        )

        return [
            LogInfo(
                msg=(
                    f"[moonmapper_nav2] static-map mission test mode={mode} "
                    f"map={map_resolved} waypoints={wp_resolved}"
                )
            ),
            nav2_stack,
            mission,
            rviz_node,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("localization_mode", default_value="odom"),
            DeclareLaunchArgument("map_file", default_value=default_map),
            DeclareLaunchArgument("waypoints_file", default_value=default_wp),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            OpaqueFunction(function=impl),
        ]
    )
