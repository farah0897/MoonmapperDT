"""Kun Nav2-lokalisering (map_server + AMCL) for rask TF/kart-test — ikke full navigasjon."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")
    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")
    params = os.path.join(nav2_pkg, "config", "nav2_params_moonmapper.yaml")
    nav2_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    rviz = LaunchConfiguration("rviz")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "map": map_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
            "params_file": params,
            "container_name": "nav2_container",
            "log_level": "info",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2_localization",
        output="screen",
        arguments=["-d", nav2_rviz],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[moonmapper_nav2] localization: standardkart er moonmapper_test_map_padded.yaml. "
                    "Ved feil: restart launch."
                )
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map_file",
                default_value=default_map,
                description="Full sti til kart-.yaml for map_server.",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            localization,
            rviz_node,
        ]
    )
