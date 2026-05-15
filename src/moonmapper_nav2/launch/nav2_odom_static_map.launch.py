"""Nav2 + static map, sim odom baseline: identity map->odom, no AMCL, no 2D Pose Estimate.

Recommended bachelor-demo stack. Requires Gazebo (sim_rover_clean) first.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
import _nav2_static_map_common as _common  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")
    launch_dir = os.path.join(nav2_pkg, "launch")

    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")
    default_params = os.path.join(nav2_pkg, "config", "nav2_params_odom_static_map.yaml")
    default_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")
    safety_to = LaunchConfiguration("safety_scan_timeout_sec")
    tf_log = LaunchConfiguration("tf_log")

    def impl(context):
        map_resolved = map_file.perform(context)
        params_resolved = params_file.perform(context)

        map_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "map_server_bringup.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map": map_file,
                "params_file": params_file,
                "autostart": "true",
            }.items(),
        )

        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "namespace": "",
                "use_namespace": "false",
                "slam": "False",
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "False",
                "log_level": "info",
                "use_localization": "False",
            }.items(),
        )

        actions = [
            _common.log_header("odom", map_resolved, params_resolved),
            _common.map_odom_identity_tf(use_sim_time),
            map_server,
            _common.depth_to_scan_node(use_sim_time),
            navigation,
            _common.safety_node(use_sim_time, safety_stop, safety_angle, safety_to),
            _common.rviz_node(use_sim_time, default_rviz, rviz),
        ]
        if tf_log.perform(context).lower() in ("true", "1", "yes"):
            actions.insert(2, _common.tf_chain_logger(use_sim_time))
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_file", default_value=default_map),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "tf_log",
                default_value="true",
                description="Log map->odom, odom->base_footprint every 5s.",
            ),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.55"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            OpaqueFunction(function=impl),
        ]
    )
