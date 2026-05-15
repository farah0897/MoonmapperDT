"""Static-map Nav2: localization_mode:=odom (default, demo) or amcl (experimental)."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)


def generate_launch_description() -> LaunchDescription:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")
    launch_dir = os.path.join(nav2_pkg, "launch")
    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")

    mode = LaunchConfiguration("localization_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    rviz = LaunchConfiguration("rviz")
    tf_log = LaunchConfiguration("tf_log")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")
    safety_to = LaunchConfiguration("safety_scan_timeout_sec")

    def impl(context):
        m = mode.perform(context).strip().lower()
        common = {
            "use_sim_time": use_sim_time,
            "map_file": map_file,
            "rviz": rviz,
            "tf_log": tf_log,
            "safety_stop_distance": safety_stop,
            "safety_front_angle_deg": safety_angle,
            "safety_scan_timeout_sec": safety_to,
        }
        if m == "amcl":
            return [
                LogInfo(msg="[moonmapper_nav2] nav2_static_map localization_mode=amcl (experimental)"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "nav2_amcl_static_map.launch.py")
                    ),
                    launch_arguments={
                        **common,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }.items(),
                ),
            ]
        if m not in ("", "odom"):
            raise RuntimeError(
                f"localization_mode must be 'odom' or 'amcl', got '{m}'"
            )
        return [
            LogInfo(
                msg=(
                    "[moonmapper_nav2] nav2_static_map localization_mode=odom "
                    "(identity map->odom, no 2D Pose Estimate)"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "nav2_odom_static_map.launch.py")
                ),
                launch_arguments=common.items(),
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "localization_mode",
                default_value="odom",
                description="'odom': identity map->odom, no AMCL. 'amcl': auto initial pose.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map_file",
                default_value=default_map,
                description="Path to map .yaml (padded test map by default).",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("tf_log", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.55"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            OpaqueFunction(function=impl),
        ]
    )
