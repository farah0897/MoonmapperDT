"""Nav2 + static map + AMCL (experimental): auto initial pose, no identity map->odom."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
import _nav2_static_map_common as _common  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")

    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")
    default_params = os.path.join(nav2_pkg, "config", "nav2_params_amcl_static_map.yaml")
    default_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    rviz = LaunchConfiguration("rviz")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")
    safety_to = LaunchConfiguration("safety_scan_timeout_sec")
    tf_log = LaunchConfiguration("tf_log")

    def impl(context):
        map_resolved = map_file.perform(context)
        params_resolved = params_file.perform(context)
        ix = initial_x.perform(context)
        iy = initial_y.perform(context)
        iyaw = initial_yaw.perform(context)

        localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_launch_dir, "localization_amcl_static_map.launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "params_file": params_file,
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
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
            _common.log_header(
                f"amcl (initial_pose x={ix} y={iy} yaw={iyaw})",
                map_resolved,
                params_resolved,
            ),
            localization,
            _common.depth_to_scan_node(use_sim_time),
            navigation,
            _common.safety_node(
                use_sim_time, safety_stop, safety_angle, safety_to, name="safety_obstacle_nav2_amcl"
            ),
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
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.0",
                description="AMCL set_initial_pose.x in map frame (match Gazebo spawn_x).",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="0.0",
                description="AMCL set_initial_pose.y in map frame (match Gazebo spawn_y).",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="0.0",
                description="AMCL set_initial_pose.yaw (rad).",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("tf_log", default_value="true"),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.55"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            OpaqueFunction(function=impl),
        ]
    )
