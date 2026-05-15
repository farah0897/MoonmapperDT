"""Nav2 odom-baseline: ingen map_server, ingen AMCL, ingen 2D Pose Estimate.

Starter Nav2 navigation-stack i odom med rolling costmaps. LaserScan kommer fra
`depth_to_scan_node` -> `/scan` (samme som øvrige moonmapper_nav2 odom-launch).

Testrekkefølge og akseptansekriterier: se `moonmapper_nav2/README.md` (seksjon
«Nav2 odom baseline»).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    default_params = os.path.join(nav2_pkg, "config", "nav2_params_odom_baseline.yaml")
    odom_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2_odom_baseline.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")
    safety_to = LaunchConfiguration("safety_scan_timeout_sec")

    def impl(context):
        params_resolved = params_file.perform(context)

        depth_to_scan = Node(
            package="moonmapper_autonomy",
            executable="depth_to_scan_node",
            name="depth_to_scan_node",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {
                    "depth_image_topic": "/depth_camera/depth_image",
                    "camera_info_topic": "/depth_camera/camera_info",
                    "scan_topic": "/scan",
                    "scan_height_mode": "roi_min",
                    "roi_top_ratio": 0.35,
                    "roi_bottom_ratio": 0.65,
                    "center_crop_ratio": 0.90,
                    "min_valid_points_per_column": 2,
                    "ground_filter_enabled": True,
                    "ground_filter_bottom_roi_ratio": 0.12,
                    "roi_percentile": 0.20,
                    "range_min": 0.15,
                    "range_max": 3.0,
                    "scan_time": 0.1,
                    "output_frame_id": "",
                },
            ],
        )

        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "namespace": "",
                "use_namespace": "false",
                "slam": "False",
                "map": "",
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "False",
                "log_level": "info",
                "use_localization": "False",
            }.items(),
        )

        safety = Node(
            package="moonmapper_autonomy",
            executable="safety_obstacle_node",
            name="safety_obstacle_nav2_odom_baseline",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {
                    "input_cmd_topic": "/cmd_vel_raw",
                    "output_cmd_topic": "/cmd_vel",
                    "scan_topic": "/scan",
                    "front_stop_distance": ParameterValue(safety_stop, value_type=float),
                    "front_angle_deg": ParameterValue(safety_angle, value_type=float),
                    "scan_timeout_sec": ParameterValue(safety_to, value_type=float),
                    "allow_reverse_when_blocked": True,
                    "publish_safety_debug": True,
                },
            ],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2_odom_baseline",
            output="screen",
            arguments=["-d", odom_rviz],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(rviz),
        )

        return [
            LogInfo(
                msg=(
                    "[moonmapper_nav2] ODOM baseline: Nav2 uten AMCL/map_server. "
                    f"params_file={params_resolved}  rviz_config={odom_rviz}"
                )
            ),
            depth_to_scan,
            bringup,
            safety,
            rviz_node,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 params (default: nav2_params_odom_baseline.yaml).",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz med moonmapper_nav2_odom_baseline.rviz (Fixed Frame: odom).",
            ),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.55"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            OpaqueFunction(function=impl),
        ]
    )
