"""Shared helpers for RTAB-Map + Nav2 (no static map_server, no identity map->odom)."""

from __future__ import annotations

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def depth_to_scan_real(
    use_sim_time: LaunchConfiguration, _unused: LaunchConfiguration
) -> Node:
    """depth_to_scan from launch-configured depth/camera_info topics."""
    return Node(
        package="moonmapper_autonomy",
        executable="depth_to_scan_node",
        name="depth_to_scan_rtabmap",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "depth_image_topic": ParameterValue(
                    LaunchConfiguration("depth_image_topic"), value_type=str
                ),
                "camera_info_topic": ParameterValue(
                    LaunchConfiguration("camera_info_topic"), value_type=str
                ),
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
                "range_max": 3.5,
                "scan_time": 0.1,
                "output_frame_id": "",
            },
        ],
    )


def safety_node_rtabmap(
    use_sim_time: LaunchConfiguration,
    safety_stop: LaunchConfiguration,
    safety_angle: LaunchConfiguration,
    safety_to: LaunchConfiguration,
) -> Node:
    return Node(
        package="moonmapper_autonomy",
        executable="safety_obstacle_node",
        name="safety_obstacle_nav2_rtabmap",
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
                "debug_log_period_sec": 2.0,
            },
        ],
    )


def map_relay_node() -> Node:
    return Node(
        package="topic_tools",
        executable="relay",
        name="rtabmap_map_to_nav2_map",
        output="screen",
        arguments=["/rtabmap/map", "/map"],
        remappings=[],
    )
