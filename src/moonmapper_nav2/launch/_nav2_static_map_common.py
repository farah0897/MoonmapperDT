"""Shared helpers for static-map Nav2 launches."""

from __future__ import annotations

import os
from typing import Any, List

from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def depth_to_scan_node(use_sim_time: LaunchConfiguration) -> Node:
    return Node(
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
                "output_frame_id": "depth_camera_optical_frame",
            },
        ],
    )


def safety_node(
    use_sim_time: LaunchConfiguration,
    safety_stop: LaunchConfiguration,
    safety_angle: LaunchConfiguration,
    safety_to: LaunchConfiguration,
    name: str = "safety_obstacle_nav2_static_map",
) -> Node:
    return Node(
        package="moonmapper_autonomy",
        executable="safety_obstacle_node",
        name=name,
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


def map_odom_identity_tf(use_sim_time: LaunchConfiguration) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_identity_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )


def tf_chain_logger(use_sim_time: LaunchConfiguration, period_sec: str = "5.0") -> Node:
    return Node(
        package="moonmapper_nav2",
        executable="nav2_tf_chain_logger_node",
        name="nav2_tf_chain_logger",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"period_sec": float(period_sec)},
        ],
    )


def rviz_node(use_sim_time: LaunchConfiguration, rviz_config: str, rviz: LaunchConfiguration) -> Node:
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2_static_map",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
    )


def log_header(mode: str, map_file: str, params_file: str) -> LogInfo:
    return LogInfo(
        msg=(
            f"[moonmapper_nav2] static map mode={mode}  map_file={map_file}  "
            f"params_file={params_file}"
        )
    )
