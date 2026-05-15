"""Static TF: base_link -> camera_link and optical frames (real robot)."""

from __future__ import annotations

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    height = LaunchConfiguration("camera_height_m")

    common = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "camera_height_m",
                default_value="0.27",
                description="Measured base_link -> camera_link z (m).",
            ),
            LogInfo(msg="[moonmapper_bringup] Publishing base_link -> camera_* static TFs"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_camera_link",
                arguments=["0", "0", height, "0", "0", "0", "base_link", "camera_link"],
                parameters=[common],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_camera_to_color_optical",
                arguments=[
                    "0",
                    "0",
                    "0",
                    str(_OPTICAL_RPY[0]),
                    str(_OPTICAL_RPY[1]),
                    str(_OPTICAL_RPY[2]),
                    "camera_link",
                    "camera_color_optical_frame",
                ],
                parameters=[common],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_camera_to_depth_optical",
                arguments=[
                    "0",
                    "0",
                    "0",
                    str(_OPTICAL_RPY[0]),
                    str(_OPTICAL_RPY[1]),
                    str(_OPTICAL_RPY[2]),
                    "camera_link",
                    "camera_depth_optical_frame",
                ],
                parameters=[common],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_camera_to_color_frame",
                arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_color_frame"],
                parameters=[common],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_camera_to_depth_frame",
                arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_depth_frame"],
                parameters=[common],
            ),
        ]
    )
