"""Intel RealSense D435 — RGB + depth; enables point cloud after startup."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"enable_sync": True},
            {"depth_module.depth_profile": "640x480x60"},
            {"rgb_camera.color_profile": "640x480x60"},
        ],
    )

    # Parameter name varies by realsense2_camera version (Humble vs Jazzy).
    enable_pc = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "param",
                    "set",
                    "/camera/camera",
                    "pointcloud.enable",
                    "true",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            LogInfo(msg="[moonmapper_bringup] RealSense D435 — wait for 'RealSense Node Is Up!'"),
            realsense,
            enable_pc,
        ]
    )
