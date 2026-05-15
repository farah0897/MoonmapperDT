"""RTAB-Map SLAM for MoonMapper real robot (camera + wheel /odom, no visual odom)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    rtabmap_share = get_package_share_directory("rtabmap_launch")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    params_hint = os.path.join(nav2_pkg, "config", "rtabmap_real_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_id = LaunchConfiguration("frame_id")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    rviz = LaunchConfiguration("rtabmap_rviz")

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "frame_id": frame_id,
            "odom_frame_id": "odom",
            "approx_sync": "true",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "use_sim_time": use_sim_time,
            "map_always_update": "true",
            "rviz": rviz,
            "rtabmap_args": (
                "Odom/MinInliers:=3 Vis/MinInliers:=3 Odom/ResetCountdown:=1 "
                "Rtabmap/LoopThr:=0.5 Mem/NotLinkedNodesKept:=false"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/camera/depth/image_rect_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("rtabmap_rviz", default_value="false"),
            LogInfo(
                msg=(
                    f"[moonmapper_nav2] RTAB-Map ({params_hint}). "
                    "Clear ~/.ros/rtabmap.db for fresh map. RTAB publishes map->odom TF."
                )
            ),
            rtabmap,
        ]
    )
