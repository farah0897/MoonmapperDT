"""Top-level bringup: real-robot RTAB-Map + Nav2 (Phase 1–3)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    exploration = os.path.join(nav2_pkg, "launch", "nav2_rtabmap_exploration_real.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_initial_spin", default_value="false"),
            DeclareLaunchArgument("enable_frontier_explorer", default_value="false"),
            DeclareLaunchArgument("start_robot_driver", default_value="false"),
            DeclareLaunchArgument("nav2_rviz", default_value="false"),
            DeclareLaunchArgument("rtabmap_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(exploration),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_initial_spin": LaunchConfiguration("enable_initial_spin"),
                    "enable_frontier_explorer": LaunchConfiguration(
                        "enable_frontier_explorer"
                    ),
                    "start_robot_driver": LaunchConfiguration("start_robot_driver"),
                    "nav2_rviz": LaunchConfiguration("nav2_rviz"),
                    "rtabmap_rviz": LaunchConfiguration("rtabmap_rviz"),
                }.items(),
            ),
        ]
    )
