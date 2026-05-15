"""RViz2 med moonmapper_slam.rviz (Steg 4: /scan, /map, /coverage/grid, TF).

Kjør etter sim + autonomi/SLAM er oppe, eller sammen med autonomy_slam_test (rviz:=true).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_cfg = os.path.join(
        get_package_share_directory("moonmapper_mapping"),
        "rviz",
        "moonmapper_slam.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            rviz,
        ]
    )
