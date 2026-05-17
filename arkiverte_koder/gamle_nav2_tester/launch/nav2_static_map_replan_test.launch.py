"""Steg 4: replan rundt hindring — nav2_static_map (odom) + Gazebo-blokk + monitor.

Forutsetter sim (sim_rover_clean) allerede kjørende.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    desc_pkg = get_package_share_directory("moonmapper_description")
    launch_dir = os.path.join(nav2_pkg, "launch")
    block_sdf = os.path.join(desc_pkg, "models", "replan_test_block", "model.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    autostart_goal = LaunchConfiguration("autostart_goal")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_static_map.launch.py")),
        launch_arguments={
            "localization_mode": "odom",
            "use_sim_time": use_sim_time,
            "rviz": LaunchConfiguration("rviz"),
            "tf_log": LaunchConfiguration("tf_log"),
            "safety_stop_distance": safety_stop,
            "safety_front_angle_deg": safety_angle,
        }.items(),
    )

    spawn_block = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_replan_test_block",
        output="screen",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
            "-name",
            "replan_test_block",
            "-file",
            block_sdf,
            "-x",
            obstacle_x,
            "-y",
            obstacle_y,
            "-z",
            "0.25",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    monitor = Node(
        package="moonmapper_nav2",
        executable="nav2_replan_monitor_node",
        name="nav2_replan_monitor",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_yaw": 0.0,
                "autostart_goal": ParameterValue(autostart_goal, value_type=bool),
                "goal_delay_sec": 14.0,
                "log_period_sec": 1.0,
                "map_margin_m": 1.05,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="moon_arena"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("tf_log", default_value="true"),
            DeclareLaunchArgument(
                "obstacle_x",
                default_value="0.75",
                description="Blokk midt i korridor (robot ~0, mål ~2.0).",
            ),
            DeclareLaunchArgument("obstacle_y", default_value="0.0"),
            DeclareLaunchArgument(
                "goal_x",
                default_value="2.0",
                description="Mål bak hindring; gå rundt via ±y.",
            ),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument(
                "autostart_goal",
                default_value="true",
                description="Send NavigateToPose etter delay (ellers bruk RViz Nav2 Goal).",
            ),
            DeclareLaunchArgument(
                "safety_stop_distance",
                default_value="0.48",
                description="Kun front sektor — angular.z følger Nav2 (sving rundt).",
            ),
            DeclareLaunchArgument(
                "safety_front_angle_deg",
                default_value="38.0",
                description="Smal front-cone: sidepass når roboten vender skjevt.",
            ),
            LogInfo(
                msg=(
                    "[moonmapper_nav2] REPLAN TEST: spawn blokk + nav2_static_map (odom). "
                    "Ikke 2D Pose Estimate. RViz Fixed Frame=map."
                )
            ),
            TimerAction(period=3.0, actions=[spawn_block]),
            nav2_stack,
            monitor,
        ]
    )
