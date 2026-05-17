"""Waypoint-misjon + samme stack som goal_obstacle_localization_test (ingen Nav2)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("moonmapper_mapping")
    default_map_yaml = os.path.join(pkg_share, "maps", "moonmapper_slam_map.yaml")
    default_waypoints = os.path.join(pkg_share, "config", "missions", "test_waypoints.yaml")

    goal_stack = os.path.join(pkg_share, "launch", "goal_obstacle_localization_test.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    rviz = LaunchConfiguration("rviz")
    waypoints_file = LaunchConfiguration("waypoints_file")
    auto_start = LaunchConfiguration("auto_start")
    loop = LaunchConfiguration("loop")
    forward_speed = LaunchConfiguration("forward_speed")
    min_forward_speed = LaunchConfiguration("min_forward_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    slow_radius = LaunchConfiguration("slow_radius")
    safety_stop_distance = LaunchConfiguration("safety_stop_distance")
    safety_front_angle_deg = LaunchConfiguration("safety_front_angle_deg")
    safety_scan_timeout_sec = LaunchConfiguration("safety_scan_timeout_sec")
    avoid_start_distance = LaunchConfiguration("avoid_start_distance")
    avoid_clear_distance = LaunchConfiguration("avoid_clear_distance")
    avoid_arc_forward_speed = LaunchConfiguration("avoid_arc_forward_speed")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(goal_stack),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("map_file", map_file),
            ("autostart", autostart),
            ("use_lifecycle_manager", use_lifecycle_manager),
            ("rviz", rviz),
            ("forward_speed", forward_speed),
            ("min_forward_speed", min_forward_speed),
            ("angular_speed", angular_speed),
            ("slow_radius", slow_radius),
            ("safety_stop_distance", safety_stop_distance),
            ("safety_front_angle_deg", safety_front_angle_deg),
            ("safety_scan_timeout_sec", safety_scan_timeout_sec),
            ("avoid_start_distance", avoid_start_distance),
            ("avoid_clear_distance", avoid_clear_distance),
            ("avoid_arc_forward_speed", avoid_arc_forward_speed),
        ],
    )

    waypoint_mission = Node(
        package="moonmapper_mapping",
        executable="waypoint_mission_node",
        name="waypoint_mission_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "waypoints_file": waypoints_file,
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "loop": ParameterValue(loop, value_type=bool),
                "goal_topic": "/goal_pose",
                "goal_frame": "map",
                "goal_state_topic": "/goal_follower/state",
                "distance_topic": "/goal_follower/distance_to_goal",
                "reached_state_name": "REACHED",
                "idle_state_name": "IDLE",
                "wait_after_reached_sec": 1.0,
                "goal_publish_period_sec": 1.0,
                "mission_rate_hz": 10.0,
                "goal_timeout_sec": 90.0,
                "max_retries_per_waypoint": 1,
                "publish_debug": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Synkroniser med Gazebo /clock.",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=default_map_yaml,
                description="Full sti til kart-.yaml (posegraf).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Lifecycle for slam_toolbox (videreføres til goal_obstacle-stack).",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="Videreføres til goal_obstacle_localization_test.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Videreføres til goal_obstacle_localization_test.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints,
                description="YAML med waypoints-liste.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Start waypoint-misjon automatisk når noden kjører.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Gjenta misjonen fra første waypoint.",
            ),
            DeclareLaunchArgument("forward_speed", default_value="0.12"),
            DeclareLaunchArgument("min_forward_speed", default_value="0.025"),
            DeclareLaunchArgument("angular_speed", default_value="0.55"),
            DeclareLaunchArgument("slow_radius", default_value="0.70"),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.55"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            DeclareLaunchArgument("avoid_start_distance", default_value="0.90"),
            DeclareLaunchArgument("avoid_clear_distance", default_value="1.20"),
            DeclareLaunchArgument("avoid_arc_forward_speed", default_value="0.04"),
            stack,
            waypoint_mission,
        ]
    )
