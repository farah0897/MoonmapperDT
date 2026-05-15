"""Coverage lawnmower + samme stack som goal_obstacle_localization_test (ingen Nav2, ingen waypoint_node)."""

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
    coverage_rviz = os.path.join(pkg_share, "rviz", "moonmapper_coverage_mission.rviz")

    goal_stack = os.path.join(pkg_share, "launch", "goal_obstacle_localization_test.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    rviz = LaunchConfiguration("rviz")
    auto_start = LaunchConfiguration("auto_start")
    loop = LaunchConfiguration("loop")
    area_min_x = LaunchConfiguration("area_min_x")
    area_max_x = LaunchConfiguration("area_max_x")
    area_min_y = LaunchConfiguration("area_min_y")
    area_max_y = LaunchConfiguration("area_max_y")
    lane_spacing = LaunchConfiguration("lane_spacing")
    start_from_min_y = LaunchConfiguration("start_from_min_y")
    start_from_min_x = LaunchConfiguration("start_from_min_x")
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
    coverage_reached_tolerance = LaunchConfiguration("coverage_reached_tolerance")
    waypoint_hold_time = LaunchConfiguration("waypoint_hold_time")
    publish_interval = LaunchConfiguration("publish_interval")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(goal_stack),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("map_file", map_file),
            ("autostart", autostart),
            ("use_lifecycle_manager", use_lifecycle_manager),
            ("rviz", rviz),
            ("rviz_config", coverage_rviz),
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

    coverage_mission = Node(
        package="moonmapper_mapping",
        executable="coverage_mission_node",
        name="coverage_mission_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "loop": ParameterValue(loop, value_type=bool),
                "goal_topic": "/goal_pose",
                "goal_frame": "map",
                "goal_state_topic": "/goal_follower/state",
                "distance_topic": "/goal_follower/distance_to_goal",
                "area_min_x": ParameterValue(area_min_x, value_type=float),
                "area_max_x": ParameterValue(area_max_x, value_type=float),
                "area_min_y": ParameterValue(area_min_y, value_type=float),
                "area_max_y": ParameterValue(area_max_y, value_type=float),
                "lane_spacing": ParameterValue(lane_spacing, value_type=float),
                "start_from_min_y": ParameterValue(start_from_min_y, value_type=bool),
                "start_from_min_x": ParameterValue(start_from_min_x, value_type=bool),
                "yaw_forward": 3.1416,
                "yaw_backward": 0.0,
                "wait_after_reached_sec": ParameterValue(waypoint_hold_time, value_type=float),
                "goal_publish_period_sec": ParameterValue(publish_interval, value_type=float),
                "mission_rate_hz": 10.0,
                "goal_timeout_sec": 120.0,
                "max_retries_per_waypoint": 1,
                "reached_state_name": "REACHED",
                "idle_state_name": "IDLE",
                "publish_debug": True,
                "coverage_reached_tolerance": ParameterValue(coverage_reached_tolerance, value_type=float),
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
                description="Lifecycle for slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="Videreføres til goal_obstacle_localization_test.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="true: RViz med coverage-path (moonmapper_coverage_mission.rviz).",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Start coverage-misjon automatisk.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Gjenta coverage fra første punkt.",
            ),
            DeclareLaunchArgument("area_min_x", default_value="-4.0", description="Rektangel min x."),
            DeclareLaunchArgument("area_max_x", default_value="-1.0", description="Rektangel max x."),
            DeclareLaunchArgument("area_min_y", default_value="-0.8", description="Rektangel min y."),
            DeclareLaunchArgument("area_max_y", default_value="0.8", description="Rektangel max y."),
            DeclareLaunchArgument("lane_spacing", default_value="0.5", description="Avstand mellom striper (y)."),
            DeclareLaunchArgument(
                "start_from_min_y",
                default_value="true",
                description="Stripe-rekkefølge starter på min y (ellers fra max y).",
            ),
            DeclareLaunchArgument(
                "start_from_min_x",
                default_value="false",
                description="Første stripe: false = start ved max x (mot min x), true = start ved min x.",
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
            DeclareLaunchArgument(
                "coverage_reached_tolerance",
                default_value="0.18",
                description="Backup-ankomst hvis REACHED-tikk mistes (m).",
            ),
            DeclareLaunchArgument(
                "waypoint_hold_time",
                default_value="0.8",
                description="Pause etter REACHED (s) → coverage wait_after_reached_sec.",
            ),
            DeclareLaunchArgument(
                "publish_interval",
                default_value="1.0",
                description="Republiser goal (s) → goal_publish_period_sec.",
            ),
            stack,
            coverage_mission,
        ]
    )
