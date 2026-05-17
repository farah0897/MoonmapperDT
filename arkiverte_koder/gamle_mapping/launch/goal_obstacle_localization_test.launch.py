"""Localization + goal + scan-unnamanøvre + safety (ingen reactive_avoidance, ingen Nav2)."""

from __future__ import annotations

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _slam_map_stem_from_map_file(map_file: str) -> str:
    p = pathlib.Path(map_file).expanduser()
    if not p.is_absolute():
        p = (pathlib.Path.cwd() / p).resolve()
    else:
        p = p.resolve()
    s = str(p)
    lower = s.lower()
    if lower.endswith((".yaml", ".yml")):
        return str(p.with_suffix(""))
    return s


def _goal_obstacle_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("moonmapper_mapping")
    default_map_yaml = os.path.join(pkg_share, "maps", "moonmapper_slam_map.yaml")
    map_path = LaunchConfiguration("map_file").perform(context).strip() or default_map_yaml
    slam_stem = _slam_map_stem_from_map_file(map_path)

    posegraph = pathlib.Path(slam_stem + ".posegraph")
    pdata = pathlib.Path(slam_stem + ".data")
    map_p = pathlib.Path(map_path).expanduser()
    if not map_p.is_absolute():
        map_p = (pathlib.Path.cwd() / map_p).resolve()
    else:
        map_p = map_p.resolve()
    yaml_hint = map_p if str(map_p).lower().endswith((".yaml", ".yml")) else None

    if not posegraph.is_file() or not pdata.is_file():
        yaml_exists = yaml_hint.is_file() if yaml_hint is not None else False
        extra = ""
        if yaml_exists:
            extra = (
                "\nFant .yaml, men ikke posegraf. Kjør save_current_map.sh under mapping.\n"
            )
        elif yaml_hint is not None and not yaml_exists:
            extra = f"\nFant ikke yaml: {yaml_hint}\n"
        raise RuntimeError(
            "Posegraf mangler for localization.\n"
            f"  Forventet: {posegraph}\n  og:        {pdata}\n"
            f"  map_file:  {map_path}\n"
            f"{extra}"
        )

    use_sim = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    use_lifecycle_manager = LaunchConfiguration(
        "use_lifecycle_manager"
    ).perform(context).lower() in ("true", "1", "yes")

    def _pf(name: str, default: float) -> float:
        txt = LaunchConfiguration(name).perform(context).strip()
        if not txt:
            return default
        try:
            return float(txt)
        except ValueError:
            return default

    forward_speed = _pf("forward_speed", 0.12)
    min_fwd = _pf("min_forward_speed", 0.025)
    angular_speed = _pf("angular_speed", 0.55)
    slow_r = _pf("slow_radius", 0.70)
    safety_stop = _pf("safety_stop_distance", 0.55)
    safety_angle = _pf("safety_front_angle_deg", 45.0)
    safety_to = _pf("safety_scan_timeout_sec", 0.6)
    avoid_start = _pf("avoid_start_distance", 0.90)
    avoid_clear = _pf("avoid_clear_distance", 1.20)
    turn_clear = max(avoid_start + 0.05, avoid_clear - 0.15)
    turn_avoid = max(0.45, min(0.55, angular_speed + 0.05))
    arc_fwd = _pf("avoid_arc_forward_speed", 0.04)
    cmd_lin_cap = max(forward_speed, 0.12)
    cmd_ang_cap = max(angular_speed, 0.45)

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_localization.yaml")

    depth_to_scan = Node(
        package="moonmapper_autonomy",
        executable="depth_to_scan_node",
        name="depth_to_scan_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim},
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
                "output_frame_id": "",
            },
        ],
    )

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim,
                "use_lifecycle_manager": use_lifecycle_manager,
                "map_file_name": slam_stem,
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("autostart"),
                NotSubstitution(LaunchConfiguration("use_lifecycle_manager")),
            )
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="[moonmapper_mapping] goal_obstacle_localization_test: slam_toolbox activate."
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("autostart"),
                NotSubstitution(LaunchConfiguration("use_lifecycle_manager")),
            )
        ),
    )

    goal_follower = Node(
        package="moonmapper_autonomy",
        executable="simple_goal_follower_node",
        name="simple_goal_follower_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim},
            {
                "output_cmd_topic": "/cmd_vel_goal",
                "base_forward_yaw_offset_rad": 3.14159265,
                "linear_sign": 1.0,
                "angular_sign": 1.0,
                "rotate_enter_angle_rad": 0.45,
                "rotate_exit_angle_rad": 0.22,
                "drive_stop_angle_rad": 0.55,
                "goal_tolerance_m": 0.15,
                "max_forward_speed": forward_speed,
                "min_forward_speed": min_fwd,
                "max_turn_speed": angular_speed,
                "rotate_angular_speed": angular_speed,
                "yaw_deadband_rad": 0.04,
                "yaw_kp": 0.8,
                "distance_slow_radius": slow_r,
                "control_rate_hz": 20.0,
            },
        ],
    )

    goal_avoid = Node(
        package="moonmapper_autonomy",
        executable="goal_obstacle_avoidance_node",
        name="goal_obstacle_avoidance_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim},
            {
                "input_cmd_topic": "/cmd_vel_goal",
                "output_cmd_topic": "/cmd_vel_raw",
                "scan_topic": "/scan",
                "control_rate_hz": 20.0,
                "scan_timeout_sec": 0.6,
                "front_angle_deg": 45.0,
                "side_angle_deg": 85.0,
                "avoid_enter_distance": avoid_start,
                "turn_clear_distance": turn_clear,
                "avoid_clear_distance": avoid_clear,
                "emergency_distance": 0.40,
                "brake_time_sec": 0.4,
                "min_turn_time_sec": 1.0,
                "max_turn_time_sec": 4.0,
                "min_arc_time_sec": 1.5,
                "max_arc_time_sec": 5.0,
                "recover_time_sec": 0.8,
                "turn_speed": turn_avoid,
                "arc_turn_speed": min(0.40, angular_speed + 0.05),
                "arc_forward_speed": arc_fwd,
                "cmd_angular_limit": cmd_ang_cap,
                "cmd_linear_limit": cmd_lin_cap,
                "side_hysteresis_sec": 1.2,
                "side_switch_margin_m": 0.25,
                "prefer_last_turn": True,
                "publish_debug": True,
            },
        ],
    )

    safety = Node(
        package="moonmapper_autonomy",
        executable="safety_obstacle_node",
        name="safety_obstacle_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim},
            {
                "input_cmd_topic": "/cmd_vel_raw",
                "output_cmd_topic": "/cmd_vel",
                "scan_topic": "/scan",
                "front_stop_distance": safety_stop,
                "front_angle_deg": safety_angle,
                "scan_timeout_sec": safety_to,
                "allow_reverse_when_blocked": True,
                "publish_safety_debug": True,
            },
        ],
    )

    return [
        LogInfo(
            msg=f"[moonmapper_mapping] goal_obstacle_localization_test: map_file={map_path} "
            f"-> slam stem={slam_stem}"
        ),
        depth_to_scan,
        slam,
        goal_follower,
        goal_avoid,
        safety,
        configure_event,
        activate_event,
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("moonmapper_mapping")
    default_map_yaml = os.path.join(pkg_share, "maps", "moonmapper_slam_map.yaml")
    rviz_cfg_default = os.path.join(pkg_share, "rviz", "moonmapper_localization.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_goal_obstacle_test",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
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
                description="Full sti til .yaml i maps/ (krever .posegraph + .data).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Lifecycle: configure + activate slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="false: EmitEvent/RegisterEventHandler for slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="true: RViz med rviz_config (standard: moonmapper_localization.rviz).",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_cfg_default,
                description="RViz-konfig (-d).",
            ),
            DeclareLaunchArgument(
                "forward_speed",
                default_value="0.12",
                description="simple_goal_follower: max_forward_speed (m/s).",
            ),
            DeclareLaunchArgument(
                "min_forward_speed",
                default_value="0.025",
                description="simple_goal_follower: min_forward_speed (m/s).",
            ),
            DeclareLaunchArgument(
                "angular_speed",
                default_value="0.55",
                description="simple_goal_follower max_turn + rotate cap; goal_avoid turn (rad/s).",
            ),
            DeclareLaunchArgument(
                "slow_radius",
                default_value="0.70",
                description="simple_goal_follower: distance_slow_radius (m).",
            ),
            DeclareLaunchArgument(
                "safety_stop_distance",
                default_value="0.55",
                description="safety_obstacle_node: front_stop_distance (m).",
            ),
            DeclareLaunchArgument(
                "safety_front_angle_deg",
                default_value="45.0",
                description="safety_obstacle_node: front sektor halvbredde (grader).",
            ),
            DeclareLaunchArgument(
                "safety_scan_timeout_sec",
                default_value="0.6",
                description="safety_obstacle_node: scan-timeout (s).",
            ),
            DeclareLaunchArgument(
                "avoid_start_distance",
                default_value="0.90",
                description="goal_obstacle_avoidance: start unnamanøver (front_min, m).",
            ),
            DeclareLaunchArgument(
                "avoid_clear_distance",
                default_value="1.20",
                description="goal_obstacle_avoidance: ARC ferdig når front > dette (m).",
            ),
            DeclareLaunchArgument(
                "avoid_arc_forward_speed",
                default_value="0.04",
                description="goal_obstacle_avoidance: arc_forward_speed (m/s).",
            ),
            OpaqueFunction(function=_goal_obstacle_setup),
            rviz,
        ]
    )
