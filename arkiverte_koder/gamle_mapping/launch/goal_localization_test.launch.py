"""Sim separat. Localization + depth + safety + simple_goal_follower (ingen reactive, ingen Nav2)."""

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

from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def _goal_localization_setup(context, *args, **kwargs):
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

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_localization.yaml")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moonmapper_autonomy"),
                "launch",
                "depth_scan_safety.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("front_stop_distance", LaunchConfiguration("safety_front_stop_distance")),
            ("front_angle_deg", LaunchConfiguration("safety_front_angle_deg")),
            ("scan_timeout_sec", LaunchConfiguration("safety_scan_timeout_sec")),
            ("allow_reverse_when_blocked", LaunchConfiguration("safety_allow_reverse_when_blocked")),
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
                    msg="[moonmapper_mapping] goal_localization_test: slam_toolbox activate."
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
                "output_cmd_topic": "/cmd_vel_raw",
            },
        ],
    )

    return [
        LogInfo(
            msg=f"[moonmapper_mapping] goal_localization_test: map_file={map_path} "
            f"-> slam stem={slam_stem}"
        ),
        stack,
        slam,
        goal_follower,
        configure_event,
        activate_event,
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("moonmapper_mapping")
    default_map_yaml = os.path.join(pkg_share, "maps", "moonmapper_slam_map.yaml")
    rviz_cfg = os.path.join(pkg_share, "rviz", "moonmapper_localization.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_goal_test",
        output="screen",
        arguments=["-d", rviz_cfg],
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
                "safety_front_stop_distance",
                default_value="0.55",
                description="Safety front stopp-avstand (m).",
            ),
            DeclareLaunchArgument(
                "safety_front_angle_deg",
                default_value="45.0",
                description="Front-sektor for safety (grader).",
            ),
            DeclareLaunchArgument(
                "safety_scan_timeout_sec",
                default_value="0.6",
                description="Scan-timeout for safety i goal-test.",
            ),
            DeclareLaunchArgument(
                "safety_allow_reverse_when_blocked",
                default_value="true",
                description="Tillat negativ linear.x når hindring blokkerer fremover.",
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
                description="true: RViz med moonmapper_localization.rviz (2D Goal -> /goal_pose).",
            ),
            DeclareLaunchArgument("forward_speed", default_value="0.12"),
            DeclareLaunchArgument("min_forward_speed", default_value="0.025"),
            DeclareLaunchArgument("angular_speed", default_value="0.55"),
            DeclareLaunchArgument("slow_radius", default_value="0.70"),
            OpaqueFunction(function=_goal_localization_setup),
            rviz,
        ]
    )
