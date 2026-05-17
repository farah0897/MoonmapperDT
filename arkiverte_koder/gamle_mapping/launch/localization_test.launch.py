"""Sim startes separat: depth+reactive+safety + slam_toolbox localization (posegraph).

map_file kan være full sti til OccupancyGrid .yaml (anbefalt) eller sti uten suffiks
som peker på serialisert kart (samme stam som ved save_current_map.sh).
"""

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


def _localization_setup(context, *args, **kwargs):
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
                "\nFant .yaml, men ikke posegraf. Da er kartet bare lagret som OccupancyGrid "
                "(map_saver), ikke med slam_toolbox serialize_map.\n"
                "Kjør hele save_current_map.sh mens async_slam_toolbox (mapping) fortsatt kjører "
                "(Terminal 2 autonomy_slam_test), så opprettes .posegraph og .data.\n"
            )
        elif yaml_hint is not None and not yaml_exists:
            extra = f"\nFant heller ikke yaml-filen: {yaml_hint}\n"

        raise RuntimeError(
            "slam_toolbox localization krever serialisert posegraf (ikke bare .yaml/.pgm).\n"
            f"  Forventet: {posegraph}\n  og:        {pdata}\n"
            f"  map_file:  {map_path}\n"
            "  Stam:      " + slam_stem + "\n"
            f"{extra}"
            "Sjekk at filene finnes: ls -la \""
            + str(pathlib.Path(slam_stem).parent)
            + '\"\n'
            "Lagre kart (mapping + Terminal 3): bash \"$(ros2 pkg prefix moonmapper_mapping)/lib/"
            "moonmapper_mapping/save_current_map.sh\" moonmapper_test_map"
        )

    use_sim = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    use_lifecycle_manager = LaunchConfiguration(
        "use_lifecycle_manager"
    ).perform(context).lower() in ("true", "1", "yes")

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_localization.yaml")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moonmapper_autonomy"),
                "launch",
                "depth_reactive_avoidance.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
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
                    msg=f"[moonmapper_mapping] slam_toolbox localization: "
                    f"activate (map_file_name stem={slam_stem})."
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

    return [
        LogInfo(
            msg=f"[moonmapper_mapping] localization_test: map_file={map_path} "
            f"-> slam map_file_name (stem)={slam_stem}"
        ),
        stack,
        slam,
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
        name="rviz2_localization",
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
                description="Full sti til .yaml i .../maps/ (ikke rviz/). Krever .posegraph + .data samme stam.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Lifecycle: configure + activate automatisk.",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="false: bruk EmitEvent/RegisterEventHandler som slam_toolbox upstream.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="true: start RViz (Fixed Frame map, moonmapper_localization.rviz).",
            ),
            OpaqueFunction(function=_localization_setup),
            rviz,
        ]
    )
