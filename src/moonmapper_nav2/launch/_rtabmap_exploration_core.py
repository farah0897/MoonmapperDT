"""Shared RTAB exploration launch builder (sim / real)."""

from __future__ import annotations

import os
import sys
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
import _nav2_rtabmap_common as _common  # noqa: E402


def _build_stack_actions(context: LaunchContext, mode: str) -> List:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    bringup_pkg = get_package_share_directory("moonmapper_bringup")
    launch_dir = os.path.join(nav2_pkg, "launch")
    bringup_launch_dir = os.path.join(bringup_pkg, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_initial_spin = LaunchConfiguration("enable_initial_spin")
    enable_map_wait = LaunchConfiguration("enable_map_ready_wait")
    enable_frontier = LaunchConfiguration("enable_frontier_explorer").perform(context).lower()

    if mode == "sim":
        params = os.path.join(nav2_pkg, "config", "nav2_params_rtabmap_sim.yaml")
        rtabmap_launch = os.path.join(launch_dir, "rtabmap_sim.launch.py")
        base_frame = "base_footprint"
        depth_topic = "/depth_camera/depth_image"
        camera_info_topic = "/depth_camera/camera_info"
        rgb_topic = "/depth_camera/image"
        frame_id = "base_footprint"
        frontier_params = os.path.join(
            nav2_pkg, "config", "frontier_params_moonmapper_sim.yaml"
        )
    else:
        params = os.path.join(nav2_pkg, "config", "nav2_params_rtabmap_real.yaml")
        rtabmap_launch = os.path.join(launch_dir, "rtabmap_real.launch.py")
        base_frame = "base_link"
        depth_topic = "/camera/camera/depth/image_rect_raw"
        camera_info_topic = "/camera/camera/color/camera_info"
        rgb_topic = "/camera/camera/color/image_raw"
        frame_id = "base_link"
        frontier_params = os.path.join(nav2_pkg, "config", "frontier_params_moonmapper.yaml")

    actions: List = [
        LogInfo(msg=f"[moonmapper_nav2] RTAB+Nav2 stack starting (mode={mode})"),
    ]

    if mode == "real":
        actions.append(
            ExecuteProcess(
                cmd=["python3", LaunchConfiguration("robot_script")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_robot_driver")),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_launch_dir, "realsense_d435.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_realsense")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_launch_dir, "camera_static_tf.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_camera_tf")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "camera_height_m": LaunchConfiguration("camera_height_m"),
                }.items(),
            )
        )

    actions.append(
        Node(
            package="moonmapper_nav2",
            executable="nav2_map_ready_wait_node",
            name="nav2_map_ready_wait",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {"map_topic": "/map", "base_frame": base_frame},
            ],
            condition=IfCondition(enable_map_wait),
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "frame_id": frame_id,
                "rgb_topic": rgb_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "rtabmap_rviz": LaunchConfiguration("rtabmap_rviz"),
            }.items(),
        )
    )
    actions.append(TimerAction(period=2.0, actions=[_common.map_relay_node()]))
    actions.append(
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "nav2_rtabmap_navigation.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "params_file": params,
                        "rviz": LaunchConfiguration("nav2_rviz"),
                        "depth_image_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                    }.items(),
                )
            ],
        )
    )
    actions.append(
        Node(
            package="moonmapper_nav2",
            executable="nav2_initial_spin_node",
            name="nav2_initial_spin",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {
                    "wait_for_map_ready": True,
                    "spin_duration_sec": 22.0,
                    "angular_z": 0.3,
                    "start_delay_sec": 8.0,
                },
            ],
            condition=IfCondition(enable_initial_spin),
        )
    )

    if enable_frontier in ("true", "1", "yes"):
        try:
            fe_share = get_package_share_directory("frontier_exploration_ros2")
            actions.append(
                TimerAction(
                    period=90.0,
                    actions=[
                        LogInfo(
                            msg=(
                                "Frontier explorer (Phase 3): start only after manual "
                                "Nav2 Goal works; needs /rtabmap/map, /map, map->odom TF."
                            )
                        ),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(fe_share, "launch", "frontier_explorer.launch.py")
                            ),
                            launch_arguments={"params_file": frontier_params}.items(),
                        ),
                    ],
                )
            )
        except Exception:
            actions.append(
                LogInfo(msg="frontier_exploration_ros2 not installed — skipping Phase 3")
            )

    return actions


def _on_preflight_exit(event, context: LaunchContext, mode: str):
    if event.returncode != 0:
        return [
            LogInfo(
                msg=(
                    f"[moonmapper_nav2] Preflight failed (exit {event.returncode}). "
                    "Ensure sim_rover_clean is running with /depth_camera/* and /odom. "
                    "RTAB/Nav2 will NOT start."
                )
            ),
        ]
    return _build_stack_actions(context, mode)


def build_rtabmap_exploration(mode: str) -> LaunchDescription:
    use_sim_time_default = "true" if mode == "sim" else "false"
    skip_preflight = LaunchConfiguration("skip_preflight")

    preflight_cmd = ["ros2", "run", "moonmapper_nav2", "nav2_rtabmap_preflight", mode]
    if mode == "sim":
        preflight_cmd.extend(["--ros-args", "-p", "use_sim_time:=true"])

    preflight = ExecuteProcess(
        cmd=preflight_cmd,
        output="screen",
        condition=UnlessCondition(skip_preflight),
    )

    decls: List = [
        DeclareLaunchArgument(
            "skip_preflight",
            default_value="false",
            description="Skip blocking preflight (debug only; sim must already be up).",
        ),
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time_default),
        DeclareLaunchArgument("enable_map_ready_wait", default_value="true"),
        DeclareLaunchArgument("enable_initial_spin", default_value="false"),
        DeclareLaunchArgument("enable_frontier_explorer", default_value="false"),
        DeclareLaunchArgument("nav2_rviz", default_value="false"),
        DeclareLaunchArgument("rtabmap_rviz", default_value="false"),
        LogInfo(
            msg=(
                "Start FIRST (separate terminal): "
                + (
                    "ros2 launch moonmapper_bringup sim_rover_clean.launch.py"
                    if mode == "sim"
                    else "python3 ~/robot.py"
                )
            )
        ),
        LogInfo(
            msg=(
                f"[moonmapper_nav2] RTAB exploration mode={mode}: "
                "NO map_server/AMCL/identity map->odom"
            )
        ),
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=lambda ctx: _build_stack_actions(ctx, mode))],
            condition=IfCondition(skip_preflight),
        ),
        preflight,
        RegisterEventHandler(
            OnProcessExit(
                target_action=preflight,
                on_exit=lambda event, context: _on_preflight_exit(event, context, mode),
            ),
            condition=UnlessCondition(skip_preflight),
        ),
    ]

    if mode == "real":
        decls.extend(
            [
                DeclareLaunchArgument("start_realsense", default_value="true"),
                DeclareLaunchArgument("start_camera_tf", default_value="true"),
                DeclareLaunchArgument("start_robot_driver", default_value="false"),
                DeclareLaunchArgument(
                    "robot_script", default_value=os.path.expanduser("~/robot.py")
                ),
                DeclareLaunchArgument("camera_height_m", default_value="0.27"),
            ]
        )

    return LaunchDescription(decls)
