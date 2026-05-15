"""map_server + AMCL with launch-time initial pose (RewrittenYaml)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    map_pkg = get_package_share_directory("moonmapper_mapping")
    default_map = os.path.join(map_pkg, "maps", "moonmapper_test_map_padded.yaml")
    default_params = os.path.join(nav2_pkg, "config", "nav2_params_amcl_static_map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    use_respawn = LaunchConfiguration("use_respawn")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "amcl.set_initial_pose": True,
                "amcl.initial_pose_x": initial_x,
                "amcl.initial_pose_y": initial_y,
                "amcl.initial_pose_z": 0.0,
                "amcl.initial_pose_yaw": initial_yaw,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {"yaml_filename": map_yaml_file}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
        condition=IfCondition(NotEqualsSubstitution(map_yaml_file, "")),
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"autostart": autostart}, {"node_names": ["map_server", "amcl"]}],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            map_server,
            amcl,
            lifecycle,
        ]
    )
