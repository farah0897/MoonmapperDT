"""Minimal Gazebo-sim: robot_state_publisher, Gazebo, spawn, ros2_control, sensorbroer.

Starter ikke Nav2, map_server, EKF, UWB eller navigasjonsstack.

Anbefalt:
  ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    desc_pkg = FindPackageShare("moonmapper_description")
    default_world = PathJoinSubstitution([desc_pkg, "worlds", "moon_arena.sdf"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_preset",
            default_value="moon",
            description=(
                "moon → moon_arena + safe_6wd. earth → earth_arena + earth_stable_6wd. "
                "See gazebo_rover.launch.py for logged gravity/friction."
            ),
        ),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.026",
            description="base_footprint spawn-z (overstyres av physics_profile spawn_z).",
        ),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true"),
        DeclareLaunchArgument(
            "physics_profile",
            default_value="",
            description=(
                "Gazebo preset: safe_6wd | earth_stable_6wd | earth_6wd | realistic_6wd | debug_low_friction. "
                "Tom = world_preset velger (moon→safe_6wd, earth→earth_stable_6wd)."
            ),
        ),
        DeclareLaunchArgument(
            "simple_collision_debug",
            default_value="true",
            description=(
                "Used when physics_profile is empty. With safe_6wd profile this is set true."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([desc_pkg, "launch", "gazebo_rover.launch.py"])
            ),
            launch_arguments={
                "world_preset": LaunchConfiguration("world_preset"),
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "enable_diff_plugin": LaunchConfiguration("enable_diff_plugin"),
                "physics_profile": LaunchConfiguration("physics_profile"),
                "simple_collision_debug": LaunchConfiguration("simple_collision_debug"),
                "use_ekf": "false",
            }.items(),
        ),
    ])
