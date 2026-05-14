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
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.026",
            description="base_footprint spawn-z over regolith (z=0).",
        ),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([desc_pkg, "launch", "gazebo_rover.launch.py"])
            ),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "enable_diff_plugin": LaunchConfiguration("enable_diff_plugin"),
                "use_ekf": "false",
            }.items(),
        ),
    ])
