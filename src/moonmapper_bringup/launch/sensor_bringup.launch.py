"""
Level 1 sensor bringup for Gazebo Sim:
  - Launches the MoonMapper Gazebo rover wrapper (IMU + stereo + RGB-D + microscope)
  - Triad spectroscopy sensors are provided by a Gazebo system plugin (no extra ROS node)

Usage:
  ros2 launch moonmapper_bringup sensor_bringup.launch.py
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
        # Roveren er ~5 cm; 0.05 m er et tryggere default-spawn for å unngå "flytende" start.
        DeclareLaunchArgument("spawn_z", default_value="0.05"),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([desc_pkg, "launch", "gazebo_rover.launch.py"])
            ),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "enable_diff_plugin": LaunchConfiguration("enable_diff_plugin"),
            }.items(),
        ),
    ])

