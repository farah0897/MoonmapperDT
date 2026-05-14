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
        # spawn_z: se DeclareLaunchArgument under (juster mot regolith z=0).
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.026",
            description=(
                "base_footprint spawn-z; med URDF base_link ~0,124 m over footprint tilsvarer ~0,026 m "
                "tidligere default 0,15 m for samme hjul-høyde."
            ),
        ),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true"),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description="Videresendes til gazebo_rover (robot_localization EKF).",
        ),
        DeclareLaunchArgument(
            "ekf_publish_tf",
            default_value="true",
            description="Når use_ekf: EKF publish_tf i gazebo_rover.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([desc_pkg, "launch", "gazebo_rover.launch.py"])
            ),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "enable_diff_plugin": LaunchConfiguration("enable_diff_plugin"),
                "use_ekf": LaunchConfiguration("use_ekf"),
                "ekf_publish_tf": LaunchConfiguration("ekf_publish_tf"),
            }.items(),
        ),
    ])

