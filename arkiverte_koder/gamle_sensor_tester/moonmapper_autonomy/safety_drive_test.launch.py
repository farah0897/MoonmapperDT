"""Kjor safety_obstacle_node med utgang til /cmd_vel (via cmd_vel_odom_relay til diff_drive).

Bruk sammen med:
  ros2 launch moonmapper_bringup sim_rover_clean.launch.py

Terminal 3 (test):
  ros2 topic echo /scan --once
  ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.06}, angular: {z: 0.0}}"
  ros2 topic echo /cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="moonmapper_autonomy",
                executable="safety_obstacle_node",
                name="safety_obstacle_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "front_stop_distance": 0.65,
                        "front_angle_deg": 45.0,
                        "scan_timeout_sec": 0.5,
                        "input_cmd_topic": "/cmd_vel_raw",
                        "output_cmd_topic": "/cmd_vel",
                        "scan_topic": "/scan",
                        "allow_reverse_when_blocked": True,
                        "publish_safety_debug": True,
                    }
                ],
            ),
        ]
    )
