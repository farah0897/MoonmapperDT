"""Launch safety_obstacle_node (cmd_vel_raw + scan -> cmd_vel_safe)."""

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
                        "front_stop_distance": 0.35,
                        "front_angle_deg": 35.0,
                        "scan_timeout_sec": 0.5,
                        "input_cmd_topic": "/cmd_vel_raw",
                        "output_cmd_topic": "/cmd_vel_safe",
                        "scan_topic": "/scan",
                        "allow_reverse_when_blocked": True,
                        "publish_safety_debug": True,
                    }
                ],
            ),
        ]
    )
