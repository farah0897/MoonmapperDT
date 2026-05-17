"""Start reactive_avoidance_node (/scan -> /cmd_vel_raw)."""

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
                executable="reactive_avoidance_node",
                name="reactive_avoidance_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "scan_topic": "/scan",
                        "output_cmd_topic": "/cmd_vel_raw",
                        "forward_speed": 0.055,
                        "slow_speed": 0.028,
                        "reverse_speed": -0.04,
                        "turn_speed": 0.68,
                        "slow_distance": 1.45,
                        "avoid_distance": 0.90,
                        "emergency_distance": 0.50,
                        "clear_distance": 0.95,
                        "clear_confirm_count": 3,
                        "stop_before_turn_time_sec": 0.3,
                        "backup_time_sec": 1.0,
                        "min_turn_time_sec": 1.4,
                        "max_turn_time_sec": 6.5,
                        "max_escape_attempts_same_direction": 0,
                        "default_turn_direction": "right",
                        "escape_forward_speed": 0.04,
                        "escape_forward_time_sec": 1.0,
                        "front_angle_deg": 45.0,
                        "side_angle_deg": 75.0,
                        "scan_timeout_sec": 0.5,
                        "control_rate_hz": 10.0,
                        "front_smooth_samples": 5,
                        "publish_obstacle_debug": True,
                    }
                ],
            ),
        ]
    )
