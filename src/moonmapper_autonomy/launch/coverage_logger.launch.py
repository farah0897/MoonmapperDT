"""Start coverage_logger_node (kun logging, styrer ikke robot)."""

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
                executable="coverage_logger_node",
                name="coverage_logger_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "odom_topic": "/odom",
                        "obstacle_state_topic": "/obstacle/current_state",
                        "cmd_vel_topic": "/cmd_vel",
                        "subscribe_optional_topics": True,
                        "grid_frame": "odom",
                        "grid_size_m": 10.0,
                        "resolution": 0.10,
                        "publish_rate_hz": 1.0,
                        "min_movement_for_distance": 0.005,
                        "mission_duration_sec": 120.0,
                        "stuck_check_window_sec": 10.0,
                        "stuck_min_displacement_m": 0.05,
                        "spin_turn_radius_m": 0.18,
                        "odom_use_best_effort_qos": True,
                    }
                ],
            ),
        ]
    )
