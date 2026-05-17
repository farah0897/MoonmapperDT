"""Depth -> /scan -> reactive_avoidance -> /cmd_vel_raw -> safety -> /cmd_vel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    depth_to_scan = Node(
        package="moonmapper_autonomy",
        executable="depth_to_scan_node",
        name="depth_to_scan_node",
        output="screen",
        parameters=[
            common_time,
            {
                "depth_image_topic": "/depth_camera/depth_image",
                "camera_info_topic": "/depth_camera/camera_info",
                "scan_topic": "/scan",
                "scan_height_mode": "roi_min",
                "roi_top_ratio": 0.35,
                "roi_bottom_ratio": 0.65,
                "center_crop_ratio": 0.90,
                "min_valid_points_per_column": 2,
                "ground_filter_enabled": True,
                "ground_filter_bottom_roi_ratio": 0.12,
                "roi_percentile": 0.20,
                "range_min": 0.15,
                "range_max": 3.0,
                "scan_time": 0.1,
                "output_frame_id": "",
            },
        ],
    )

    reactive = Node(
        package="moonmapper_autonomy",
        executable="reactive_avoidance_node",
        name="reactive_avoidance_node",
        output="screen",
        parameters=[
            common_time,
            {
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
            },
        ],
    )

    safety = Node(
        package="moonmapper_autonomy",
        executable="safety_obstacle_node",
        name="safety_obstacle_node",
        output="screen",
        parameters=[
            common_time,
            {
                "input_cmd_topic": "/cmd_vel_raw",
                "output_cmd_topic": "/cmd_vel",
                "scan_topic": "/scan",
                "front_stop_distance": 0.75,
                "front_angle_deg": 50.0,
                "scan_timeout_sec": 0.5,
                "allow_reverse_when_blocked": True,
                "publish_safety_debug": True,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            depth_to_scan,
            reactive,
            safety,
        ]
    )
