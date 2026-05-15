"""Depth -> /scan + safety på /cmd_vel_raw -> /cmd_vel (uten reactive_avoidance)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    front_stop = LaunchConfiguration("front_stop_distance")
    front_angle = LaunchConfiguration("front_angle_deg")
    scan_timeout = LaunchConfiguration("scan_timeout_sec")
    allow_rev = LaunchConfiguration("allow_reverse_when_blocked")
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
                "front_stop_distance": ParameterValue(front_stop, value_type=float),
                "front_angle_deg": ParameterValue(front_angle, value_type=float),
                "scan_timeout_sec": ParameterValue(scan_timeout, value_type=float),
                "allow_reverse_when_blocked": ParameterValue(allow_rev, value_type=bool),
                "publish_safety_debug": True,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_reverse_when_blocked",
                default_value="true",
                description="Ved blocked_front: tillat negativ linear.x (rygg) fra raw kommando.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "front_stop_distance",
                default_value="0.55",
                description="Stopp fremover hvis nærmeste front-stråle er nærmere enn dette (m).",
            ),
            DeclareLaunchArgument(
                "front_angle_deg",
                default_value="50.0",
                description="Halv bredde (grader) for front-sektor på /scan.",
            ),
            DeclareLaunchArgument(
                "scan_timeout_sec",
                default_value="0.5",
                description="Maks alder på /scan-stempel (s).",
            ),
            depth_to_scan,
            safety,
        ]
    )
