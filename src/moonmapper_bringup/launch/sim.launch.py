"""
MoonMapper sim bringup (Webots):
  - Webots + driver
  - robot_state_publisher
  - optional: teleop, EKF, SLAM, RViz

Usage:
  ros2 launch moonmapper_bringup sim.launch.py
    [use_rviz:=true|false]
    [use_teleop:=true|false]
    [use_ekf:=true|false]
    [ekf_publish_tf:=true|false]
    [use_slam:=true|false]
    [use_sim_time:=true|false]
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_teleop_arg = DeclareLaunchArgument("use_teleop", default_value="false")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false")
    ekf_publish_tf_arg = DeclareLaunchArgument(
        "ekf_publish_tf",
        default_value="true",
        description=(
            "If true, EKF publishes odom->base_link TF. "
            "Disable to avoid duplicate TF when simulation/driver already publishes it."
        ),
    )
    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")
    use_ekf = LaunchConfiguration("use_ekf")
    ekf_publish_tf = LaunchConfiguration("ekf_publish_tf")
    use_slam = LaunchConfiguration("use_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_dir = get_package_share_directory("moonmapper_bringup")
    desc_dir = get_package_share_directory("moonmapper_description")
    webots_dir = get_package_share_directory("moonmapper_webots")
    loc_dir = get_package_share_directory("moonmapper_localization")
    slam_dir = get_package_share_directory("moonmapper_slam")

    urdf_path = os.path.join(desc_dir, "urdf", "moonmapper_rover.urdf.xacro")
    robot_description = process_file(urdf_path).toxml()

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(webots_dir, "launch", "webots_launch.py")
        ),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    teleop = Node(
        package="moonmapper_control",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        condition=IfCondition(use_teleop),
    )

    ekf_params = os.path.join(loc_dir, "config", "ekf.yaml")
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_params,
            {
                "use_sim_time": use_sim_time,
                "publish_tf": ekf_publish_tf,
            },
        ],
        condition=IfCondition(use_ekf),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "slam.launch.py")
        ),
        condition=IfCondition(use_slam),
    )

    rviz_config = os.path.join(bringup_dir, "config", "moonmapper_rviz.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_teleop_arg,
        use_ekf_arg,
        ekf_publish_tf_arg,
        use_slam_arg,
        use_sim_time_arg,
        webots_launch,
        robot_state_publisher,
        teleop,
        ekf_node,
        slam_launch,
        rviz,
    ])
