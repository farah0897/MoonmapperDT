"""unity_minimal.launch.py

Minimal ROS 2-stack for Unity-simulering.

Starter:
    - robot_state_publisher (publiserer /robot_description + /tf fra /joint_states)
    - ros_tcp_endpoint      (TCP-server for Unity ROS-TCP-Connector)
    - rviz2                 (visualiser sensordata fra Unity)

Unity-siden:
    - Importerer URDF-en (install/moonmapper_description/share/.../moonmapper.urdf.xacro
      eller foerst kjoer `xacro ... > moonmapper.urdf` og importer den).
    - ROS Settings: Protocol=ROS 2, IP=<ROS-host>, Port=10000.
    - Publiserer /joint_states, /imu, /stereo/left/image_raw, /depth_camera/points osv.
    - Abonnerer paa /unity/wheel_velocities (fra full-launch) hvis fjernstyring trengs.

Typisk bruk:
    ros2 launch moonmapper_description unity_minimal.launch.py
    # Start saa Unity Play-mode.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("moonmapper_description")

    default_model = PathJoinSubstitution(
        [pkg_share, "urdf", "moonmapper.urdf.xacro"])
    default_rviz = PathJoinSubstitution(
        [pkg_share, "rviz", "moonmapper.rviz"])
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "unity_params.yaml"])

    # --- Launch-argumenter ---------------------------------------------------
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Full sti til rover-xacro.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz,
        description="Full sti til RViz-config.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML med ros_tcp_endpoint + unity_cmd_vel_bridge-parametre.",
    )
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="0.0.0.0",
        description="IP-en Unity kobler seg til. 0.0.0.0 = alle grensesnitt.",
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port",
        default_value="10000",
        description="TCP-port for Unity ROS-TCP-Connector.",
    )

    # --- robot_description via xacro ----------------------------------------
    # use_gazebo er beholdt som xacro-arg for bakoverkompatibilitet,
    # men settes eksplisitt false her slik at world->base_footprint
    # fikserer roveren i RViz-visualiseringen.
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
                " use_gazebo:=false",
            ]
        ),
        value_type=str,
    )

    # --- Noder ---------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    tcp_endpoint_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "ROS_IP": LaunchConfiguration("ros_ip"),
                "ROS_TCP_PORT": LaunchConfiguration("ros_tcp_port"),
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            use_rviz_arg,
            params_arg,
            ros_ip_arg,
            ros_tcp_port_arg,
            rsp_node,
            tcp_endpoint_node,
            rviz_node,
        ]
    )
