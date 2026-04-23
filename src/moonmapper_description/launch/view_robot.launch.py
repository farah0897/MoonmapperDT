from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("moonmapper_description")

    default_model = PathJoinSubstitution([pkg_share, "urdf", "moonmapper.urdf.xacro"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "moonmapper.rviz"])
    default_vec3 = "0 0 0"

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to robot xacro file.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz,
        description="Absolute path to rviz2 config file.",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Use joint_state_publisher_gui (otherwise joint_state_publisher).",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with a preconfigured config.",
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="Prefix for link/joint names (multi-robot).",
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Enable Gazebo tags (placeholder for future).",
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Enable ros2_control tags (placeholder for future).",
    )
    mesh_scale_arg = DeclareLaunchArgument(
        "mesh_scale",
        default_value="1.0",
        description="Uniform mesh scale (e.g. 0.001 for mm->m).",
    )
    bogie_left_xyz_arg = DeclareLaunchArgument(
        "bogie_left_xyz",
        default_value=default_vec3,
        description="Bogie left xyz offset relative to base_link (meters).",
    )
    bogie_left_rpy_arg = DeclareLaunchArgument(
        "bogie_left_rpy",
        default_value=default_vec3,
        description="Bogie left rpy offset relative to base_link (radians).",
    )
    bogie_right_xyz_arg = DeclareLaunchArgument(
        "bogie_right_xyz",
        default_value=default_vec3,
        description="Bogie right xyz offset relative to base_link (meters).",
    )
    bogie_right_rpy_arg = DeclareLaunchArgument(
        "bogie_right_rpy",
        default_value=default_vec3,
        description="Bogie right rpy offset relative to base_link (radians).",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
                " prefix:=",
                LaunchConfiguration("prefix"),
                " use_gazebo:=",
                LaunchConfiguration("use_gazebo"),
                " use_ros2_control:=",
                LaunchConfiguration("use_ros2_control"),
                " mesh_scale:=",
                LaunchConfiguration("mesh_scale"),
                " bogie_left_xyz:='",
                LaunchConfiguration("bogie_left_xyz"),
                "' bogie_left_rpy:='",
                LaunchConfiguration("bogie_left_rpy"),
                "' bogie_right_xyz:='",
                LaunchConfiguration("bogie_right_xyz"),
                "' bogie_right_rpy:='",
                LaunchConfiguration("bogie_right_rpy"),
                "'",
            ]
        ),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
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
            use_gui_arg,
            use_rviz_arg,
            prefix_arg,
            use_gazebo_arg,
            use_ros2_control_arg,
            mesh_scale_arg,
            bogie_left_xyz_arg,
            bogie_left_rpy_arg,
            bogie_right_xyz_arg,
            bogie_right_rpy_arg,
            rsp_node,
            jsp_node,
            jsp_gui_node,
            rviz_node,
        ]
    )
