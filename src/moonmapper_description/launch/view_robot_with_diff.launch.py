"""Launch rocker-bogie visualization med kinematic coupling.

Topologi:
    joint_state_publisher_gui  --/joint_states_raw-->
        rocker_bogie_kinematics (= rocker_diff_joint.py)
        --/joint_states-->  robot_state_publisher  --/tf-->  RViz2

Kun de to rocker-jointene er manipulerbare slidere (via YAML-config).
Resten av rocker-bogie-systemet foelger automatisk via kinematics-noden.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=default, description=desc)


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("moonmapper_description")

    default_model  = PathJoinSubstitution([pkg, "urdf", "moonmapper_rover.urdf.xacro"])
    default_rviz   = PathJoinSubstitution([pkg, "rviz", "moonmapper.rviz"])
    default_jspcfg = PathJoinSubstitution([pkg, "config", "jsp_drivers_only.yaml"])

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        _arg("model",        default_model,  "Path to robot xacro."),
        _arg("rvizconfig",   default_rviz,   "Path to rviz2 config."),
        _arg("jsp_config",   default_jspcfg, "YAML for joint_state_publisher(_gui)."),
        _arg("use_gui",      "true",         "Use joint_state_publisher_gui."),
        _arg("use_rviz",     "true",         "Start RViz2."),
        _arg("prefix",       "",             "Prefix for link/joint names."),
        _arg("use_gazebo",   "false",        "Enable Gazebo tags (placeholder)."),
        _arg("use_ros2_control", "false",    "Enable ros2_control tags (placeholder)."),
        _arg("mesh_scale",   "1.0",          "Uniform mesh scale."),

        # --- Joint name overrides (matcher URDF-navn) ---
        _arg("left_rocker",  "rocker_left_joint",      "URDF-navn for venstre rocker."),
        _arg("right_rocker", "rocker_right_joint",     "URDF-navn for hoegre rocker."),
        _arg("left_bogie",   "bogie_left_joint",       "URDF-navn for venstre bogie."),
        _arg("right_bogie",  "bogie_right_joint",      "URDF-navn for hoegre bogie."),
        _arg("left_hinge",   "hengsel_diff_L_joint",   "URDF-navn for venstre diff-hengsel."),
        _arg("right_hinge",  "hengsel_diff_R_joint",   "URDF-navn for hoegre diff-hengsel."),
        _arg("diff_joint",   "rocker_bogie_diff_joint","URDF-navn for diff-stangen."),

        # --- Diff gains:  k_L*L + k_R*R + k0  (default (L+R)/2) ---
        _arg("k_diff_L", "0.5", "Diff-gain for venstre rocker."),
        _arg("k_diff_R", "0.5", "Diff-gain for hoegre rocker."),
        _arg("k_diff_0", "0.0", "Diff-offset."),

        # --- Hinge gains:  d*rocker + e ---
        _arg("d_L", "1.0", "Hengsel L gain paa rocker_left."),
        _arg("e_L", "0.0", "Hengsel L offset."),
        _arg("d_R", "1.0", "Hengsel R gain paa rocker_right."),
        _arg("e_R", "0.0", "Hengsel R offset."),

        # --- Bogie gains:  a*rocker + b*diff + c ---
        _arg("a_bl", "-0.5", "Bogie L gain paa rocker_left."),
        _arg("b_bl",  "0.0", "Bogie L gain paa diff."),
        _arg("c_bl",  "0.0", "Bogie L offset."),
        _arg("a_br", "-0.5", "Bogie R gain paa rocker_right."),
        _arg("b_br",  "0.0", "Bogie R gain paa diff."),
        _arg("c_br",  "0.0", "Bogie R offset."),
    ]

    # ------------------------------------------------------------------
    # robot_description (xacro)
    # ------------------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " prefix:=",           LaunchConfiguration("prefix"),
            " use_gazebo:=",       LaunchConfiguration("use_gazebo"),
            " use_ros2_control:=", LaunchConfiguration("use_ros2_control"),
            " mesh_scale:=",       LaunchConfiguration("mesh_scale"),
        ]),
        value_type=str,
    )

    # ------------------------------------------------------------------
    # Noder
    # ------------------------------------------------------------------
    # robot_state_publisher lytter paa default /joint_states (vaar output)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # JSP (uten GUI) – fallback hvis use_gui:=false
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
        parameters=[LaunchConfiguration("jsp_config")],
        remappings=[("/joint_states", "/joint_states_raw")],
    )

    # JSP GUI – kun rocker-slidere aktive (resten hides via YAML)
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        parameters=[LaunchConfiguration("jsp_config")],
        remappings=[("/joint_states", "/joint_states_raw")],
    )

    # Kinematics-noden (tidligere "rocker_diff" – naa full kobling)
    kinematics_node = Node(
        package="moonmapper_description",
        executable="rocker_diff_joint.py",
        name="rocker_bogie_kinematics",
        output="screen",
        parameters=[{
            "input_topic":  "/joint_states_raw",
            "output_topic": "/joint_states",

            "left_rocker":  LaunchConfiguration("left_rocker"),
            "right_rocker": LaunchConfiguration("right_rocker"),
            "left_bogie":   LaunchConfiguration("left_bogie"),
            "right_bogie":  LaunchConfiguration("right_bogie"),
            "left_hinge":   LaunchConfiguration("left_hinge"),
            "right_hinge":  LaunchConfiguration("right_hinge"),
            "diff_joint":   LaunchConfiguration("diff_joint"),

            "k_diff_L": LaunchConfiguration("k_diff_L"),
            "k_diff_R": LaunchConfiguration("k_diff_R"),
            "k_diff_0": LaunchConfiguration("k_diff_0"),

            "d_L": LaunchConfiguration("d_L"),
            "e_L": LaunchConfiguration("e_L"),
            "d_R": LaunchConfiguration("d_R"),
            "e_R": LaunchConfiguration("e_R"),

            "a_bl": LaunchConfiguration("a_bl"),
            "b_bl": LaunchConfiguration("b_bl"),
            "c_bl": LaunchConfiguration("c_bl"),
            "a_br": LaunchConfiguration("a_br"),
            "b_br": LaunchConfiguration("b_br"),
            "c_br": LaunchConfiguration("c_br"),
        }],
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
        args + [rsp_node, jsp_node, jsp_gui_node, kinematics_node, rviz_node]
    )
