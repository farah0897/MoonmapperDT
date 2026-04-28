"""Launch MoonMapper i Gazebo Sim 8 (Harmonic).

Kjorer:
  * gz sim (server + GUI) med en valgfri verden
  * robot_state_publisher med Gazebo-xacro-wrapperen
  * ros_gz_sim create for å spawne roveren
  * ros_gz_bridge parameter_bridge for clock/tf/cmd_vel/sensorer
  * spawner for joint_state_broadcaster og diff_drive_controller
  * (valgfritt) RViz

Typisk bruk:
  ros2 launch moonmapper_description gazebo_rover.launch.py
  ros2 launch moonmapper_description gazebo_rover.launch.py world:=...moon_arena.sdf spawn_z:=0.15
  ros2 launch moonmapper_description gazebo_rover.launch.py enable_diff_plugin:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("moonmapper_description")

    default_xacro = PathJoinSubstitution(
        [pkg, "urdf", "moonmapper_rover_gazebo.urdf.xacro"]
    )
    default_world = PathJoinSubstitution([pkg, "worlds", "moon_arena.sdf"])
    default_rviz = PathJoinSubstitution([pkg, "rviz", "moonmapper.rviz"])
    default_bridge = PathJoinSubstitution(
        [pkg, "config", "ros_gz_bridge.yaml"]
    )
    default_controllers = PathJoinSubstitution(
        [pkg, "config", "wheel_controllers.yaml"]
    )

    args = [
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS_DOMAIN_ID for all processes in this launch.",
        ),
        DeclareLaunchArgument(
            "rmw_implementation",
            # Jazzy på Ubuntu installerer typisk FastDDS som standard.
            # CycloneDDS finnes ikke alltid (librmw_cyclonedds_cpp.so), så hold default trygg.
            default_value="rmw_fastrtps_cpp",
            description="RMW_IMPLEMENTATION for all processes in this launch.",
        ),
        DeclareLaunchArgument("model", default_value=default_xacro,
                              description="Path til Gazebo-xacro-wrapper."),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Path til .sdf-verden."),
        DeclareLaunchArgument(
            "world_name",
            default_value="moon_arena",
            description="Gazebo world name (for ros_gz_sim create -world).",
        ),
        DeclareLaunchArgument(
            "force_set_pose_after_spawn",
            default_value="true",
            description="Call /world/<name>/set_pose after spawning (ensures spawn_z is applied).",
        ),
        DeclareLaunchArgument(
            "print_pose_after_spawn",
            default_value="true",
            description="Print /world/<name>/pose/info once after set_pose (shows actual x/y/z).",
        ),
        DeclareLaunchArgument("rvizconfig", default_value=default_rviz,
                              description="RViz-konfig."),
        DeclareLaunchArgument("bridge_config", default_value=default_bridge,
                              description="ros_gz_bridge parameter_bridge YAML."),
        DeclareLaunchArgument("controllers", default_value=default_controllers,
                              description="ros2_control controllers YAML."),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true",
                              description="Aktiver RockerBogieDifferential-pluginen."),
        DeclareLaunchArgument("enable_triad_spectroscopy", default_value="false",
                              description="Aktiver TriadSpectroscopy raycast-plugin (mye loggstøy)."),
        DeclareLaunchArgument("diff_right_sign", default_value="1.0",
                              description="Diff-plugin: right_sign (+1: qL+qR, -1: qL-qR)."),
        DeclareLaunchArgument("diff_kp", default_value="5.0",
                              description="Diff-plugin: kp for rocker constraint."),
        DeclareLaunchArgument("diff_kd", default_value="0.5",
                              description="Diff-plugin: kd for rocker constraint."),
        DeclareLaunchArgument("diff_max_torque", default_value="0.5",
                              description="Diff-plugin: max torque for rocker constraint."),
        DeclareLaunchArgument("diff_k_diff_L", default_value="-0.41",
                              description="Diff-plugin: k_diff_L for diff target."),
        DeclareLaunchArgument("diff_k_diff_R", default_value="0.41",
                              description="Diff-plugin: k_diff_R for diff target."),
        DeclareLaunchArgument("diff_k_diff_0", default_value="0.0",
                              description="Diff-plugin: k_diff_0 for diff target."),
        DeclareLaunchArgument("diff_kp_aux", default_value="1.0",
                              description="Diff-plugin: kp_aux for diff joint."),
        DeclareLaunchArgument("diff_kd_aux", default_value="0.2",
                              description="Diff-plugin: kd_aux for diff joint."),
        DeclareLaunchArgument("diff_max_torque_aux", default_value="0.2",
                              description="Diff-plugin: max_torque_aux for diff joint."),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.2"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Vis Gazebo GUI (false = headless)."),
    ]

    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " use_gazebo:=true",
            
            " enable_diff_plugin:=", LaunchConfiguration("enable_diff_plugin"),
            " enable_urdf_mimic:=false",
            " enable_triad_spectroscopy:=", LaunchConfiguration("enable_triad_spectroscopy"),
            " diff_right_sign:=", LaunchConfiguration("diff_right_sign"),
            " diff_kp:=", LaunchConfiguration("diff_kp"),
            " diff_kd:=", LaunchConfiguration("diff_kd"),
            " diff_max_torque:=", LaunchConfiguration("diff_max_torque"),
            " diff_k_diff_L:=", LaunchConfiguration("diff_k_diff_L"),
            " diff_k_diff_R:=", LaunchConfiguration("diff_k_diff_R"),
            " diff_k_diff_0:=", LaunchConfiguration("diff_k_diff_0"),
            " diff_kp_aux:=", LaunchConfiguration("diff_kp_aux"),
            " diff_kd_aux:=", LaunchConfiguration("diff_kd_aux"),
            " diff_max_torque_aux:=", LaunchConfiguration("diff_max_torque_aux"),
        ]),
        value_type=str,
    )

    # Gi gz sim tilgang til pluginen og meshene via resource path.
    set_ros_domain = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=LaunchConfiguration("ros_domain_id"),
    )
    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw_implementation"),
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[PathJoinSubstitution([pkg, ".."]), ":",
               PathJoinSubstitution([pkg, "worlds"]), ":",
               PathJoinSubstitution([pkg, "meshes"])],
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[PathJoinSubstitution([FindPackageShare("moonmapper_description"),
                                     "..", "..", "lib"])],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"),
                                  "launch", "gz_sim.launch.py"]),
        ]),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r -v 3"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", LaunchConfiguration("world_name"),
            "-name", "moonmapper",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    # Hard-set pose after spawn (some physics/canonical-link setups may ignore create's initial pose).
    force_set_pose = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", ["/world/", LaunchConfiguration("world_name"), "/set_pose"],
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req",
            [
                "name: 'moonmapper' ",
                "position { x: ", LaunchConfiguration("spawn_x"),
                " y: ", LaunchConfiguration("spawn_y"),
                " z: ", LaunchConfiguration("spawn_z"),
                " } ",
                "orientation { w: 1 x: 0 y: 0 z: 0 }",
            ],
        ],
        output="screen",
        condition=__import__("launch.conditions", fromlist=["IfCondition"])
            .IfCondition(LaunchConfiguration("force_set_pose_after_spawn")),
    )

    print_pose = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            [
                "gz topic -e -t /world/",
                LaunchConfiguration("world_name"),
                "/pose/info -n 1 | sed -n '/name: \"moonmapper\"/,/orientation/p'",
            ],
        ],
        output="screen",
        condition=__import__("launch.conditions", fromlist=["IfCondition"])
            .IfCondition(LaunchConfiguration("print_pose_after_spawn")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("bridge_config"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    # ros2_control spawners (kjorer IN-PROCESS via gz_ros2_control; vi bare
    # aktiverer dem).
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    load_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Aktiver joint_state_broadcaster først, deretter diff_drive.
    after_spawn_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_rover, on_exit=[force_set_pose, print_pose, load_jsb]),
    )
    after_jsb_diff = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_diff]),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=__import__("launch.conditions", fromlist=["IfCondition"])
            .IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [
        set_ros_domain,
        set_rmw,
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        spawn_rover,
        bridge,
        after_spawn_jsb,
        after_jsb_diff,
        rviz,
    ])
