"""Launch MoonMapper i Gazebo Sim 8 (Harmonic).

Kjører:
  * gz sim (server + GUI) med en valgfri verden
  * robot_state_publisher med Gazebo-xacro-wrapperen
  * ros_gz_sim create for å spawne roveren
  * ros_gz_bridge parameter_bridge for clock/tf/sensorer
  * spawner for joint_state_broadcaster og diff_drive_controller
  * (valgfritt) RViz

Typisk bruk:

  ros2 launch moonmapper_description gazebo_rover.launch.py

  ros2 launch moonmapper_description gazebo_rover.launch.py world:=...moon_arena.sdf spawn_z:=0.006

  # EKF (robot_localization), hjul-odom som eneste odom-TF-kilde:
  #   ros2 launch moonmapper_description gazebo_rover.launch.py use_ekf:=true

  # Viktig: vertikal høyde heter spawn_z (ikke z_spawn). z_spawn:=... virker som alias.
  # Etter endring i denne fila: colcon build --packages-select moonmapper_description

Teleop (hjul styres KUN her — ikke /cmd_vel til GZ):

  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
    -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true -p frame_id:=base_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _physics_only_debug(context):
    """physics_only_debug: uten RViz; tvinger ikke rocker_diff_debug (unngår gz-plugin-spam)."""
    raw = (context.launch_configurations.get("physics_only_debug") or "").strip().lower()
    if raw not in ("true", "1", "yes"):
        return []
    return [
        SetLaunchConfiguration("use_rviz", "false"),
        LogInfo(
            msg=(
                "[gazebo_rover] physics_only_debug: use_rviz=false (Gazebo+ros2_control isolert). "
                "rocker_diff_coupling_mode default=weak (anbefalt). "
                "rocker_diff_debug styres kun av launch-arg (default false) — sett true for throttlet qL/qR-logg. "
                "coupling_mode:=off er diagnose/passiv URDF; ikke normal kjøring."
            ),
        ),
    ]


def _diff_relay_ekf_chain(context, *, load_jsb):
    """diff_drive-spawner (valgfri odom-TF-overlay), relay og valgfri EKF etter load_jsb."""
    desc_share = get_package_share_directory("moonmapper_description")
    bringup_share = get_package_share_directory("moonmapper_bringup")
    use_ekf = (context.launch_configurations.get("use_ekf") or "false").strip().lower() in (
        "true",
        "1",
        "yes",
    )
    base_diff = os.path.join(desc_share, "config", "diff_drive_controller.yaml")
    overlay = os.path.join(desc_share, "config", "diff_drive_controller_disable_odom_tf.yaml")
    diff_args = [
        "diff_drive_controller",
        "--controller-manager",
        "/controller_manager",
        "-p",
        base_diff,
    ]
    if use_ekf:
        diff_args.extend(["-p", overlay])

    load_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=diff_args,
        output="screen",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    relay_params = [
        {"use_sim_time": use_sim_time},
        {"publish_odom_relay": not use_ekf},
        {
            "use_smoothed_twist_stamped_input": ParameterValue(
                LaunchConfiguration("cmd_vel_relay_use_smoothed_twist_stamped"),
                value_type=bool,
            ),
        },
        {
            "twist_stamped_topic": LaunchConfiguration("cmd_vel_relay_twist_stamped_topic"),
        },
        {
            "publish_twist_cmd_vel_mirror": ParameterValue(
                LaunchConfiguration("cmd_vel_relay_publish_twist_cmd_vel_mirror"),
                value_type=bool,
            ),
        },
    ]

    cmd_vel_odom_relay = Node(
        package="moonmapper_bringup",
        executable="cmd_vel_odom_relay",
        name="cmd_vel_odom_relay",
        parameters=relay_params,
        output="screen",
    )

    rocker_limit_watch = Node(
        package="moonmapper_bringup",
        executable="rocker_joint_limit_watch",
        name="rocker_joint_limit_watch",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    ekf_params_path = os.path.join(bringup_share, "config", "ekf_wheel_odom.yaml")
    ekf_pub_raw = (context.launch_configurations.get("ekf_publish_tf") or "true").strip().lower()
    ekf_publish_tf = ekf_pub_raw in ("true", "1", "yes")
    builtin_wheel_ekf = (context.launch_configurations.get("builtin_wheel_ekf") or "true").strip().lower() in (
        "true",
        "1",
        "yes",
    )

    on_exit_after_diff = [cmd_vel_odom_relay, rocker_limit_watch]
    if use_ekf and builtin_wheel_ekf:
        ekf_node = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params_path,
                {
                    "use_sim_time": use_sim_time,
                    "publish_tf": ekf_publish_tf,
                },
            ],
        )
        on_exit_after_diff.append(ekf_node)

    return [
        RegisterEventHandler(
            OnProcessExit(target_action=load_jsb, on_exit=[load_diff]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=load_diff, on_exit=on_exit_after_diff),
        ),
    ]


def _spawn_z_alias(context):
    """z_spawn:= er vanlig skrivefeil; kopier til spawn_z hvis z_spawn er satt."""
    z_alias = (context.launch_configurations.get("z_spawn") or "").strip()
    if z_alias:
        return [
            LogInfo(
                msg=(
                    "[gazebo_rover] z_spawn er satt; bruk helst spawn_z:= for samme verdi. "
                    f"Bruker z={z_alias} m som spawn-høyde."
                ),
            ),
            SetLaunchConfiguration("spawn_z", z_alias),
        ]
    return []


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("moonmapper_description")
    default_xacro = PathJoinSubstitution(
        [pkg, "urdf", "moonmapper_rover_gazebo.urdf.xacro"],
    )
    default_world = PathJoinSubstitution([pkg, "worlds", "moon_arena.sdf"])
    default_rviz = PathJoinSubstitution([pkg, "rviz", "moonmapper.rviz"])
    default_bridge = PathJoinSubstitution([pkg, "config", "ros_gz_bridge.yaml"])
    default_controllers = PathJoinSubstitution(
        [pkg, "config", "wheel_controllers.yaml"],
    )
    default_joint_state_broadcaster = PathJoinSubstitution(
        [pkg, "config", "joint_state_broadcaster.yaml"],
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
        DeclareLaunchArgument(
            "model",
            default_value=default_xacro,
            description="Path til Gazebo-xacro-wrapper.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Path til .sdf-verden.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="moon_arena",
            description="Gazebo world name (for ros_gz_sim create -world).",
        ),
        DeclareLaunchArgument(
            "gz_sim_verbosity",
            default_value="2",
            description=(
                "gz sim -v nivaa (0-4). 3 = meget detaljert (tungt med GUI+RTAB+Nav2); "
                "2 eller 1 gir mindre logg- og UI-belastning."
            ),
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
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz,
            description="RViz-konfig.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=default_bridge,
            description="ros_gz_bridge parameter_bridge YAML.",
        ),
        DeclareLaunchArgument(
            "controllers",
            default_value=default_controllers,
            description="ros2_control controllers YAML.",
        ),
        DeclareLaunchArgument(
            "enable_triad_spectroscopy",
            default_value="false",
            description="Aktiver TriadSpectroscopy raycast-plugin (mye loggstøy).",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.006",
            description=(
                "Start-høyde (m) for ros_gz_sim create -z og set_pose (modellens rot-frame = base_footprint). "
                "URDF løfter base_link med base_link_z_above_footprint (~0,124 m); default ~0,006 m tilsvarer "
                "tidligere ~0,13 m for hele kroppen da footprint og base_link var sammenfallende. "
                "Juster til hjulene treffer regolith-plan (z=0). Ikke forveksle med z_spawn."
            ),
        ),
        DeclareLaunchArgument(
            "z_spawn",
            default_value="",
            description=(
                "Alias: hvis ikke tom overstyrer spawn_z (for vanlig skrivefeil). "
                "Foretrekk spawn_z:=0.006 på kommandolinja (eller juster etter arena)."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description=(
                "True: robot_localization EKF (hjul-odom), /odometry/filtered, "
                "TF odom->base_footprint; diff_drive enable_odom_tf av; ingen /odom-relay."
            ),
        ),
        DeclareLaunchArgument(
            "builtin_wheel_ekf",
            default_value="true",
            description=(
                "Nar use_ekf: true = start innebygd ekf_filter_node (ekf_wheel_odom.yaml). "
                "False = ekstern lokal/global EKF (f.eks. UWB digital tvilling); behold diff_drive TF-overlay."
            ),
        ),
        DeclareLaunchArgument(
            "ekf_publish_tf",
            default_value="true",
            description="Nar use_ekf: om EKF skal publisere TF (odom->base_footprint).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_use_smoothed_twist_stamped",
            default_value="false",
            description=(
                "True: cmd_vel_odom_relay abonnerer paa TwistStamped-topic "
                "(se cmd_vel_relay_twist_stamped_topic) i stedet for /cmd_vel (Twist)."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_twist_stamped_topic",
            default_value="/cmd_vel_smoothed",
            description="TwistStamped-inngang naar use_smoothed er true (default /cmd_vel_smoothed).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_publish_twist_cmd_vel_mirror",
            default_value="true",
            description=(
                "Nar use_smoothed: true = publiser ogsaa speil-Twist paa /cmd_vel. "
                "False naar velocity_smoother allerede publiserer til /cmd_vel."
            ),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Vis Gazebo GUI (false = headless).",
        ),
        DeclareLaunchArgument(
            "physics_only_debug",
            default_value="false",
            description=(
                "Når true: use_rviz=false (isoler Gazebo+diff_drive). "
                "Tvinger ikke rocker_diff_debug — default forblir false (ingen RockerBogieDiff-spam). "
                "rocker_diff_coupling_mode default forblir weak."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_coupling_mode",
            default_value="weak",
            description=(
                "RockerBogieDifferential: off | weak | full. "
                "weak=standard i sim (lav PD, walking-beam + diff-hint). "
                "off=diagnose/passiv URDF uten plugin-krefter (kan mette begge rockere ~0,61 rad; ikke normal modus). "
                "full=stivere PD."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_debug",
            default_value="false",
            description=(
                "Throttlet gzmsg (qL,qR,bogie,diff,tau) i RockerBogieDifferential hvert print_interval. "
                "Kun true når du eksplisitt vil ha plugin-debug."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_print_interval",
            default_value="0.5",
            description="Sekunder mellom debug-linjer når rocker_diff_debug:=true.",
        ),
        DeclareLaunchArgument(
            "simple_collision_debug",
            default_value="false",
            description=(
                "Når true: rocker/bogie/hjul bruker primitive collision (boks/sylinder) i URDF; "
                "STL-collision av; stereo-kamera uten mesh-collision. Mindre snagging i Gazebo."
            ),
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
                " use_gazebo:=true",
                " enable_urdf_mimic:=false",
                " enable_triad_spectroscopy:=",
                LaunchConfiguration("enable_triad_spectroscopy"),
                " rocker_diff_coupling_mode:=",
                LaunchConfiguration("rocker_diff_coupling_mode"),
                " rocker_diff_debug:=",
                LaunchConfiguration("rocker_diff_debug"),
                " rocker_diff_print_interval:=",
                LaunchConfiguration("rocker_diff_print_interval"),
                " simple_collision_debug:=",
                LaunchConfiguration("simple_collision_debug"),
            ],
        ),
        value_type=str,
    )

    # Gi gz sim tilgang til meshene via resource path.
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
        value=[
            PathJoinSubstitution([pkg, ".."]),
            ":",
            PathJoinSubstitution([pkg, "worlds"]),
            ":",
            PathJoinSubstitution([pkg, "meshes"]),
        ],
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            PathJoinSubstitution(
                [FindPackageShare("moonmapper_description"), "..", "..", "lib"],
            ),
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"],
                ),
            ],
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                TextSubstitution(text=" -r -v "),
                LaunchConfiguration("gz_sim_verbosity"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
            "-name",
            "moonmapper",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    # NB: Ikke bruk launch.substitutions.Command her — den *kjører* en shell-kommando og
    # fanger stdout (brukes til xacro → robot_description). For gz service: bruk lister
    # som ExecuteProcess flater ut til ett argument per pose / --req-streng.
    force_set_pose = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            ["/world/", LaunchConfiguration("world_name"), "/set_pose"],
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            [
                'name: "moonmapper" ',
                "position { x: ",
                LaunchConfiguration("spawn_x"),
                " y: ",
                LaunchConfiguration("spawn_y"),
                " z: ",
                LaunchConfiguration("spawn_z"),
                " } ",
                "orientation { w: 1 x: 0 y: 0 z: 0 }",
            ],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("force_set_pose_after_spawn")),
    )

    print_pose = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "gz topic -e -t /world/",
                LaunchConfiguration("world_name"),
                "/pose/info -n 1 | sed -n '/name: \"moonmapper\"/,/orientation/p'",
            ],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("print_pose_after_spawn")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("bridge_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    # ros2_control spawners (kjører IN-PROCESS via gz_ros2_control; vi bare aktiverer dem).
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "-p",
            default_joint_state_broadcaster,
        ],
        output="screen",
    )

    # diff_drive-spawner (valgfri odom-TF-overlay), cmd_vel_odom_relay, rocker_watch,
    # valgfri EKF — bygges i OpaqueFunction for aa lese use_ekf fra launch-kontekst.
    diff_relay_ekf_opaque = OpaqueFunction(
        function=lambda context: _diff_relay_ekf_chain(context, load_jsb=load_jsb),
    )

    # Aktiver joint_state_broadcaster først; deretter diff_drive + relay (+ EKF) via opaque.
    after_spawn_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_rover,
            on_exit=[force_set_pose, print_pose, load_jsb],
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        args
        + [
            OpaqueFunction(function=_physics_only_debug),
            OpaqueFunction(function=_spawn_z_alias),
            set_ros_domain,
            set_rmw,
            set_gz_resource_path,
            set_gz_plugin_path,
            gz_sim,
            robot_state_publisher,
            spawn_rover,
            bridge,
            after_spawn_jsb,
            diff_relay_ekf_opaque,
            rviz,
        ],
    )
