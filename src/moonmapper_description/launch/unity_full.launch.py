"""unity_full.launch.py

Full ROS 2-stack for Unity-simulering med fjernstyring.

Starter alt fra unity_minimal.launch.py pluss:
    - unity_cmd_vel_bridge : /cmd_vel -> /unity/wheel_velocities (6-elements array)
    - teleop_twist_keyboard (valgfri, default=true, egen terminal)

Topologi:
    tastatur --> /cmd_vel --> unity_cmd_vel_bridge --> /unity/wheel_velocities --> Unity
    Unity --> /joint_states --> robot_state_publisher --> /tf --> RViz
    Unity --> /imu, /stereo/*, /depth_camera/* --> RViz

Typisk bruk:
    ros2 launch moonmapper_description unity_full.launch.py

    # I egen terminal (hvis start_teleop:=false):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("moonmapper_description")

    default_params = PathJoinSubstitution(
        [pkg_share, "config", "unity_params.yaml"])

    # --- Launch-argumenter ---------------------------------------------------
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML med ros_tcp_endpoint + unity_cmd_vel_bridge-parametre.",
    )
    start_teleop_arg = DeclareLaunchArgument(
        "start_teleop",
        default_value="true",
        description="Start teleop_twist_keyboard i egen xterm-terminal.",
    )
    stamped_arg = DeclareLaunchArgument(
        "stamped",
        default_value="false",
        description=(
            "Hvis true: teleop publiserer TwistStamped paa /cmd_vel. "
            "unity_cmd_vel_bridge lytter paa begge typer, saa verdien "
            "paavirker bare teleop-klienten. frame_id settes til base_link."
        ),
    )
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip", default_value="0.0.0.0")
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port", default_value="10000")

    # --- Inkluder minimal-oppsettet (RSP + TCP-endpoint + RViz) --------------
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "unity_minimal.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "ros_ip": LaunchConfiguration("ros_ip"),
            "ros_tcp_port": LaunchConfiguration("ros_tcp_port"),
        }.items(),
    )

    # --- /cmd_vel -> /unity/wheel_velocities ---------------------------------
    # use_stamped settes fra launch-arg `stamped` saa broen abonnerer paa
    # riktig meldingstype (Twist eller TwistStamped).
    cmd_vel_bridge_node = Node(
        package="moonmapper_description",
        executable="unity_cmd_vel_node.py",
        name="unity_cmd_vel_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_stamped": LaunchConfiguration("stamped")},
        ],
    )

    # --- Tastatur-teleop (valgfri) -------------------------------------------
    # Default: kjoer teleop i egen terminal-emulator som brukeren VELGER
    # selv via `teleop_terminal`-argumentet. Gyldige valg:
    #   xterm           (krever `sudo apt install xterm`)
    #   gnome-terminal  (krever `sudo apt install gnome-terminal`)
    #   konsole
    #   none            -> ikke start teleop (kjoer selv i annen terminal)
    teleop_cmd_twist = [
        "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
        "--ros-args",
        "-p", "speed:=0.05",
        # Stoerre turn enn speed/L saa svinger gir synlig differensial
        # (unity_cmd_vel_bridge klemmer uansett til max_angular_velocity).
        "-p", "turn:=1.5",
    ]
    teleop_cmd_stamped = teleop_cmd_twist + [
        "-p", "stamped:=true",
        "-p", "frame_id:=base_link",
    ]

    teleop_twist = ExecuteProcess(
        cmd=["xterm", "-T", "teleop_twist_keyboard [Twist]", "-e",
             *teleop_cmd_twist],
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("start_teleop"),
                NotSubstitution(LaunchConfiguration("stamped")),
            )
        ),
    )

    teleop_twist_stamped = ExecuteProcess(
        cmd=["xterm", "-T", "teleop_twist_keyboard [TwistStamped]", "-e",
             *teleop_cmd_stamped],
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("start_teleop"),
                LaunchConfiguration("stamped"),
            )
        ),
    )

    return LaunchDescription(
        [
            params_arg,
            start_teleop_arg,
            stamped_arg,
            ros_ip_arg,
            ros_tcp_port_arg,
            minimal_launch,
            cmd_vel_bridge_node,
            teleop_twist,
            teleop_twist_stamped,
        ]
    )
