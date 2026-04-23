"""Launch Webots + MoonMapper driver (used by moonmapper_bringup)."""
import os
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg_dir = get_package_share_directory("moonmapper_webots")
    robot_desc_path = os.path.join(pkg_dir, "resource", "moonmapper_rover_webots.urdf")
    world_path = os.path.join(pkg_dir, "worlds", "moonmapper_world.wbt")

    webots = WebotsLauncher(world=world_path)

    driver = WebotsController(
        robot_name="moonmapper_rover",
        parameters=[{"robot_description": robot_desc_path}],
    )

    return launch.LaunchDescription([
        webots,
        driver,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
