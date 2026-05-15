"""Deprecated alias — use nav2_rtabmap_exploration_sim.launch.py or _real.launch.py."""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import sys

from ament_index_python.packages import get_package_share_directory

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
from _rtabmap_exploration_core import build_rtabmap_exploration  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    from launch import LaunchDescription as LD
    from launch.actions import LogInfo

    ld = build_rtabmap_exploration("sim")
    ld.entities.insert(0, LogInfo(
        msg=(
            "[moonmapper_nav2] nav2_rtabmap_exploration.launch.py → SIM mode. "
            "Use nav2_rtabmap_exploration_real.launch.py on hardware."
        )
    ))
    return ld
