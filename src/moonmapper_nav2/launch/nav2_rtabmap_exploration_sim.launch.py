"""RTAB-Map + Nav2 in Gazebo sim (depth_camera topics, base_footprint)."""

from __future__ import annotations

import os
import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
from _rtabmap_exploration_core import build_rtabmap_exploration  # noqa: E402


def generate_launch_description():
    return build_rtabmap_exploration("sim")
