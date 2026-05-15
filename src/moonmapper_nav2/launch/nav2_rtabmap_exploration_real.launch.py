"""RTAB-Map + Nav2 on physical rover (RealSense, base_link, robot.py)."""

from __future__ import annotations

import sys
import os

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
from _rtabmap_exploration_core import build_rtabmap_exploration  # noqa: E402


def generate_launch_description():
    return build_rtabmap_exploration("real")
