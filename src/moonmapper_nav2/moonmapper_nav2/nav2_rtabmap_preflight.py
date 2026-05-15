#!/usr/bin/env python3
"""Blocking preflight for RTAB+Nav2 stack. Exit 0 when ready, 1 on timeout."""

from __future__ import annotations

import sys
import time
from typing import List, Tuple

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown

try:
    from rclpy.wait_for_message import wait_for_message
except ImportError:
    wait_for_message = None  # type: ignore

SIM_CHECKS: List[Tuple[str, type]] = [
    ("/depth_camera/image", Image),
    ("/depth_camera/depth_image", Image),
    ("/depth_camera/camera_info", CameraInfo),
]
SIM_ODOM_TOPICS = ("/odom", "/diff_drive_controller/odom")
REAL_CHECKS: List[Tuple[str, type]] = [
    ("/camera/camera/color/image_raw", Image),
    ("/camera/camera/depth/image_rect_raw", Image),
    ("/camera/camera/color/camera_info", CameraInfo),
]


class Nav2RtabmapPreflight:
    def __init__(self, mode: str, use_sim_time: bool) -> None:
        self._mode = mode
        self._checks = list(SIM_CHECKS if mode == "sim" else REAL_CHECKS)
        self._base_frame = "base_footprint" if mode == "sim" else "base_link"
        self._use_sim_time = use_sim_time

    def _spin_graph(self, node: Node, n: int = 8) -> None:
        for _ in range(n):
            rclpy.spin_once(node, timeout_sec=0.15)

    def _topic_has_data(self, node: Node, topic: str, msg_type: type, timeout: float) -> bool:
        self._spin_graph(node)
        if node.count_publishers(topic) == 0:
            return False
        if wait_for_message is None:
            return True
        per_qos = max(0.5, timeout / 2.0)
        qos_profiles = (
            (qos_profile_default, qos_profile_sensor_data)
            if msg_type is Odometry
            else (qos_profile_sensor_data, qos_profile_default)
        )
        for qos in qos_profiles:
            try:
                wait_for_message(
                    msg_type,
                    node,
                    topic,
                    time_to_wait=per_qos,
                    qos_profile=qos,
                )
                return True
            except Exception:
                continue
        return False

    def _odom_ok(self, node: Node, timeout: float) -> bool:
        if self._mode == "sim":
            for topic in SIM_ODOM_TOPICS:
                if self._topic_has_data(node, topic, Odometry, timeout):
                    return True
            return False
        return self._topic_has_data(node, "/odom", Odometry, timeout)

    def run(self, max_wait_sec: float = 120.0) -> bool:
        rclpy.init()
        overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, self._use_sim_time)]
        node = Node("nav2_rtabmap_preflight", parameter_overrides=overrides)
        buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        tf2_ros.TransformListener(buf, node, spin_thread=True)
        deadline = time.monotonic() + max_wait_sec

        node.get_logger().info(
            f"preflight mode={self._mode} use_sim_time={self._use_sim_time} — "
            f"waiting for camera+odom+TF odom->{self._base_frame}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            missing: List[str] = []
            for topic, msg_type in self._checks:
                if not self._topic_has_data(node, topic, msg_type, 3.0):
                    missing.append(topic)
            if not self._odom_ok(node, 3.0):
                missing.append("/odom")

            try:
                buf.lookup_transform(
                    "odom", self._base_frame, Time(), timeout=Duration(seconds=0.5)
                )
                tf_ok = True
            except tf2_ros.TransformException:
                tf_ok = False

            if not missing and tf_ok:
                node.get_logger().info("PREFLIGHT OK — RTAB/Nav2 will start")
                node.destroy_node()
                safe_shutdown()
                return True

            hint = ""
            if self._mode == "sim":
                hint = " (ensure sim_rover_clean is running; drive once for /odom)"
            node.get_logger().warn(
                f"preflight waiting… topics_missing={missing or 'none'} "
                f"TF odom->{self._base_frame}={'OK' if tf_ok else 'MISSING'}{hint}"
            )
            time.sleep(2.0)

        node.get_logger().error("PREFLIGHT TIMEOUT — RTAB/Nav2 will NOT start")
        node.destroy_node()
        safe_shutdown()
        return False


def main(argv: List[str] | None = None) -> int:
    args = list(argv if argv is not None else sys.argv)
    mode = "sim"
    use_sim_time = False
    if len(args) > 1 and args[1] in ("sim", "real"):
        mode = args[1]
    for i, a in enumerate(args):
        if a.startswith("use_sim_time:=") or a == "use_sim_time:=true":
            use_sim_time = "true" in a.lower()
        if a == "-p" and i + 1 < len(args) and "use_sim_time" in args[i + 1]:
            use_sim_time = "true" in args[i + 1].lower()
    if mode == "sim" and "--ros-args" not in args:
        use_sim_time = True

    if mode not in ("sim", "real"):
        print("usage: nav2_rtabmap_preflight [sim|real] [--ros-args -p use_sim_time:=true]", file=sys.stderr)
        return 2
    try:
        return 0 if Nav2RtabmapPreflight(mode, use_sim_time).run() else 1
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        if is_shutdown_exception(exc):
            return 0
        raise


if __name__ == "__main__":
    sys.exit(main())
