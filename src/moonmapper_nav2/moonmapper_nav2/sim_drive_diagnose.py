#!/usr/bin/env python3
"""Print sim drive-chain diagnostics: cmd_vel, odom, TF, optional Gazebo pose."""

from __future__ import annotations

import math
import subprocess
import sys
import time
from typing import List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.time import Time

try:
    from rclpy.wait_for_message import wait_for_message
except ImportError:
    wait_for_message = None  # type: ignore


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _topic_endpoints(topic: str) -> Tuple[List[str], List[str]]:
    try:
        out = subprocess.run(
            ["ros2", "topic", "info", topic, "-v"],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return [], []
    pubs: List[str] = []
    subs: List[str] = []
    section = ""
    for line in out.stdout.splitlines():
        s = line.strip()
        if s.startswith("Publisher count:"):
            section = "pub"
            continue
        if s.startswith("Subscription count:"):
            section = "sub"
            continue
        if s.startswith("Node name:") and section == "pub":
            pubs.append(s.split(":", 1)[1].strip())
        elif s.startswith("Node name:") and section == "sub":
            subs.append(s.split(":", 1)[1].strip())
    return pubs, subs


def _gz_base_yaw(world: str = "moon_arena") -> Optional[float]:
    """Yaw of moonmapper model in Gazebo (world frame), if gz CLI is available."""
    topic = f"/world/{world}/pose/info"
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            capture_output=True,
            text=True,
            timeout=6,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if out.returncode != 0:
        return None
    in_model = False
    for line in out.stdout.splitlines():
        if 'name: "moonmapper"' in line or "name: 'moonmapper'" in line:
            in_model = True
            continue
        if in_model and line.strip().startswith("orientation"):
            continue
        if in_model and "z:" in line and "w:" not in line:
            # skip partial parse
            pass
        if in_model and "w:" in line:
            # next lines have x,y,z,w — fragile; scan block
            pass
    # Simpler: parse last orientation block after moonmapper
    block = out.stdout.split('name: "moonmapper"')
    if len(block) < 2:
        return None
    tail = block[1]
    vals = {}
    for key in ("x", "y", "z", "w"):
        marker = f"{key}:"
        if marker not in tail:
            return None
        frag = tail.split(marker, 1)[1].split("\n", 1)[0].strip()
        vals[key] = float(frag)
    return _yaw_from_quat(vals["x"], vals["y"], vals["z"], vals["w"])


class SimDriveDiagnose(Node):
    def __init__(self) -> None:
        super().__init__("sim_drive_diagnose")
        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("world_name", "moon_arena")
        self.declare_parameter("sample_sec", 2.0)

    def run(self) -> int:
        use_sim = self.get_parameter("use_sim_time").value
        world = str(self.get_parameter("world_name").value)
        sample = float(self.get_parameter("sample_sec").value)

        print("=== MoonMapper sim drive chain diagnose ===")
        print(f"use_sim_time={use_sim}  sample_sec={sample}")
        print()

        for topic in (
            "/cmd_vel",
            "/cmd_vel_raw",
            "/cmd_vel_smoothed",
            "/cmd_vel_nav",
            "/diff_drive_controller/cmd_vel",
            "/odom",
            "/diff_drive_controller/odom",
        ):
            pubs, subs = _topic_endpoints(topic)
            print(f"--- {topic} ---")
            print(f"  publishers: {pubs or '(none)'}")
            print(f"  subscribers: {subs or '(none)'}")

        print()
        print("--- TF odom -> base_footprint ---")
        buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        tf2_ros.TransformListener(buf, self, spin_thread=True)
        t0 = time.monotonic()
        odom_yaw0: Optional[float] = None
        map_odom_yaw0: Optional[float] = None
        while time.monotonic() - t0 < sample:
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf_odom = buf.lookup_transform(
                "odom", "base_footprint", Time(), timeout=Duration(seconds=1.0)
            )
            q = tf_odom.transform.rotation
            odom_yaw0 = _yaw_from_quat(q.x, q.y, q.z, q.w)
            print(
                f"  OK  x={tf_odom.transform.translation.x:.4f} "
                f"y={tf_odom.transform.translation.y:.4f} "
                f"yaw={math.degrees(odom_yaw0):.2f} deg"
            )
        except tf2_ros.TransformException as exc:
            print(f"  MISSING: {exc}")

        print()
        print("--- TF map -> odom (RTAB-Map) ---")
        try:
            tf_map = buf.lookup_transform(
                "map", "odom", Time(), timeout=Duration(seconds=1.0)
            )
            q = tf_map.transform.rotation
            map_odom_yaw0 = _yaw_from_quat(q.x, q.y, q.z, q.w)
            print(
                f"  OK  x={tf_map.transform.translation.x:.4f} "
                f"y={tf_map.transform.translation.y:.4f} "
                f"yaw={math.degrees(map_odom_yaw0):.2f} deg"
            )
        except tf2_ros.TransformException as exc:
            print(f"  (not available — OK without RTAB stack): {exc}")

        print()
        print("--- /odom sample ---")
        odom_msg: Optional[Odometry] = None
        if wait_for_message is not None:
            try:
                odom_msg = wait_for_message(
                    Odometry, self, "/odom", time_to_wait=3.0, qos_profile=qos_profile_default
                )
            except Exception as exc:
                print(f"  no /odom message: {exc}")
        if odom_msg is not None:
            q = odom_msg.pose.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            print(
                f"  pose yaw={math.degrees(yaw):.2f} deg  "
                f"twist lin.x={odom_msg.twist.twist.linear.x:.4f} "
                f"ang.z={odom_msg.twist.twist.angular.z:.4f}"
            )
            print(f"  header.frame_id={odom_msg.header.frame_id}  child={odom_msg.child_frame_id}")

        print()
        print("--- Gazebo model yaw (ground truth) ---")
        gz_yaw = _gz_base_yaw(world)
        if gz_yaw is None:
            print("  (gz topic unavailable — start sim and ensure gz in PATH)")
        else:
            print(f"  moonmapper yaw ≈ {math.degrees(gz_yaw):.2f} deg (world frame)")

        print()
        print("--- Expected chain (sim) ---")
        print("  Nav2: controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed")
        print("        -> collision_monitor -> /cmd_vel_raw -> safety_obstacle -> /cmd_vel")
        print("        -> cmd_vel_odom_relay -> /diff_drive_controller/cmd_vel -> gz_ros2_control")
        print("  Odom: diff_drive_controller -> /diff_drive_controller/odom + TF odom->base_footprint")
        print("        topic_tools relay -> /odom (1:1, no integration)")
        print("  Map:  RTAB-Map -> map->odom only")
        print()
        print("--- Rotation test (sim must be running; stop Nav2 for clean test) ---")
        print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
              '"{linear: {x: 0.0}, angular: {z: 0.7}}" -r 10')
        print("  Or through safety: pub to /cmd_vel_raw with Nav2+safety running.")
        print()
        if odom_yaw0 is not None and gz_yaw is not None:
            diff = abs(math.degrees(odom_yaw0 - gz_yaw))
            if diff > 15.0:
                print(
                    f"WARN: odom TF yaw vs Gazebo yaw differ by {diff:.1f} deg — "
                    "check cmd_vel signs / wheel contact / duplicate odom sources."
                )
            else:
                print("OK: odom TF yaw roughly matches Gazebo (within 15 deg).")
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    rclpy.init(args=argv)
    node = SimDriveDiagnose()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
