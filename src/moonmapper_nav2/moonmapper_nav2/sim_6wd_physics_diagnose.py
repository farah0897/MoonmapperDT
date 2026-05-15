#!/usr/bin/env python3
"""6WD Gazebo physics / diff_drive chain diagnostics."""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None  # type: ignore

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.time import Time
from sensor_msgs.msg import JointState

from moonmapper_nav2.gz_pose_util import gz_model_pose_xy_yaw
from moonmapper_nav2.rclpy_shutdown import safe_shutdown
from moonmapper_nav2.sim_6wd_motion_test import _build_rclpy_init_argv

try:
    from rclpy.wait_for_message import wait_for_message
except ImportError:
    wait_for_message = None  # type: ignore

WHEEL_JOINTS = [
    "wheel_l1_joint", "wheel_l2_joint", "wheel_l3_joint",
    "wheel_r1_joint", "wheel_r2_joint", "wheel_r3_joint",
]


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


def _controller_list() -> str:
    try:
        out = subprocess.run(
            ["ros2", "control", "list_controllers", "-c", "/controller_manager"],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
        return out.stdout.strip() or out.stderr.strip() or "(no output)"
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return f"(unavailable: {exc})"


def _param_get(node_name: str, name: str) -> Optional[str]:
    try:
        proc = subprocess.run(
            ["ros2", "param", "get", node_name, name],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        if proc.returncode == 0:
            return proc.stdout.strip()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass
    return None


def _diff_drive_params() -> Dict[str, str]:
    names = (
        "wheel_separation", "wheel_radius",
        "base_frame_id", "odom_frame_id", "enable_odom_tf",
    )
    out: Dict[str, str] = {}
    for name in names:
        val = _param_get("/diff_drive_controller", name)
        if val is not None:
            out[name] = val
    return out


def _parse_world_sdf(path: str) -> Tuple[str, str, str]:
    gravity_s = "(unknown)"
    mu_s, mu2_s = "(unknown)", "(unknown)"
    try:
        root = ET.parse(path).getroot()
        grav = root.find(".//gravity")
        if grav is not None and grav.text:
            gravity_s = grav.text.strip()
        for friction in root.findall(".//model[@name='ground_plane']//friction"):
            ode = friction.find("ode")
            if ode is None:
                continue
            mu = ode.find("mu")
            mu2 = ode.find("mu2")
            if mu is not None and mu.text:
                mu_s = mu.text.strip()
            if mu2 is not None and mu2.text:
                mu2_s = mu2.text.strip()
            break
    except (ET.ParseError, OSError) as exc:
        gravity_s = f"(parse error: {exc})"
    return gravity_s, mu_s, mu2_s


def _gravity_z(gravity_s: str) -> Optional[float]:
    parts = gravity_s.split()
    if len(parts) >= 3:
        try:
            return float(parts[2])
        except ValueError:
            pass
    return None


def _resolve_world_sdf_path(world_name: str, explicit: str) -> Optional[str]:
    if explicit.strip():
        return explicit.strip()
    env = os.environ.get("MOONMAPPER_WORLD_SDF", "").strip()
    if env:
        return env
    if get_package_share_directory is None:
        return None
    try:
        share = get_package_share_directory("moonmapper_description")
        candidate = os.path.join(share, "worlds", f"{world_name}.sdf")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    return None


def _parse_double(raw: str) -> Optional[float]:
    # ros2 param get prints: "Double value is: 0.217112" or "Boolean value is: true"
    for token in raw.replace(",", " ").split():
        try:
            return float(token)
        except ValueError:
            continue
    return None


class Sim6wdPhysicsDiagnose(Node):
    def __init__(self) -> None:
        super().__init__("sim_6wd_physics_diagnose")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("world_name", "moon_arena")
        self.declare_parameter("world_sdf_path", "")
        self.declare_parameter("physics_profile", "")
        self.declare_parameter("wheel_mu1", "")
        self.declare_parameter("wheel_mu2", "")
        self.declare_parameter("sample_sec", 1.5)
        self.declare_parameter("angular_z_check", 0.7)
        self._last_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState) -> None:
        self._last_js = msg

    def run(self) -> int:
        world = str(self.get_parameter("world_name").value)
        sdf_path = _resolve_world_sdf_path(
            world, str(self.get_parameter("world_sdf_path").value)
        )
        profile = str(self.get_parameter("physics_profile").value).strip()
        if not profile:
            profile = os.environ.get("MOONMAPPER_PHYSICS_PROFILE", "").strip() or "(unknown)"
        wheel_mu1 = str(self.get_parameter("wheel_mu1").value).strip()
        wheel_mu2 = str(self.get_parameter("wheel_mu2").value).strip()
        sample = float(self.get_parameter("sample_sec").value)
        ang_z_check = float(self.get_parameter("angular_z_check").value)

        print("=== MoonMapper 6WD physics diagnose ===")
        print(f"world={world}  sample_sec={sample}")
        print()
        print("--- World SDF / gravity / ground friction ---")
        if sdf_path:
            print(f"  world_sdf={sdf_path}")
            grav_s, g_mu, g_mu2 = _parse_world_sdf(sdf_path)
            print(f"  gravity={grav_s}  ground_mu={g_mu}  ground_mu2={g_mu2}")
            gz = _gravity_z(grav_s)
            if gz is not None and -2.5 < gz < -0.5:
                print(
                    "  WARNING: lunar-like gravity (|g_z| ~ 1.62) — "
                    "low normal force → wheel slip; use world_preset:=earth for validation."
                )
        else:
            print("  (world_sdf_path / MOONMAPPER_WORLD_SDF / share worlds/ not found)")
        print(f"  physics_profile={profile}")
        if wheel_mu1 or wheel_mu2:
            print(f"  wheel_mu (params): mu1={wheel_mu1 or '?'}  mu2={wheel_mu2 or '?'}")
        else:
            print(
                "  wheel contact: cylinder r=0.045 m L=0.039 m, collision rpy=pi/2 0 0, "
                "joint axis Y; xacro wheel_mu* from physics_profile (see launch log)"
            )
        print()

        print("--- ros2_control controllers ---")
        print(_controller_list())
        print()

        for topic in (
            "/cmd_vel",
            "/diff_drive_controller/cmd_vel",
            "/odom",
            "/diff_drive_controller/odom",
            "/joint_states",
        ):
            pubs, subs = _topic_endpoints(topic)
            print(f"--- {topic} ---")
            print(f"  publishers:  {pubs or '(none)'}")
            print(f"  subscribers: {subs or '(none)'}")
        print()

        print("--- diff_drive_controller params ---")
        params = _diff_drive_params()
        if params:
            for k, v in sorted(params.items()):
                print(f"  {k}: {v}")
            sep = _parse_double(params.get("wheel_separation", ""))
            rad = _parse_double(params.get("wheel_radius", ""))
            if sep is not None and rad is not None and rad > 0.0:
                w_exp = ang_z_check * sep / (2.0 * rad)
                print(
                    f"  expected |wheel_speed| @ angular.z={ang_z_check}: "
                    f"{w_exp:.3f} rad/s  (omega*sep/(2*r))"
                )
            enable_tf = params.get("enable_odom_tf", "")
            if enable_tf:
                print(f"  (enable_odom_tf={enable_tf} → TF odom→base_footprint from controller)")
        else:
            print("  (controller not running or params unavailable)")
        print()

        t0 = time.monotonic()
        while time.monotonic() - t0 < sample:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("--- wheel joint velocities (rad/s) ---")
        if self._last_js is None:
            print("  (no /joint_states yet)")
        else:
            name_to_vel = dict(zip(self._last_js.name, self._last_js.velocity))
            for jn in WHEEL_JOINTS:
                v = name_to_vel.get(jn)
                print(f"  {jn}: {v if v is not None else '—'}")
            left = [name_to_vel.get(j) for j in WHEEL_JOINTS[:3] if j in name_to_vel]
            right = [name_to_vel.get(j) for j in WHEEL_JOINTS[3:] if j in name_to_vel]
            if left and right:
                print(
                    f"  L mean={sum(left)/len(left):.3f}  R mean={sum(right)/len(right):.3f}  "
                    f"(expect opposite signs under pure rotation cmd)"
                )
        print()

        print("--- TF odom -> base_footprint ---")
        buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        tf2_ros.TransformListener(buf, self, spin_thread=True)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf_odom = buf.lookup_transform(
                "odom", "base_footprint", Time(), timeout=Duration(seconds=2.0)
            )
            q = tf_odom.transform.rotation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            print(
                f"  OK  x={tf_odom.transform.translation.x:.4f} "
                f"y={tf_odom.transform.translation.y:.4f} "
                f"yaw={math.degrees(yaw):.2f} deg"
            )
        except tf2_ros.TransformException as exc:
            print(f"  MISSING: {exc}")
            if params.get("enable_odom_tf", "").lower().find("false") >= 0:
                print("  hint: enable_odom_tf is false — no TF from diff_drive_controller")
        print()

        print("--- /odom sample ---")
        if wait_for_message is not None:
            ok, odom = wait_for_message(
                Odometry, self, "/odom", time_to_wait=3.0, qos_profile=qos_profile_default
            )
            if ok and odom is not None:
                q = odom.pose.pose.orientation
                yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
                print(
                    f"  pose yaw={math.degrees(yaw):.2f} deg  "
                    f"twist lin.x={odom.twist.twist.linear.x:.4f} "
                    f"ang.z={odom.twist.twist.angular.z:.4f}"
                )
            else:
                print("  no /odom within timeout")
        print()

        print("--- Gazebo ground truth (gz pose) ---")
        gz = gz_model_pose_xy_yaw(world=world)
        if gz is None:
            print("  (gz topic unavailable)")
        else:
            x, y, yaw = gz
            print(f"  moonmapper  x={x:.4f} y={y:.4f} yaw={math.degrees(yaw):.2f} deg")
        print()

        print("--- Expected chain ---")
        print("  /cmd_vel -> cmd_vel_odom_relay -> /diff_drive_controller/cmd_vel")
        print("  -> gz_ros2_control -> 6 wheel velocity interfaces")
        print("  /diff_drive_controller/odom + TF -> topic_tools relay -> /odom")
        print(
            "  physics_profile: moon→safe_6wd, earth→earth_6wd "
            "(sim_rover_clean world_preset:=earth|moon)"
        )
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    init_argv, _ = _build_rclpy_init_argv(raw_argv)
    rclpy.init(args=init_argv)
    node = Sim6wdPhysicsDiagnose()
    try:
        return node.run()
    finally:
        node.destroy_node()
        safe_shutdown()


if __name__ == "__main__":
    sys.exit(main())
