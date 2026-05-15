#!/usr/bin/env python3
"""6WD physics test: Gazebo ground truth required (earth). /odom logged only."""

from __future__ import annotations

import math
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from moonmapper_nav2.gz_pose_util import (
    GazeboPoseReader,
    GzPose,
    is_stable_pose,
    unwrap_yaw_delta,
    yaw_from_quat,
)
from moonmapper_nav2.rclpy_shutdown import safe_shutdown

try:
    from rclpy.wait_for_message import wait_for_message
except ImportError:
    wait_for_message = None  # type: ignore

WHEEL_JOINTS = [
    "wheel_l1_joint", "wheel_l2_joint", "wheel_l3_joint",
    "wheel_r1_joint", "wheel_r2_joint", "wheel_r3_joint",
]

MOTION_CASES: Dict[str, Tuple[float, float, float]] = {
    "forward": (0.12, 0.0, 8.0),
    "rotate": (0.0, 0.7, 10.0),
    "arc": (0.08, 0.35, 8.0),
}

ALL_CASES_ORDER = ("stability", "forward", "rotate", "arc")

_THIS_NODE = "sim_6wd_motion_test"
_ODOM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
_CMD_VEL_HZ_MIN = 10.0
_GZ_SAMPLE_INTERVAL_SEC = 0.5
_MAX_ROLL_PITCH_DEG = 25.0


def _topic_publishers(topic: str) -> List[str]:
    try:
        out = subprocess.run(
            ["ros2", "topic", "info", topic, "-v"],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    pubs: List[str] = []
    section = ""
    for line in out.stdout.splitlines():
        s = line.strip()
        if s.startswith("Publisher count:"):
            section = "pub"
            continue
        if s.startswith("Subscription count:"):
            section = ""
            continue
        if s.startswith("Node name:") and section == "pub":
            pubs.append(s.split(":", 1)[1].strip())
    return pubs


def _odom_xy_yaw(msg: Odometry) -> Tuple[float, float, float]:
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return p.x, p.y, yaw_from_quat(o.x, o.y, o.z, o.w)


def _scan_use_sim_time(tokens: List[str], default: bool = True) -> bool:
    for i, a in enumerate(tokens):
        if a.startswith("use_sim_time:="):
            return "true" in a.lower()
        if a == "-p" and i + 1 < len(tokens) and tokens[i + 1].startswith("use_sim_time:="):
            return "true" in tokens[i + 1].lower()
    return default


def _collect_ros_param_tokens(tokens: List[str]) -> List[str]:
    out: List[str] = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t in ("--case", "--environment"):
            i += 2 if i + 1 < len(tokens) else 1
            continue
        if t == "--":
            i += 1
            continue
        if t == "--ros-args":
            i += 1
            while i < len(tokens) and tokens[i] not in ("--",):
                if tokens[i] in ("--case", "--environment"):
                    break
                out.append(tokens[i])
                i += 1
            continue
        if t in ("-p", "--param") and i + 1 < len(tokens):
            name_val = tokens[i + 1]
            if not name_val.startswith(("--case", "--environment")):
                out.extend([t, name_val])
            i += 2
            continue
        if t.startswith("-p") and ":=" in t:
            out.append(t)
        i += 1
    return out


def _build_rclpy_init_argv(raw_argv: List[str]) -> Tuple[List[str], List[str]]:
    if "--" in raw_argv:
        dash = raw_argv.index("--")
        before, after = raw_argv[:dash], raw_argv[dash + 1 :]
    else:
        before, after = raw_argv, []
    all_tokens = before + after
    use_sim = _scan_use_sim_time(all_tokens, default=True)
    param_tokens = _collect_ros_param_tokens(all_tokens)
    has_use_sim = any("use_sim_time" in t for t in param_tokens)
    init_argv = [_THIS_NODE, "--ros-args"]
    if not has_use_sim:
        init_argv.extend(["-p", f"use_sim_time:={'true' if use_sim else 'false'}"])
    init_argv.extend(param_tokens)
    return init_argv, all_tokens


def _pop_flag(argv: List[str], flag: str) -> Tuple[List[str], Optional[str]]:
    if flag not in argv:
        return argv, None
    idx = argv.index(flag)
    value = argv[idx + 1] if idx + 1 < len(argv) else None
    return argv[:idx] + argv[idx + 2 :], value


def _world_for_environment(environment: str) -> str:
    return "earth_arena" if environment == "earth" else "moon_arena"


def _print_pose_line(label: str, p: GzPose) -> None:
    print(
        f"  {label}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
        f"roll={math.degrees(p.roll):.1f}° pitch={math.degrees(p.pitch):.1f}° "
        f"yaw={math.degrees(p.yaw):.1f}°"
    )


def _gz_metrics(poses: List[GzPose]) -> Tuple[Optional[float], Optional[float], List[GzPose]]:
    if len(poses) < 1:
        return None, None, poses
    yaws = [p.yaw for p in poses]
    gdyaw = unwrap_yaw_delta(yaws) if len(yaws) >= 2 else 0.0
    gdist = math.hypot(poses[-1].x - poses[0].x, poses[-1].y - poses[0].y)
    return gdist, gdyaw, poses


def _check_stability_series(poses: List[GzPose]) -> Tuple[bool, str]:
    if not poses:
        return False, "no Gazebo pose samples"
    for p in poses:
        ok, msg = is_stable_pose(p, max_roll_deg=_MAX_ROLL_PITCH_DEG, max_pitch_deg=_MAX_ROLL_PITCH_DEG)
        if not ok:
            return False, msg
    return True, "ok"


class Sim6wdMotionTest(Node):
    def __init__(self) -> None:
        super().__init__("sim_6wd_motion_test")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("world_name", "earth_arena")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("min_odom_samples", 3)
        self.declare_parameter("stability_wait_sec", 5.0)
        self._reader: Optional[GazeboPoseReader] = None
        self._environment = "earth"
        self._odom_yaw_series: List[float] = []
        self._odom_xy_series: List[Tuple[float, float]] = []
        self._command_active = False
        self._active_twist: Optional[Twist] = None
        self._cmd_timer = None
        self._wheel_left_sum = 0.0
        self._wheel_right_sum = 0.0
        self._wheel_sample_count = 0
        self._max_wheel_speed = 0.0
        self.create_subscription(Odometry, "/odom", self._on_odom, _ODOM_QOS)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def set_gz_reader(self, reader: GazeboPoseReader) -> None:
        self._reader = reader

    def _reset_case_state(self) -> None:
        self._odom_yaw_series = []
        self._odom_xy_series = []
        self._command_active = False
        self._wheel_left_sum = 0.0
        self._wheel_right_sum = 0.0
        self._wheel_sample_count = 0
        self._max_wheel_speed = 0.0

    def _on_odom(self, msg: Odometry) -> None:
        if not self._command_active:
            return
        x, y, yaw = _odom_xy_yaw(msg)
        self._odom_yaw_series.append(yaw)
        self._odom_xy_series.append((x, y))

    def _on_js(self, msg: JointState) -> None:
        if not self._command_active:
            return
        m = dict(zip(msg.name, msg.velocity))
        left = [m[j] for j in WHEEL_JOINTS[:3] if j in m]
        right = [m[j] for j in WHEEL_JOINTS[3:] if j in m]
        if left and right:
            self._wheel_left_sum += sum(left) / len(left)
            self._wheel_right_sum += sum(right) / len(right)
            self._wheel_sample_count += 1
        for jn in WHEEL_JOINTS:
            v = m.get(jn)
            if v is not None:
                self._max_wheel_speed = max(self._max_wheel_speed, abs(v))

    def _wheel_means(self) -> Tuple[Optional[float], Optional[float]]:
        if self._wheel_sample_count == 0:
            return None, None
        n = float(self._wheel_sample_count)
        return self._wheel_left_sum / n, self._wheel_right_sum / n

    def _check_cmd_vel_publishers(self) -> bool:
        pubs = _topic_publishers("/cmd_vel")
        others = [p for p in pubs if p != _THIS_NODE]
        if others:
            print(
                "Existing /cmd_vel publisher detected "
                f"({', '.join(others)}). Stop external sources before motion test."
            )
            return False
        return True

    def _preflight_odom(self, timeout_sec: float = 5.0) -> bool:
        pubs = _topic_publishers("/odom")
        if not pubs:
            print(
                "FAIL: no /odom publisher — start Gazebo sim first:\n"
                "  ros2 launch moonmapper_bringup sim_rover_clean.launch.py "
                "world_preset:=earth physics_profile:=earth_stable_6wd"
            )
            return False
        print(f"  /odom publishers: {', '.join(pubs)} (logged only, not PASS/FAIL)")
        if wait_for_message is None:
            return True
        ok, _ = wait_for_message(
            Odometry, self, "/odom", time_to_wait=timeout_sec, qos_profile=_ODOM_QOS
        )
        if not ok:
            print("FAIL: no /odom message (sim running? use_sim_time:=true)")
            return False
        return True

    def _require_gz(self) -> bool:
        if self._reader is None or not self._reader.ready:
            print(
                "FAIL: Gazebo ground truth unavailable. "
                "Physics test cannot pass using /odom only.\n"
                "  /odom integrates wheel speeds and ignores slip/tipping.\n"
                "  Start sim, then re-run. Check gz with: gz model -m moonmapper -p"
            )
            if self._reader is not None:
                self._reader.print_debug()
            return False
        return True

    def _collect_gz_poses(self, duration_sec: float, publish_twist: Optional[Twist]) -> List[GzPose]:
        assert self._reader is not None
        poses: List[GzPose] = []
        rate = max(float(self.get_parameter("rate_hz").value), _CMD_VEL_HZ_MIN)
        period = 1.0 / rate
        if publish_twist is not None:
            self._start_cmd_timer(publish_twist, rate)
            self._command_active = True
        t_end = time.monotonic() + duration_sec
        last_gz = 0.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_gz >= _GZ_SAMPLE_INTERVAL_SEC:
                p = self._reader.sample()
                if p is not None:
                    poses.append(p)
                last_gz = now
        if publish_twist is not None:
            self._command_active = False
            self._stop_cmd_timer()
            stop = Twist()
            for _ in range(int(rate)):
                self._pub.publish(stop)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(period)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        p = self._reader.sample()
        if p is not None:
            poses.append(p)
        return poses

    def _cmd_timer_cb(self) -> None:
        if self._active_twist is not None:
            self._pub.publish(self._active_twist)

    def _start_cmd_timer(self, twist: Twist, rate_hz: float) -> None:
        period = 1.0 / max(rate_hz, _CMD_VEL_HZ_MIN)
        self._active_twist = twist
        if self._cmd_timer is not None:
            self.destroy_timer(self._cmd_timer)
        self._cmd_timer = self.create_timer(period, self._cmd_timer_cb)

    def _stop_cmd_timer(self) -> None:
        self._active_twist = None
        if self._cmd_timer is not None:
            self.destroy_timer(self._cmd_timer)
            self._cmd_timer = None

    def run_stability(self) -> bool:
        print("\n=== Motion test: stability (no cmd_vel, wait) ===")
        if not self._require_gz():
            return False
        wait_sec = float(self.get_parameter("stability_wait_sec").value)
        poses = self._collect_gz_poses(wait_sec, None)
        print(f"  gz samples: {len(poses)}")
        if poses:
            _print_pose_line("first", poses[0])
            _print_pose_line("last", poses[-1])
        ok, msg = _check_stability_series(poses)
        if not ok:
            print(f"FAIL: {msg}")
            return False
        print("PASS: robot stable on wheels before motion tests")
        return True

    def run_case(self, case: str, lin_x: float, ang_z: float, duration: float) -> bool:
        if not self._require_gz():
            return False
        strict = self._environment == "earth"
        print(f"\n=== Motion test: {case} (lin.x={lin_x}, ang.z={ang_z}, {duration}s) ===")
        self._reset_case_state()

        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z

        if wait_for_message is not None:
            ok, _ = wait_for_message(
                Odometry, self, "/odom", time_to_wait=5.0, qos_profile=_ODOM_QOS
            )
            if not ok:
                print("FAIL: no /odom before command")
                return False

        gz_poses = self._collect_gz_poses(duration, twist)
        print(f"  gz samples: {len(gz_poses)}")
        if gz_poses:
            _print_pose_line("gz first", gz_poses[0])
            _print_pose_line("gz last", gz_poses[-1])

        stab_ok, stab_msg = _check_stability_series(gz_poses)
        if not stab_ok:
            print(f"FAIL: {stab_msg}")
            return False

        n_odom = len(self._odom_yaw_series)
        odom_dyaw = unwrap_yaw_delta(self._odom_yaw_series) if n_odom >= 2 else 0.0
        odom_dist = 0.0
        if n_odom >= 2:
            ox0, oy0 = self._odom_xy_series[0]
            ox1, oy1 = self._odom_xy_series[-1]
            odom_dist = math.hypot(ox1 - ox0, oy1 - oy0)
        print(
            f"  odom (info only): n={n_odom} dist={odom_dist:.3f} m "
            f"dyaw={math.degrees(odom_dyaw):.1f}°"
        )

        gdist, gdyaw, _ = _gz_metrics(gz_poses)
        if gdist is None or gdyaw is None:
            print("FAIL: Gazebo ground truth unavailable during motion")
            return False
        print(
            f"  gz (PASS/FAIL): dist={gdist:.3f} m "
            f"dyaw(unwrapped)={math.degrees(gdyaw):.1f}°"
        )

        wl, wr = self._wheel_means()
        if wl is not None and wr is not None:
            print(
                f"  wheels: L_mean={wl:.3f} R_mean={wr:.3f} rad/s  "
                f"max|w|={self._max_wheel_speed:.3f}"
            )

        if not strict:
            if abs(odom_dyaw) > math.radians(20.0) and abs(gdyaw) < abs(odom_dyaw) * 0.5:
                print(
                    f"  moon: odom/gz yaw slip ratio "
                    f"{abs(gdyaw / odom_dyaw) if odom_dyaw else 0:.2f}"
                )
            print("MOON DIAGNOSTIC (gz logged; soft thresholds)")
            return True

        passed = True
        if case == "forward":
            if gdist < 0.4:
                print(f"FAIL: forward gz dist {gdist:.3f} < 0.4 m")
                passed = False
            if abs(gdyaw) > math.radians(20.0):
                print(f"FAIL: forward gz |yaw| {math.degrees(abs(gdyaw)):.1f}° > 20°")
                passed = False
        elif case == "rotate":
            if abs(gdyaw) < math.radians(90.0):
                print(f"FAIL: rotate gz |dyaw| {math.degrees(abs(gdyaw)):.1f}° < 90°")
                passed = False
            if gdist > 0.25:
                print(f"FAIL: rotate gz drift {gdist:.3f} m > 0.25 m")
                passed = False
            if wl is not None and wr is not None and wl * wr > 0.0:
                print("FAIL: wheels same sign (expected opposite for spin)")
                passed = False
            if abs(odom_dyaw) >= math.radians(90.0) and abs(gdyaw) < math.radians(45.0):
                print(
                    "FAIL: odom/gazebo yaw mismatch — likely slip/contact/world physics "
                    f"(odom {math.degrees(odom_dyaw):.1f}° gz {math.degrees(gdyaw):.1f}°)"
                )
                passed = False
        elif case == "arc":
            if gdist < 0.2:
                print(f"FAIL: arc gz dist {gdist:.3f} < 0.2 m")
                passed = False
            if abs(gdyaw) < math.radians(30.0):
                print(f"FAIL: arc gz |dyaw| {math.degrees(abs(gdyaw)):.1f}° < 30°")
                passed = False
            if abs(odom_dyaw) >= math.radians(30.0) and abs(gdyaw) < math.radians(20.0):
                print(
                    "FAIL: odom/gazebo yaw mismatch — "
                    f"odom {math.degrees(odom_dyaw):.1f}° gz {math.degrees(gdyaw):.1f}°"
                )
                passed = False

        print("PASS" if passed else "FAIL")
        return passed


def main(argv: Optional[List[str]] = None) -> int:
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    init_argv, flag_tokens = _build_rclpy_init_argv(raw_argv)
    flag_tokens, environment = _pop_flag(flag_tokens, "--environment")
    environment = (environment or "earth").strip().lower()
    if environment not in ("earth", "moon"):
        print(f"Unknown --environment '{environment}'", file=sys.stderr)
        return 2

    flag_tokens, case = _pop_flag(flag_tokens, "--case")
    case = (case or "all").strip().lower()

    rclpy.init(args=init_argv)
    node = Sim6wdMotionTest()
    node._environment = environment
    world_name = _world_for_environment(environment)
    node.set_parameters([Parameter("world_name", Parameter.Type.STRING, world_name)])

    reader = GazeboPoseReader(requested_world=world_name, model_name="moonmapper")
    node.set_gz_reader(reader)

    try:
        print("=== 6WD motion test (Gazebo ground truth required for earth PASS) ===")
        print(
            "  ros2 run moonmapper_nav2 sim_6wd_motion_test "
            "--ros-args -p use_sim_time:=true -- --case all --environment earth"
        )
        print(
            "  Previous false PASS: gz unavailable + fallback to /odom. "
            "/odom is NOT physical truth under slip/tipping."
        )
        reader.print_debug()

        if environment == "earth" and not reader.ready:
            print(
                "\nFAIL: Gazebo ground truth unavailable. "
                "Physics test cannot pass using /odom only."
            )
            return 2

        if not node._check_cmd_vel_publishers():
            return 2
        if not node._preflight_odom():
            return 2

        if case == "all":
            cases = list(ALL_CASES_ORDER)
        elif case == "stability":
            cases = ["stability"]
        elif case in MOTION_CASES:
            cases = [case]
        else:
            print(f"Unknown case '{case}'", file=sys.stderr)
            return 2

        results = []
        for name in cases:
            if name == "stability":
                ok = node.run_stability()
            else:
                lin, ang, dur = MOTION_CASES[name]
                ok = node.run_case(name, lin, ang, dur)
            results.append((name, ok))
            if name == "stability" and not ok and case == "all":
                print("\nAborting remaining cases (stability failed).")
                break

        print("\n=== Summary ===")
        for name, ok in results:
            label = "PASS" if ok else ("DIAG" if environment == "moon" else "FAIL")
            print(f"  {name}: {label}")
        if environment == "moon":
            return 0
        ran = {n for n, _ in results}
        required = [n for n in cases if n in ran]
        return 0 if all(ok for n, ok in results if n in required) else 1
    finally:
        node.destroy_node()
        safe_shutdown()


if __name__ == "__main__":
    sys.exit(main())
