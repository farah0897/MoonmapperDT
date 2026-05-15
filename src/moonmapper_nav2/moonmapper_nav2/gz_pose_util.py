"""Gazebo Sim ground-truth pose for 6WD physics tests (not /odom)."""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import List, Optional, Tuple

GZ_SUBPROCESS_TIMEOUT_SEC = 0.8


@dataclass
class GzPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    source: str = ""


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def unwrap_yaw_delta(yaws: List[float]) -> float:
    if len(yaws) < 2:
        return 0.0
    total = 0.0
    prev = yaws[0]
    for yaw in yaws[1:]:
        d = yaw - prev
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        total += d
        prev = yaw
    return total


def list_gz_pose_topics(timeout_sec: float = 2.0) -> List[str]:
    if shutil.which("gz") is None:
        return []
    try:
        out = subprocess.run(
            ["timeout", str(timeout_sec), "gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=timeout_sec + 0.5,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    topics: List[str] = []
    for line in out.stdout.splitlines():
        t = line.strip()
        if "/pose/info" in t or "/dynamic_pose/info" in t:
            topics.append(t.split()[0] if " " in t else t)
    return topics


def _worlds_from_topics(topics: List[str]) -> List[str]:
    worlds: List[str] = []
    for t in topics:
        m = re.match(r"/world/([^/]+)/(dynamic_pose|pose)/info", t)
        if m and m.group(1) not in worlds:
            worlds.append(m.group(1))
    return worlds


def _parse_pose_block(tail: str) -> Optional[Tuple[float, float, float, float, float, float]]:
    pos: dict[str, float] = {}
    ori: dict[str, float] = {}
    section = ""
    for line in tail.splitlines()[:50]:
        s = line.strip()
        if s.startswith("position"):
            section = "pos"
            continue
        if s.startswith("orientation"):
            section = "ori"
            continue
        for key in ("x", "y", "z", "w"):
            if s.startswith(f"{key}:"):
                val = float(s.split(":", 1)[1].strip())
                if section == "pos" and key in ("x", "y", "z"):
                    pos[key] = val
                elif section == "ori" and key in ("x", "y", "z", "w"):
                    ori[key] = val
    if "x" not in pos or "y" not in pos or "z" not in pos or "w" not in ori:
        return None
    roll, pitch, yaw = quat_to_rpy(
        ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori["w"]
    )
    return pos["x"], pos["y"], pos["z"], roll, pitch, yaw


def _pose_from_topic(
    topic: str,
    model_name: str,
    timeout_sec: float,
) -> Optional[GzPose]:
    try:
        out = subprocess.run(
            [
                "timeout",
                str(max(0.2, timeout_sec)),
                "gz",
                "topic",
                "-e",
                "-t",
                topic,
                "-n",
                "1",
            ],
            capture_output=True,
            text=True,
            timeout=timeout_sec + 0.4,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if out.returncode != 0 or not out.stdout.strip():
        return None
    text = out.stdout
    for marker in (f'name: "{model_name}"', f"name: '{model_name}'"):
        if marker in text:
            tail = text.split(marker, 1)[1]
            parsed = _parse_pose_block(tail)
            if parsed is not None:
                x, y, z, roll, pitch, yaw = parsed
                return GzPose(x, y, z, roll, pitch, yaw, source=topic)
    if f'name: "{model_name}"' not in text and "pose {" in text:
        for block in re.split(r"\n\s*pose\s*\{", text):
            if model_name not in block:
                continue
            parsed = _parse_pose_block(block)
            if parsed is not None:
                x, y, z, roll, pitch, yaw = parsed
                return GzPose(x, y, z, roll, pitch, yaw, source=topic)
    return None


def _pose_from_gz_model_cli(
    model_name: str,
    timeout_sec: float,
) -> Optional[GzPose]:
    try:
        out = subprocess.run(
            ["timeout", str(max(0.3, timeout_sec)), "gz", "model", "-m", model_name, "-p"],
            capture_output=True,
            text=True,
            timeout=timeout_sec + 0.5,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if out.returncode != 0:
        return None
    text = out.stdout + "\n" + out.stderr
    m = re.search(
        r"Pose\s*\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]\s*"
        r"\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]",
        text,
    )
    if not m:
        return None
    x, y, z, roll, pitch, yaw = (float(m.group(i)) for i in range(1, 7))
    return GzPose(x, y, z, roll, pitch, yaw, source="gz model -p")


@dataclass
class GazeboPoseReader:
    """Resolves Gazebo ground-truth source once; call sample() during tests."""

    requested_world: str
    model_name: str = "moonmapper"
    detected_world: Optional[str] = None
    pose_topic: Optional[str] = None
    mode: str = "none"
    available_topics: List[str] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self.available_topics = list_gz_pose_topics()
        worlds = _worlds_from_topics(self.available_topics)
        candidates: List[str] = []
        if self.requested_world:
            candidates.append(self.requested_world)
        for w in worlds:
            if w not in candidates:
                candidates.append(w)
        for w in ("earth_arena", "moon_arena", "default"):
            if w not in candidates:
                candidates.append(w)

        if _pose_from_gz_model_cli(self.model_name, GZ_SUBPROCESS_TIMEOUT_SEC) is not None:
            self.mode = "gz model -p"
            self.detected_world = self.requested_world or (worlds[0] if worlds else None)
            return

        for world in candidates:
            for suffix in ("dynamic_pose/info", "pose/info"):
                topic = f"/world/{world}/{suffix}"
                p = _pose_from_topic(topic, self.model_name, GZ_SUBPROCESS_TIMEOUT_SEC)
                if p is not None:
                    self.mode = "topic"
                    self.detected_world = world
                    self.pose_topic = topic
                    return

        self.mode = "none"

    @property
    def ready(self) -> bool:
        return self.mode != "none"

    def sample(self, timeout_sec: float = GZ_SUBPROCESS_TIMEOUT_SEC) -> Optional[GzPose]:
        if self.mode == "gz model -p":
            return _pose_from_gz_model_cli(self.model_name, timeout_sec)
        if self.mode == "topic" and self.pose_topic:
            return _pose_from_topic(self.pose_topic, self.model_name, timeout_sec)
        p = _pose_from_gz_model_cli(self.model_name, timeout_sec)
        if p is not None:
            self.mode = "gz model -p"
            return p
        for topic in self.available_topics or []:
            p = _pose_from_topic(topic, self.model_name, timeout_sec)
            if p is not None:
                self.mode = "topic"
                self.pose_topic = topic
                m = re.match(r"/world/([^/]+)/", topic)
                if m:
                    self.detected_world = m.group(1)
                return p
        return None

    def print_debug(self) -> None:
        print("--- Gazebo ground truth ---")
        print(f"  requested_world={self.requested_world}")
        print(f"  model_name={self.model_name}")
        print(f"  detected_world={self.detected_world or '(none)'}")
        print(f"  mode={self.mode}  topic={self.pose_topic or '(n/a)'}")
        if self.available_topics:
            print(f"  pose topics ({len(self.available_topics)}):")
            for t in self.available_topics[:8]:
                print(f"    {t}")
            if len(self.available_topics) > 8:
                print(f"    ... +{len(self.available_topics) - 8} more")
        else:
            print("  pose topics: (none — is Gazebo sim running?)")


def is_stable_pose(
    pose: GzPose,
    *,
    max_roll_deg: float = 25.0,
    max_pitch_deg: float = 25.0,
    min_z: float = 0.015,
    max_z: float = 0.35,
) -> Tuple[bool, str]:
    roll_d = math.degrees(abs(pose.roll))
    pitch_d = math.degrees(abs(pose.pitch))
    if roll_d > max_roll_deg or pitch_d > max_pitch_deg:
        return False, (
            f"robot unstable / tipped (roll={roll_d:.1f}° pitch={pitch_d:.1f}°)"
        )
    if pose.z < min_z or pose.z > max_z:
        return False, f"abnormal z={pose.z:.3f} m (expected ~0.03–0.25 upright)"
    return True, "ok"


# Back-compat
def gz_model_pose_xy_yaw(
    model_name: str = "moonmapper",
    world: str = "earth_arena",
    timeout_sec: float = GZ_SUBPROCESS_TIMEOUT_SEC,
) -> Optional[Tuple[float, float, float]]:
    reader = GazeboPoseReader(requested_world=world, model_name=model_name)
    p = reader.sample(timeout_sec)
    if p is None:
        return None
    return p.x, p.y, p.yaw  # noqa: RET504


def probe_gz_pose(world: str = "earth_arena", timeout_sec: float = 1.0) -> bool:
    return GazeboPoseReader(requested_world=world).sample(timeout_sec) is not None
