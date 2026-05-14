#!/usr/bin/env python3
"""Simuler BU04/UWB-avstander fra robot-tag til faste anker (Gazebo ground truth -> sensor).

Gazebo-modellpose brukes KUN til aa beregne syntetiske avstander. Den skal ikke mates
inn i Nav2. Vi bruker sensor_msgs/Range som generisk avstandsmelding (radiation_type
settes til INFRARED som «placeholder» — UWB er ikke en egen type i Range-enum).
"""

from __future__ import annotations

import math
import random
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Range


def _quat_rotate_vec(qx: float, qy: float, qz: float, qw: float, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    """Roter vektor (vx,vy,vz) med enhetsquaternion (qx,qy,qz,qw)."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    fx = vx + qw * tx + (qy * tz - qz * ty)
    fy = vy + qw * ty + (qz * tx - qx * tz)
    fz = vz + qw * tz + (qx * ty - qy * tx)
    return fx, fy, fz


class UwbRangeSimNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_range_sim")

        self.declare_parameter("anchors_file", "")
        self.declare_parameter("ground_truth_pose_topic", "/gz/moonmapper/pose")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("range_noise_std", 0.05)
        self.declare_parameter("range_bias", 0.02)
        self.declare_parameter("dropout_probability", 0.02)
        self.declare_parameter("outlier_probability", 0.01)
        self.declare_parameter("outlier_std", 0.30)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("enable_uwb_noise", True)

        af = str(self.get_parameter("anchors_file").get_parameter_value().string_value).strip()
        if not af:
            from ament_index_python.packages import get_package_share_directory

            af = str(
                Path(get_package_share_directory("moonmapper_bringup")) / "config" / "uwb_anchors.yaml",
            )
        self._anchors_path = Path(af)
        doc = yaml.safe_load(self._anchors_path.read_text(encoding="utf-8"))
        self._anchors = self._load_anchors_dict(doc.get("uwb_anchors") or {})
        tag = doc.get("robot_tag") or {}
        self._tag_off = (float(tag.get("x", 0.0)), float(tag.get("y", 0.0)), float(tag.get("z", 0.10)))

        self._gt_topic = str(self.get_parameter("ground_truth_pose_topic").value)
        self._rate_hz = float(self.get_parameter("update_rate_hz").value)
        self._noise_std = float(self.get_parameter("range_noise_std").value)
        self._bias = float(self.get_parameter("range_bias").value)
        self._p_drop = float(self.get_parameter("dropout_probability").value)
        self._p_out = float(self.get_parameter("outlier_probability").value)
        self._out_std = float(self.get_parameter("outlier_std").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._min_range = float(self.get_parameter("min_range").value)
        self._enable_noise = bool(self.get_parameter("enable_uwb_noise").value)
        seed = int(self.get_parameter("random_seed").value)
        self._rng = random.Random(seed)

        self._last_pose: Pose | None = None
        self.create_subscription(Pose, self._gt_topic, self._on_pose, 10)

        self._pubs: dict[str, Any] = {}
        for key, spec in self._anchors.items():
            topic = f"/uwb/range/{key}"
            self._pubs[key] = self.create_publisher(Range, topic, 10)
            self.get_logger().info("Publiserer %s (anker-frame=%s)" % (topic, spec["frame_id"]))

        period = 1.0 / max(0.1, self._rate_hz)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "uwb_range_sim: GT-topic=%s, stoy=%s, rate=%.2f Hz"
            % (self._gt_topic, self._enable_noise, self._rate_hz),
        )

    def _load_anchors_dict(self, ua: dict[str, Any]) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for name, row in ua.items():
            if not isinstance(row, dict):
                continue
            fid = str(row.get("frame_id", name))
            out[str(name)] = {
                "frame_id": fid,
                "x": float(row.get("x", 0.0)),
                "y": float(row.get("y", 0.0)),
                "z": float(row.get("z", 0.0)),
            }
        if len(out) < 4:
            self.get_logger().warning("Forventet 4 anker i yaml, fant %d" % len(out))
        return out

    def _on_pose(self, msg: Pose) -> None:
        self._last_pose = msg

    def _tick(self) -> None:
        pose = self._last_pose
        if pose is None:
            return

        rt = self.get_clock().now().to_msg()

        # Modellpose = base_footprint i verden; tag-offset i base_frame (samme rot som base).
        qx, qy, qz, qw = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        ox, oy, oz = self._tag_off
        tx, ty, tz = _quat_rotate_vec(qx, qy, qz, qw, ox, oy, oz)
        px = float(pose.position.x) + tx
        py = float(pose.position.y) + ty
        pz = float(pose.position.z) + tz

        for key, spec in self._anchors.items():
            if key not in self._pubs:
                continue
            if self._enable_noise and self._rng.random() < self._p_drop:
                continue

            ax, ay, az = spec["x"], spec["y"], spec["z"]
            dist = math.sqrt((px - ax) ** 2 + (py - ay) ** 2 + (pz - az) ** 2)

            if self._enable_noise:
                dist += self._bias
                dist += self._rng.gauss(0.0, self._noise_std)
                if self._rng.random() < self._p_out:
                    dist += self._rng.gauss(0.0, self._out_std)

            dist = max(self._min_range, min(self._max_range, dist))

            msg = Range()
            msg.header.stamp = rt
            msg.header.frame_id = spec["frame_id"]
            # INFRARED brukes som generisk «range»-plassholder; UWB er ikke egen type i sensor_msgs/Range.
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.0
            msg.min_range = float(self._min_range)
            msg.max_range = float(self._max_range)
            msg.range = float(dist)
            self._pubs[key].publish(msg)


def main() -> None:
    rclpy.init()
    node = UwbRangeSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
