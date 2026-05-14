#!/usr/bin/env python3
"""2D-trilateration fra fire UWB-avstander (least squares, 3+ anker paakrevd)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range


class UwbTrilaterationNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_trilateration")

        self.declare_parameter("anchors_file", "")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("output_topic", "/uwb/pose")
        self.declare_parameter("range_topics", ["/uwb/range/anchor_1", "/uwb/range/anchor_2", "/uwb/range/anchor_3", "/uwb/range/anchor_4"])
        self.declare_parameter("range_timeout_sec", 0.5)
        self.declare_parameter("fixed_z", 0.0)
        self.declare_parameter("position_variance_xy", 0.01)  # 0.1^2 som utgangspunkt
        self.declare_parameter("yaw_variance", 10.0)

        af = str(self.get_parameter("anchors_file").get_parameter_value().string_value).strip()
        if not af:
            from ament_index_python.packages import get_package_share_directory

            af = str(
                Path(get_package_share_directory("moonmapper_bringup")) / "config" / "uwb_anchors.yaml",
            )
        doc = yaml.safe_load(Path(af).read_text(encoding="utf-8"))
        self._anchors = self._anchors_xy(doc.get("uwb_anchors") or {})

        topics = list(self.get_parameter("range_topics").get_parameter_value().string_array_value)
        if len(topics) < 3:
            topics = ["/uwb/range/anchor_1", "/uwb/range/anchor_2", "/uwb/range/anchor_3", "/uwb/range/anchor_4"]
        self._range_last: dict[str, tuple[Any, Range]] = {}
        for i, t in enumerate(topics):
            key = "anchor_%d" % (i + 1)
            self.create_subscription(Range, t, self._make_cb(key), 10)

        out = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(PoseWithCovarianceStamped, out, 10)
        self._out_frame = str(self.get_parameter("output_frame_id").value)
        self._timeout = float(self.get_parameter("range_timeout_sec").value)
        self._fixed_z = float(self.get_parameter("fixed_z").value)
        self._var_xy = float(self.get_parameter("position_variance_xy").value)
        self._var_yaw = float(self.get_parameter("yaw_variance").value)

        period = 1.0 / 20.0
        self.create_timer(period, self._tick)
        self.get_logger().info("uwb_trilateration: ut-topic=%s frame=%s" % (out, self._out_frame))

    def _anchors_xy(self, ua: dict[str, Any]) -> dict[str, tuple[float, float]]:
        out: dict[str, tuple[float, float]] = {}
        for name, row in ua.items():
            if not isinstance(row, dict):
                continue
            out[str(name)] = (float(row.get("x", 0.0)), float(row.get("y", 0.0)))
        return out

    def _make_cb(self, key: str):
        def _cb(msg: Range) -> None:
            self._range_last[key] = (Time.from_msg(msg.header.stamp), msg)

        return _cb

    def _tick(self) -> None:
        now = self.get_clock().now()
        used: list[tuple[str, float, float, float]] = []
        stale = False
        for key, (ax, ay) in self._anchors.items():
            if key not in self._range_last:
                continue
            t_msg, msg = self._range_last[key]
            age_sec = (now - t_msg).nanoseconds * 1e-9
            if age_sec > self._timeout:
                stale = True
                continue
            r = float(msg.range)
            if not math.isfinite(r) or r <= 0.0:
                continue
            used.append((key, ax, ay, r))
        if stale and len(used) >= 3:
            self.get_logger().warning("Noen UWB-malinger er eldre enn timeout (%.2f s)" % self._timeout)

        if len(used) < 3:
            return

        # Referanse-anker = foerste i «used» (stabil rekkefoelge etter yaml-nokler)
        used.sort(key=lambda x: x[0])
        x0, y0, r0 = used[0][1], used[0][2], used[0][3]
        rows = []
        rhs = []
        for _k, xi, yi, ri in used[1:]:
            rows.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
            rhs.append(
                r0 * r0 - ri * ri + xi * xi - x0 * x0 + yi * yi - y0 * y0,
            )
        a_mat = np.array(rows, dtype=float)
        b_vec = np.array(rhs, dtype=float)
        sol, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
        x, y = float(sol[0]), float(sol[1])

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = self._fixed_z
        out.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        cov = [0.0] * 36
        cov[0] = self._var_xy
        cov[7] = self._var_xy
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = self._var_yaw
        out.pose.covariance = cov

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = UwbTrilaterationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
