#!/usr/bin/env python3
"""Periodic TF chain logging for map / odom / base_footprint (static-map Nav2 debug)."""

from __future__ import annotations

import math
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node

from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown
from rclpy.time import Time


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _fmt(t: TransformStamped) -> str:
    tr = t.transform.translation
    yaw = _yaw_from_quat(t.transform.rotation)
    return f"xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) yaw={math.degrees(yaw):.1f}°"


class TfChainLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_tf_chain_logger")
        self.declare_parameter("period_sec", 5.0)
        self.declare_parameter("base_frame", "base_footprint")
        period = max(1.0, self.get_parameter("period_sec").get_parameter_value().double_value)
        self._base = self.get_parameter("base_frame").get_parameter_value().string_value
        self._buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._timer = self.create_timer(period, self._log_chain)
        self.get_logger().info(
            f"TF chain logger: every {period:.1f}s logs map->odom, odom->{self._base}, map->{self._base}"
        )

    def _lookup(self, parent: str, child: str) -> str:
        try:
            t = self._buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
            return _fmt(t)
        except tf2_ros.TransformException as ex:
            return f"MISSING ({ex})"

    def _log_chain(self) -> None:
        if not rclpy.ok():
            return
        map_odom = self._lookup("map", "odom")
        odom_base = self._lookup("odom", self._base)
        map_base = self._lookup("map", self._base)
        self.get_logger().info(f"TF map->odom: {map_odom}")
        self.get_logger().info(f"TF odom->{self._base}: {odom_base}")
        self.get_logger().info(f"TF map->{self._base}: {map_base}")


def main() -> int:
    rclpy.init()
    node: TfChainLoggerNode | None = None
    try:
        node = TfChainLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        safe_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
