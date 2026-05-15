#!/usr/bin/env python3
"""Wait until /map has valid data; log TF chain for RTAB-Map + Nav2 bringup."""

from __future__ import annotations

import sys

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown


class MapReadyWaitNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_map_ready_wait")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("check_period_sec", 2.0)
        self.declare_parameter("publish_ready", True)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._base = str(self.get_parameter("base_frame").value)
        period = max(1.0, float(self.get_parameter("check_period_sec").value))
        self._pub_ready = bool(self.get_parameter("publish_ready").value)

        self._ready = False
        self._last_map: OccupancyGrid | None = None
        self._buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buf, self, spin_thread=True)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, qos)
        if self._pub_ready:
            self._ready_pub = self.create_publisher(Bool, "/nav2/map_ready", 10)
        self._timer = self.create_timer(period, self._check)
        self.get_logger().info(f"map_ready_wait: listening on {self._map_topic}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg
        w, h = int(msg.info.width), int(msg.info.height)
        if w > 0 and h > 0 and not self._ready:
            self._ready = True
            out = Bool()
            out.data = True
            if self._pub_ready:
                self._ready_pub.publish(out)
            self.get_logger().info(
                f"/map READY resolution={msg.info.resolution:.3f} size={w}x{h}"
            )

    def _lookup(self, parent: str, child: str) -> str:
        try:
            t = self._buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
            tr = t.transform.translation
            return f"OK xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})"
        except tf2_ros.TransformException as ex:
            return f"MISSING ({ex})"

    def _check(self) -> None:
        if not rclpy.ok():
            return
        map_s = "no data"
        if self._last_map is not None:
            w, h = int(self._last_map.info.width), int(self._last_map.info.height)
            map_s = f"{w}x{h} ready={self._ready}"
        self.get_logger().info(
            f"map={map_s} | odom->{self._base}: {self._lookup('odom', self._base)} | "
            f"map->odom: {self._lookup('map', 'odom')} | "
            f"map->{self._base}: {self._lookup('map', self._base)}"
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = MapReadyWaitNode()
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
