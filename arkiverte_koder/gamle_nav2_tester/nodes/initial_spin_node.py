#!/usr/bin/env python3
"""Phase 2: publish cmd_vel spin after delay (build initial RTAB-Map coverage)."""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown


class InitialSpinNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_initial_spin")
        self.declare_parameter("start_delay_sec", 5.0)
        self.declare_parameter("spin_duration_sec", 22.0)
        self.declare_parameter("angular_z", 0.3)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wait_for_map_ready", True)

        self._delay = float(self.get_parameter("start_delay_sec").value)
        self._duration = float(self.get_parameter("spin_duration_sec").value)
        self._angular_z = float(self.get_parameter("angular_z").value)
        self._rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        topic = str(self.get_parameter("cmd_vel_topic").value)
        self._wait_map = bool(self.get_parameter("wait_for_map_ready").value)

        self._pub = self.create_publisher(Twist, topic, 10)
        self._map_ready = not self._wait_map
        self._spin_start: Time | None = None
        self._done = False

        if self._wait_map:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
            self.get_logger().info("initial_spin: waiting for /map with data before spin")
        else:
            self.get_logger().info(f"initial_spin: starting after {self._delay:.1f}s delay")

        period = 1.0 / self._rate
        self._timer = self.create_timer(period, self._tick)

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._map_ready:
            return
        if int(msg.info.width) > 0 and int(msg.info.height) > 0:
            self._map_ready = True
            self.get_logger().info(
                f"/map ready ({msg.info.width}x{msg.info.height}); "
                f"spin starts in {self._delay:.1f}s"
            )
            self._spin_start = self.get_clock().now() + Duration(seconds=self._delay)

    def _tick(self) -> None:
        if self._done or not rclpy.ok():
            return
        now = self.get_clock().now()
        if not self._map_ready:
            return
        if self._spin_start is None:
            self._spin_start = now + Duration(seconds=self._delay)
        if now < self._spin_start:
            return
        elapsed = (now - self._spin_start).nanoseconds * 1e-9
        cmd = Twist()
        if elapsed < self._duration:
            cmd.angular.z = self._angular_z
            self._pub.publish(cmd)
            return
        cmd.angular.z = 0.0
        self._pub.publish(cmd)
        if not self._done:
            self._done = True
            self.get_logger().info("initial_spin: completed 360° mapping spin")


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = InitialSpinNode()
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
