"""
Placeholder ROS2 node for reading Triad raw burst data (via Arduino serial).

Phase 1:
- Only logs that it has started.

TODO:
- Implement serial reading (pyserial) and parsing according to Arduino log format
- Publish `moonmapper_interfaces/msg/TriadRaw`
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class TriadSerialNode(Node):
    def __init__(self) -> None:
        super().__init__("triad_serial_node")
        self.get_logger().info("triad_serial_node started (placeholder).")
        self.get_logger().info("TODO: read serial from Arduino and publish TriadRaw.")


def main() -> None:
    rclpy.init()
    node = TriadSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

