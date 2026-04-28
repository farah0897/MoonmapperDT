"""
Placeholder ROS2 node for microscope camera integration.

Phase 2:
- Acquire microscope images, timestamp, and publish/store them.

Current (phase 1):
- Only logs that it has started.

TODO:
- Decide camera interface (OpenCV, vendor SDK, ROS image pipeline)
- Publish `sensor_msgs/Image` or write to dataset folder with metadata linkage
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class MicroscopeCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("microscope_camera_node")
        self.get_logger().info("microscope_camera_node started (placeholder).")
        self.get_logger().info("TODO: capture microscope images and publish/store them.")


def main() -> None:
    rclpy.init()
    node = MicroscopeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

