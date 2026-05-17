"""
Placeholder ROS2 node to convert TriadRaw -> TriadFeatures.

Phase 1:
- Subscribe to raw triad bursts (future)
- Extract features and publish TriadFeatures

Current (phase 1 bootstrap):
- Only logs that it has started.

TODO:
- Subscribe to TriadRaw
- Use shared feature extraction logic (ml/training/extract_triad_features.py or a shared lib)
- Publish TriadFeatures
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class FeatureExtractionNode(Node):
    def __init__(self) -> None:
        super().__init__("feature_extraction_node")
        self.get_logger().info("feature_extraction_node started (placeholder).")
        self.get_logger().info("TODO: subscribe TriadRaw and publish TriadFeatures.")


def main() -> None:
    rclpy.init()
    node = FeatureExtractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

