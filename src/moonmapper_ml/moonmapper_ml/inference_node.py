"""
Placeholder ROS2 node for ML inference.

Phase 1:
- Load a Random Forest model (joblib) when available
- Subscribe to `/triad/features` (TriadFeatures) later
- Publish `/ml/classification` (MaterialClassification) later

Current:
- Only logs that it has started.
- Attempts to load a model; logs if missing.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node

from .model_loader import load_model


class MLInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("ml_inference_node")

        # TODO: make this a ROS parameter
        default_model_path = Path(__file__).resolve().parent.parent / "models" / "random_forest.joblib"

        self.get_logger().info("ml_inference_node started (placeholder).")
        model = load_model(default_model_path)
        if model is None:
            self.get_logger().warn(
                f"Model file missing or could not be loaded: {default_model_path}. "
                "TODO: train and place a model artifact, or configure a parameter."
            )
        else:
            self.get_logger().info(f"Loaded model: {default_model_path}")

        self.get_logger().info("TODO: subscribe /triad/features and publish /ml/classification.")


def main() -> None:
    rclpy.init()
    node = MLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

