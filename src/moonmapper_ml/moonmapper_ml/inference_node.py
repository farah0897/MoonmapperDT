"""
Placeholder ROS2 node for ML inference.

Phase 1:
- Load a Random Forest model (joblib) when available
- Subscribe to `/triad/features` (TriadFeatures) later
- Publish `/ml/triad_classification` (MaterialClassification) later

Current:
- Only logs that it has started.
- Attempts to load a model; logs if missing.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node

from .model_loader import load_joblib_artifact


def _default_model_paths() -> tuple[Path, Path]:
    """
    Find default model + encoder paths.

    Priority:
    1) Installed package share dir (works after `colcon build`)
    2) Source tree fallback (useful during development)
    """
    # 1) Installed share directory
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        share_dir = Path(get_package_share_directory("moonmapper_ml"))
        model = share_dir / "models" / "random_forest_triad.joblib"
        encoder = share_dir / "models" / "label_encoder_triad.joblib"
        return model, encoder
    except Exception:
        pass

    # 2) Source tree fallback
    ws_src_models = Path(__file__).resolve().parents[3] / "src" / "moonmapper_ml" / "models"
    return ws_src_models / "random_forest_triad.joblib", ws_src_models / "label_encoder_triad.joblib"


class MLInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("ml_inference_node")

        # Parameters
        default_model, default_encoder = _default_model_paths()
        self.declare_parameter("model_path", str(default_model))
        self.declare_parameter("label_encoder_path", str(default_encoder))

        self.get_logger().info("ml_inference_node started (placeholder).")
        model_path = Path(self.get_parameter("model_path").value)
        enc_path = Path(self.get_parameter("label_encoder_path").value)

        model, model_status = load_joblib_artifact(model_path)
        if model is None:
            if model_status == "missing_file":
                self.get_logger().warn(f"Model file missing: {model_path}")
            elif model_status == "missing_joblib":
                self.get_logger().warn(
                    f"Model file exists but Python dependency 'joblib' is missing in ROS runtime. Path: {model_path}"
                )
            else:
                self.get_logger().warn(f"Model file could not be loaded ({model_status}). Path: {model_path}")
        else:
            self.get_logger().info(f"Loaded model: {model_path}")

        encoder, enc_status = load_joblib_artifact(enc_path)
        if encoder is None:
            if enc_status == "missing_file":
                self.get_logger().warn(f"Label encoder missing: {enc_path}")
            elif enc_status == "missing_joblib":
                self.get_logger().warn(
                    f"Label encoder exists but Python dependency 'joblib' is missing in ROS runtime. Path: {enc_path}"
                )
            else:
                self.get_logger().warn(f"Label encoder could not be loaded ({enc_status}). Path: {enc_path}")
        else:
            self.get_logger().info(f"Loaded label encoder: {enc_path}")

        self.get_logger().info("TODO: subscribe /triad/features and publish /ml/triad_classification.")


def main() -> None:
    rclpy.init()
    node = MLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C may already have triggered shutdown via signal handler.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

