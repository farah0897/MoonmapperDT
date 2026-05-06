"""
Feature utilities for ROS2 inference.

TODO:
- Convert `moonmapper_interfaces/msg/TriadFeatures` to a numpy feature vector
- Ensure consistent feature ordering with the training pipeline
"""

from __future__ import annotations

from typing import List, Optional


def triad_features_to_vector(
    *,
    mean_36: List[float],
    std_36: List[float],
    min_36: List[float],
    max_36: List[float],
    rms_total: float,
    rms_sensor_0: float,
    rms_sensor_1: float,
) -> List[float]:
    """
    Baseline: concatenate arrays into a single flat feature vector.

    Returns:
        list[float] suitable as input to scikit-learn model predict().

    TODO:
    - Add optional ratios/derivatives based on `ml/configs/feature_config.yaml`
    - Validate lengths (expected 36 for arrays)
    """
    vec: List[float] = []
    vec.extend(mean_36)
    vec.extend(std_36)
    vec.extend(min_36)
    vec.extend(max_36)
    vec.append(float(rms_total))
    vec.append(float(rms_sensor_0))
    vec.append(float(rms_sensor_1))
    return vec


def triad_features_msg_to_vector(msg) -> List[float]:
    """
    Convert a ROS `TriadFeatures` message into a flat feature vector.

    Note: kept as a simple list[float] to avoid requiring numpy at runtime.
    TODO: validate lengths and align exact ordering with training schema.
    """
    return triad_features_to_vector(
        mean_36=list(getattr(msg, "mean_36", [])),
        std_36=list(getattr(msg, "std_36", [])),
        min_36=list(getattr(msg, "min_36", [])),
        max_36=list(getattr(msg, "max_36", [])),
        rms_total=float(getattr(msg, "rms_total", 0.0)),
        rms_sensor_0=float(getattr(msg, "rms_sensor_0", 0.0)),
        rms_sensor_1=float(getattr(msg, "rms_sensor_1", 0.0)),
    )


def probabilities_to_classification_msg(
    *,
    predicted_class: str,
    confidence: float,
    class_names: List[str],
    probabilities: List[float],
    source: str,
    header: Optional[object] = None,
):
    """
    Build a `MaterialClassification` message if available, otherwise return a string fallback.
    """
    try:
        from moonmapper_interfaces.msg import MaterialClassification  # type: ignore
    except Exception:
        # Fallback: caller can publish as std_msgs/String
        return f"{source}:{predicted_class} conf={confidence:.3f}"

    msg = MaterialClassification()
    if header is not None:
        msg.header = header
    msg.predicted_class = str(predicted_class)
    msg.confidence = float(confidence)
    msg.class_names = [str(x) for x in class_names]
    msg.probabilities = [float(x) for x in probabilities]
    msg.source = str(source)
    return msg

