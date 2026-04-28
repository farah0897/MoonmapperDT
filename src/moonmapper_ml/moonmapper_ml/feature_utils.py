"""
Feature utilities for ROS2 inference.

TODO:
- Convert `moonmapper_interfaces/msg/TriadFeatures` to a numpy feature vector
- Ensure consistent feature ordering with the training pipeline
"""

from __future__ import annotations

from typing import List


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

