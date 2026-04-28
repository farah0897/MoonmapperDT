"""
Utility for loading ML models in ROS2 runtime.

The runtime environment may not have all ML dependencies installed.
This loader handles missing dependencies and missing files gracefully.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


def load_model(model_path: str | Path) -> Any | None:
    """
    Load a model artifact from disk.

    Returns:
        The loaded model object, or None if it cannot be loaded.

    Notes:
    - Uses joblib if available.
    - Does not raise on missing file; returns None instead.
    """
    path = Path(model_path)
    if not path.exists():
        return None

    try:
        import joblib  # type: ignore
    except Exception:  # noqa: BLE001
        return None

    try:
        return joblib.load(path)
    except Exception:  # noqa: BLE001
        return None

