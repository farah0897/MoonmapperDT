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


def load_label_encoder(encoder_path: str | Path) -> Any | None:
    """
    Load a label encoder (joblib) from disk.
    Returns None if missing or cannot be loaded.
    """
    path = Path(encoder_path)
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


def load_joblib_artifact(path_in: str | Path) -> tuple[Any | None, str]:
    """
    Load a joblib artifact with a human-readable status string.

    Returns:
        (obj_or_none, status)
    where status is one of:
        - "missing_file"
        - "missing_joblib"
        - "load_error:<message>"
        - "ok"
    """
    path = Path(path_in)
    if not path.exists():
        return None, "missing_file"

    try:
        import joblib  # type: ignore
    except Exception:
        return None, "missing_joblib"

    try:
        return joblib.load(path), "ok"
    except Exception as exc:  # noqa: BLE001
        return None, f"load_error:{exc}"

