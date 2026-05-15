"""Safe rclpy shutdown helpers (Jazzy/Humble compatible)."""

from __future__ import annotations

import rclpy


def safe_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def is_shutdown_exception(exc: BaseException) -> bool:
    name = type(exc).__name__
    return name in ("ExternalShutdownException", "RCLError", "RuntimeError")
