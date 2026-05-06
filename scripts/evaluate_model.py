"""
Evaluate a trained model on `ml/datasets/processed/test.csv`.
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.evaluate_model import main as impl_main  # type: ignore

    if argv is None:
        argv = list(sys.argv[1:])

    def _has(flag: str) -> bool:
        return any(a == flag or a.startswith(flag + "=") for a in argv)

    if not _has("--model_path"):
        argv.extend(["--model_path", "ml/models/random_forest.joblib"])
    if not _has("--encoder_path"):
        argv.extend(["--encoder_path", "ml/models/label_encoder.joblib"])
    if not _has("--input"):
        argv.extend(["--input", "ml/datasets/processed/test.csv"])
    if not _has("--cm-out"):
        argv.extend(["--cm-out", "ml/datasets/processed/confusion_matrix.png"])

    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0], *argv]
        return int(impl_main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
