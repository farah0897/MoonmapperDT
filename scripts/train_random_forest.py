"""
Train Random Forest on `ml/datasets/processed/triad_features.csv`.

Writes:
  ml/models/random_forest.joblib
  ml/models/label_encoder.joblib
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.train_random_forest import main as impl_main  # type: ignore

    if argv is None:
        argv = list(sys.argv[1:])

    def _has(flag: str) -> bool:
        return any(a == flag or a.startswith(flag + "=") for a in argv)

    if not _has("--input"):
        argv.extend(["--input", "ml/datasets/processed/triad_features.csv"])
    if not _has("--model-output"):
        argv.extend(["--model-output", "ml/models/random_forest.joblib"])
    if not _has("--encoder-output"):
        argv.extend(["--encoder-output", "ml/models/label_encoder.joblib"])

    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0], *argv]
        return int(impl_main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
