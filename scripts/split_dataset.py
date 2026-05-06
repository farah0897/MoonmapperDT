"""
Split `ml/datasets/processed/triad_features.csv` into train/val/test splits.

Writes:
  ml/datasets/processed/train.csv
  ml/datasets/processed/val.csv
  ml/datasets/processed/test.csv
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.split_dataset import main as impl_main  # type: ignore

    if argv is None:
        argv = list(sys.argv[1:])

    def _has(flag: str) -> bool:
        return any(a == flag or a.startswith(flag + "=") for a in argv)

    if not _has("--input"):
        argv.extend(["--input", "ml/datasets/processed/triad_features.csv"])

    if not _has("--out-dir"):
        argv.extend(["--out-dir", "ml/datasets/processed"])

    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0], *argv]
        return int(impl_main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
