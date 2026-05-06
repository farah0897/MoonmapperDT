"""
Extract Triad features using `ml/datasets/metadata.csv` and raw files under `ml/datasets/`.

Writes:
  ml/datasets/processed/triad_features.csv
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.extract_triad_features import main as impl_main  # type: ignore

    if argv is None:
        argv = list(sys.argv[1:])

    def _has(flag: str) -> bool:
        return any(a == flag or a.startswith(flag + "=") for a in argv)

    if not _has("--metadata"):
        argv.extend(["--metadata", "ml/datasets/metadata.csv"])
    if not _has("--raw-dir"):
        argv.extend(["--raw-dir", "ml/datasets"])
    if not _has("--output"):
        argv.extend(["--output", "ml/datasets/processed/triad_features.csv"])

    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0], *argv]
        return int(impl_main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
