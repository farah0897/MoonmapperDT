"""
Collect a Triad BURST over serial and write to `ml/datasets/raw/`.

Wraps `ml/data_collection/collect_triad_burst.py` with defaults under `ml/datasets/`.

Usage:
  Windows:
    python scripts/collect_triad_data.py --list-ports
    python scripts/collect_triad_data.py --port COM3 --sample-id S0001 --append-metadata

  Linux/WSL:
    python3 scripts/collect_triad_data.py --port /dev/ttyACM0 --sample-id S0001 --append-metadata
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    from ml.data_collection.collect_triad_burst import main as impl_main  # type: ignore

    if argv is None:
        argv = list(sys.argv[1:])

    def _has(flag: str) -> bool:
        return any(a == flag or a.startswith(flag + "=") for a in argv)

    if not _has("--output-dir"):
        argv.extend(["--output-dir", "ml/datasets/raw"])
    if not _has("--metadata-path"):
        argv.extend(["--metadata-path", "ml/datasets/metadata.csv"])

    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0], *argv]
        return int(impl_main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
