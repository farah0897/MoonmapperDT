#!/usr/bin/env python3
"""
Fix legacy raw Triad CSV channel mapping to match SparkFun's official order.

Context
-------
Older logging code (and FARAH-derived variants) may have written raw CSV columns
named like S0_610, S0_645, ... but filled those columns with values from a
different getter order (R/I/S/J/T/U/V/W/K/L permutation).

The updated Arduino logger (`arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino`)
now matches SparkFun's example order 1:1:
  A,B,C,D,E,F,G,H,R,I,S,J,T,U,V,W,K,L

This script rewrites *values* in the legacy raw CSV so that the meaning of
columns like S0_610, S0_645, ... matches the updated logger.

Important
---------
- Column *names* are kept the same.
- Only the 610..940 band columns are permuted. 410..585 remain unchanged.
- Works for both S0_/S1_ schemas and L_/R_ schemas (as used by the ML extractor).

Usage
-----
  python3 scripts/fix_triad_raw_channel_order.py --in-dir ml/datasets/raw --out-dir ml/datasets/raw_fixed

You can also pass individual files with --in-file.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator


WAVELENGTHS = (410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940)

# This is the value-permutation required to transform *legacy* files (written with the
# old Moonmapper mapping) into the new SparkFun-consistent mapping, while keeping the
# same column names.
#
# For each sensor prefix P in {S0,S1} (and similarly L/R):
#   new P_610 = old P_730
#   new P_645 = old P_610
#   new P_680 = old P_760
#   new P_705 = old P_645
#   new P_730 = old P_810
#   new P_760 = old P_860
#   new P_810 = old P_900
#   new P_860 = old P_940
#   new P_900 = old P_680
#   new P_940 = old P_705
PERMUTE_610_940 = {
    610: 730,
    645: 610,
    680: 760,
    705: 645,
    730: 810,
    760: 860,
    810: 900,
    860: 940,
    900: 680,
    940: 705,
}


@dataclass(frozen=True)
class Schema:
    p0: str  # e.g. "S0" or "L"
    p1: str  # e.g. "S1" or "R"


def _detect_schema(fieldnames: Iterable[str]) -> Schema:
    fields = set(fieldnames)
    s0 = {f"S0_{wl}" for wl in WAVELENGTHS}
    s1 = {f"S1_{wl}" for wl in WAVELENGTHS}
    l0 = {f"L_{wl}" for wl in WAVELENGTHS}
    r1 = {f"R_{wl}" for wl in WAVELENGTHS}

    if s0.issubset(fields) and s1.issubset(fields):
        return Schema("S0", "S1")
    if l0.issubset(fields) and r1.issubset(fields):
        return Schema("L", "R")

    raise ValueError(
        "Could not detect schema. Expected columns like "
        "S0_410..S0_940 + S1_410..S1_940, or L_410..L_940 + R_410..R_940."
    )


def _iter_csv_files(in_dir: Path) -> Iterator[Path]:
    for p in sorted(in_dir.rglob("*.csv")):
        if p.is_file():
            yield p


def _rewrite_row(row: dict[str, str], *, schema: Schema) -> dict[str, str]:
    out = dict(row)  # start with everything unchanged
    for prefix in (schema.p0, schema.p1):
        for new_wl, old_wl in PERMUTE_610_940.items():
            new_col = f"{prefix}_{new_wl}"
            old_col = f"{prefix}_{old_wl}"
            # swap values by assignment from old -> new
            if new_col in row and old_col in row:
                out[new_col] = row[old_col]
    return out


def fix_file(in_path: Path, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with in_path.open("r", newline="") as f_in:
        reader = csv.DictReader(f_in)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {in_path}")
        schema = _detect_schema(reader.fieldnames)
        fieldnames = list(reader.fieldnames)

        with out_path.open("w", newline="") as f_out:
            writer = csv.DictWriter(f_out, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for row in reader:
                writer.writerow(_rewrite_row(row, schema=schema))


def main() -> int:
    ap = argparse.ArgumentParser(description="Fix legacy Triad raw CSV channel mapping.")
    ap.add_argument("--in-dir", default="", help="Directory containing raw CSV files to fix (recursively).")
    ap.add_argument("--out-dir", default="", help="Output directory for fixed CSV files (mirrors structure).")
    ap.add_argument("--in-file", default="", help="Fix a single raw CSV file.")
    ap.add_argument("--out-file", default="", help="Output path for a single fixed CSV file.")
    args = ap.parse_args()

    if args.in_file:
        if not args.out_file:
            raise SystemExit("--out-file is required when using --in-file")
        fix_file(Path(args.in_file), Path(args.out_file))
        print(f"Wrote {args.out_file}")
        return 0

    if not args.in_dir or not args.out_dir:
        raise SystemExit("Provide either --in-file/--out-file or --in-dir/--out-dir")

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)
    if not in_dir.is_dir():
        raise SystemExit(f"--in-dir is not a directory: {in_dir}")

    n = 0
    for in_path in _iter_csv_files(in_dir):
        rel = in_path.relative_to(in_dir)
        out_path = out_dir / rel
        fix_file(in_path, out_path)
        n += 1

    print(f"Done. Wrote {n} file(s) under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

