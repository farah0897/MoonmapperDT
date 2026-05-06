"""
Fill missing/default fields in `ml/datasets/metadata.csv` safely (no manual editing).

What it does:
- Sets label_material based on label_object mapping (e.g. aluminium_1krone -> aluminium).
  If label_material is empty or "ukjent", it will be replaced by the mapped value (if known).
- Fills empty run_id using a simple per-object sequence:
  For each label_object, find the minimum numeric sample index among rows in the CSV.
  Then for each row: seq = sample_num - (min_sample_num - 1) and set run_id = R{seq:02d}.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional


LABEL_TO_MATERIAL = {
    "aluminium_1krone": "aluminium",
    "jern_stang": "jern",
    "staal_10krone": "staal",
    "titan_skru": "titan",
}


DEFAULTS = {}


def _parse_sample_num(sample_id: str) -> Optional[int]:
    sample_id = sample_id.strip()
    if not sample_id.startswith("S"):
        return None
    rest = sample_id[1:]
    if not rest.isdigit():
        return None
    return int(rest)


def _fmt_id(prefix: str, n: int) -> str:
    if n < 100:
        return f"{prefix}{n:02d}"
    return f"{prefix}{n}"


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Fill missing fields in metadata.csv safely.")
    p.add_argument("--metadata-path", default="ml/datasets/metadata.csv")
    p.add_argument("--backup", action="store_true", help="Write metadata.csv.bak before modifying.")
    args = p.parse_args(argv)

    meta_path = Path(args.metadata_path)
    if not meta_path.exists():
        raise SystemExit(f"Metadata not found: {meta_path}")

    rows: List[Dict[str, str]] = []
    with meta_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise SystemExit("metadata.csv has no header")
        fieldnames = list(reader.fieldnames)
        required = {"sample_id", "label_object", "label_material"}
        missing = required - set(fieldnames)
        if missing:
            raise SystemExit(f"metadata.csv missing columns: {sorted(missing)}")
        for r in reader:
            rows.append(dict(r))

    # Compute per-label min sample number
    min_by_label: Dict[str, int] = {}
    for r in rows:
        sid = str(r.get("sample_id", "")).strip()
        label = str(r.get("label_object", "")).strip()
        n = _parse_sample_num(sid)
        if not label or n is None:
            continue
        cur = min_by_label.get(label)
        if cur is None or n < cur:
            min_by_label[label] = n

    updated = 0
    for r in rows:
        label = str(r.get("label_object", "")).strip()
        sid = str(r.get("sample_id", "")).strip()
        n = _parse_sample_num(sid)

        # label_material
        material_target = LABEL_TO_MATERIAL.get(label, "")
        if material_target:
            lm = str(r.get("label_material", "")).strip()
            if (not lm) or lm.lower() == "ukjent":
                r["label_material"] = material_target
                updated += 1

        # run_id
        if n is not None and label and label in min_by_label:
            seq = n - (min_by_label[label] - 1)
            if "run_id" in r and not str(r.get("run_id", "")).strip():
                r["run_id"] = _fmt_id("R", seq)
                updated += 1

    if args.backup:
        bak = meta_path.with_suffix(meta_path.suffix + ".bak")
        bak.write_bytes(meta_path.read_bytes())
        print(f"[fill_metadata_defaults] Backup written: {bak}")

    tmp = meta_path.with_suffix(meta_path.suffix + ".tmp")
    with tmp.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    tmp.replace(meta_path)

    print(f"[fill_metadata_defaults] Done. Applied {updated} field updates in {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

