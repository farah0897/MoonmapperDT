#!/usr/bin/env python3
"""
Plot og sammenlign to Triad rå-CSV (samme format som moonmapper_triad_logger).

Bruk:
  cd ~/rover/simulasjon/moonmapper_ws
  source .venv-ml/bin/activate
  python3 scripts/compare_triad_spectra.py \\
    ml/datasets/raw/staal_triad_raw.csv \\
    ml/datasets/raw/ukjent_triad_raw.csv \\
    --label-a "Stål 4.5 mm" \\
    --label-b "Ukjent kule" \\
    --out ml/datasets/processed/compare_spectra.png

Standard: gjennomsnitt over alle burst-rader (typisk 20) per fil.
Velg --columns N for normaliserte kurver (N0/N1).
Bruk --both for én figur med S øverst og N nederst (form vs nivå).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

WAVELENGTHS_NM = (
    410,
    435,
    460,
    485,
    510,
    535,
    560,
    585,
    610,
    645,
    680,
    705,
    730,
    760,
    810,
    860,
    900,
    940,
)


def _load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{path}: ingen header")
        return list(reader)


def _series_from_csv(
    path: Path,
    *,
    prefix: str,
    aggregate: str,
) -> list[float]:
    rows = _load_rows(path)
    cols = [f"{prefix}_{wl}" for wl in WAVELENGTHS_NM]
    if not rows:
        raise SystemExit(f"{path}: ingen datarader")
    missing = [c for c in cols if c not in rows[0]]
    if missing:
        raise SystemExit(
            f"{path}: mangler kolonner for Triad ({prefix}_410..). Mangler: {missing[:4]}..."
        )

    def mean_col(c: str) -> float:
        vals = [_safe_float(r[c]) for r in rows]
        return sum(vals) / len(vals)

    if aggregate == "first":
        return [_safe_float(rows[0][c]) for c in cols]
    return [mean_col(c) for c in cols]


def _safe_float(x: str) -> float:
    try:
        return float(x)
    except ValueError as exc:
        raise ValueError(f"Klarte ikke tolke tall: {x!r}") from exc


def _plot_two_sensors(
    axes: tuple,
    wl: list[int],
    *,
    y0_a: list[float],
    y1_a: list[float],
    y0_b: list[float],
    y1_b: list[float],
    sens: tuple[str, str],
    label_a: str,
    label_b: str,
    row_title: str,
) -> None:
    pfx = f"{row_title} — " if row_title else ""
    for ax, y_a, y_b, title in (
        (axes[0], y0_a, y0_b, f"{pfx}Sensor 0 ({sens[0]})"),
        (axes[1], y1_a, y1_b, f"{pfx}Sensor 1 ({sens[1]})"),
    ):
        ax.plot(wl, y_a, "o-", label=label_a, linewidth=1.5, markersize=4)
        ax.plot(wl, y_b, "s-", label=label_b, linewidth=1.5, markersize=4)
        ax.set_xlabel("Bølgelengde (nm)")
        ax.set_ylabel("Verdi")
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)


def main() -> int:
    p = argparse.ArgumentParser(description="Sammenlign to Triad spektrum-CSV (S eller N kolonner).")
    p.add_argument("csv_a", type=Path, help="Første råfil (f.eks. stålkule)")
    p.add_argument("csv_b", type=Path, help="Andre råfil (f.eks. ukjent kule)")
    p.add_argument("--label-a", default="Sample A", help="Tekst i legend (fil A)")
    p.add_argument("--label-b", default="Sample B", help="Tekst i legend (fil B)")
    p.add_argument(
        "--both",
        action="store_true",
        help="Én PNG med to rader: S0/S1 (korrigert) og N0/N1 (normalisert). Ignorerer --columns.",
    )
    p.add_argument(
        "--columns",
        choices=("S", "N"),
        default="S",
        help="S = korrigerte S0/S1. N = normaliserte N0/N1 (kurveform). Ignoreres hvis --both.",
    )
    p.add_argument(
        "--aggregate",
        choices=("mean", "first"),
        default="mean",
        help="mean = gjennomsnitt over burst (anbefalt). first = kun første rad.",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Lagre figur som PNG. Uten --out åpnes et vindu.",
    )
    args = p.parse_args()

    for path in (args.csv_a, args.csv_b):
        if not path.exists():
            print(f"Fant ikke fil: {path}", file=sys.stderr)
            return 2

    wl = list(WAVELENGTHS_NM)

    try:
        if args.both:
            s0_a = _series_from_csv(args.csv_a, prefix="S0", aggregate=args.aggregate)
            s1_a = _series_from_csv(args.csv_a, prefix="S1", aggregate=args.aggregate)
            s0_b = _series_from_csv(args.csv_b, prefix="S0", aggregate=args.aggregate)
            s1_b = _series_from_csv(args.csv_b, prefix="S1", aggregate=args.aggregate)
            n0_a = _series_from_csv(args.csv_a, prefix="N0", aggregate=args.aggregate)
            n1_a = _series_from_csv(args.csv_a, prefix="N1", aggregate=args.aggregate)
            n0_b = _series_from_csv(args.csv_b, prefix="N0", aggregate=args.aggregate)
            n1_b = _series_from_csv(args.csv_b, prefix="N1", aggregate=args.aggregate)
        else:
            sens = ("S0", "S1") if args.columns == "S" else ("N0", "N1")
            s0_a = _series_from_csv(args.csv_a, prefix=sens[0], aggregate=args.aggregate)
            s1_a = _series_from_csv(args.csv_a, prefix=sens[1], aggregate=args.aggregate)
            s0_b = _series_from_csv(args.csv_b, prefix=sens[0], aggregate=args.aggregate)
            s1_b = _series_from_csv(args.csv_b, prefix=sens[1], aggregate=args.aggregate)
    except (ValueError, SystemExit) as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.both:
        fig, axes = plt.subplots(2, 2, figsize=(12, 9), sharey=False)
        fig.suptitle(
            f"Triad — S (korrigert) øverst, N (normalisert form) nederst — {args.aggregate}"
        )
        _plot_two_sensors(
            (axes[0][0], axes[0][1]),
            wl,
            y0_a=s0_a,
            y1_a=s1_a,
            y0_b=s0_b,
            y1_b=s1_b,
            sens=("S0", "S1"),
            label_a=args.label_a,
            label_b=args.label_b,
            row_title="S",
        )
        _plot_two_sensors(
            (axes[1][0], axes[1][1]),
            wl,
            y0_a=n0_a,
            y1_a=n1_a,
            y0_b=n0_b,
            y1_b=n1_b,
            sens=("N0", "N1"),
            label_a=args.label_a,
            label_b=args.label_b,
            row_title="N",
        )
    else:
        sens = ("S0", "S1") if args.columns == "S" else ("N0", "N1")
        col_title = "S (korrigert)" if args.columns == "S" else "N (normalisert)"
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=False)
        fig.suptitle(f"Triad spektrum — {col_title} — {args.aggregate}")
        _plot_two_sensors(
            (axes[0], axes[1]),
            wl,
            y0_a=s0_a,
            y1_a=s1_a,
            y0_b=s0_b,
            y1_b=s1_b,
            sens=sens,
            label_a=args.label_a,
            label_b=args.label_b,
            row_title="",
        )

    plt.tight_layout()

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.out, dpi=150)
        print(f"Lagret: {args.out}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
