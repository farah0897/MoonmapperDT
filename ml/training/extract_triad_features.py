"""
Feature extraction for SparkFun AS7265X Triad spectrometer sensors.

Phase 1 (baseline):
- Triad-only features from two sensors (2 * 18 = 36 channels total).
- No assumptions about real dataset existing yet.

Expected raw format (initial assumption / TODO):
- A CSV containing at least 36 float values per sample.
- The first 18 belong to sensor_0, the next 18 belong to sensor_1.

This module provides:
- read_triad_raw_csv(): robust CSV reader (single row or many rows)
- compute_basic_features(): mean/std/min/max over 36 channels
- compute_rms_features(): rms_total, rms_sensor_0, rms_sensor_1
- extract_features(): convenience wrapper returning a feature dict

TODO:
- Align with the final Arduino logging format.
- Add support for timestamps, multiple bursts, and per-burst aggregation.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


NUM_SENSORS_DEFAULT = 2
CHANNELS_PER_SENSOR_DEFAULT = 18


def _safe_float(x: str) -> float:
    try:
        return float(x)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"Could not parse float from: {x!r}") from exc


def read_triad_raw_csv(
    path: str | Path,
    *,
    num_sensors: int = NUM_SENSORS_DEFAULT,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> List[List[float]]:
    """
    Read a Triad raw CSV file.

    Returns a list of samples, where each sample is a list of floats with length
    num_sensors * channels_per_sensor.

    The reader is tolerant to:
- header rows (non-numeric; skipped)
- extra columns (ignored after expected length)

    TODO: define final raw CSV schema (column names, timestamps, etc).
    """
    path = Path(path)
    expected_len = num_sensors * channels_per_sensor
    samples: List[List[float]] = []

    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # Try parse first expected_len items as floats; if fails, treat row as header.
            try:
                values = [_safe_float(v) for v in row[:expected_len]]
            except ValueError:
                continue
            if len(values) != expected_len:
                continue
            samples.append(values)

    if not samples:
        raise ValueError(
            f"No numeric samples found in {path}. "
            "This is expected early on; provide a raw CSV with at least 36 numeric values per row."
        )

    return samples


def _split_sensors(
    sample_36: Sequence[float],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Tuple[List[float], List[float]]:
    if len(sample_36) != 2 * channels_per_sensor:
        raise ValueError(
            f"Expected {2 * channels_per_sensor} values, got {len(sample_36)}"
        )
    s0 = list(sample_36[:channels_per_sensor])
    s1 = list(sample_36[channels_per_sensor : 2 * channels_per_sensor])
    return s0, s1


def _mean(xs: Sequence[float]) -> float:
    return sum(xs) / max(1, len(xs))


def _std(xs: Sequence[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    var = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return math.sqrt(var)


def _rms(xs: Sequence[float]) -> float:
    if not xs:
        return 0.0
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def compute_basic_features(sample_36: Sequence[float]) -> Dict[str, List[float]]:
    """
    Compute basic per-channel summary features.

    Returns dict containing:
    - mean_36, std_36, min_36, max_36 (each a list[float] length 36)

    Note: For a single sample row, mean/min/max are identical to the sample itself.
    This function is mainly useful when `sample_36` is already an aggregated representation.

    TODO: For burst data, compute statistics across time for each channel.
    """
    x = list(sample_36)
    if len(x) != 36:
        raise ValueError(f"Expected 36 channels, got {len(x)}")

    # In a burst setting, these would be computed across multiple rows.
    return {
        "mean_36": x,
        "std_36": [0.0] * 36,
        "min_36": x,
        "max_36": x,
    }


def compute_burst_basic_features(samples: Sequence[Sequence[float]]) -> Dict[str, List[float]]:
    """
    Compute mean/std/min/max per channel across multiple samples (burst).

    Each sample must have length 36.
    """
    if not samples:
        raise ValueError("No samples provided")
    n = len(samples)
    if any(len(s) != 36 for s in samples):
        raise ValueError("All samples must have length 36")

    means: List[float] = []
    stds: List[float] = []
    mins: List[float] = []
    maxs: List[float] = []
    for i in range(36):
        col = [float(samples[j][i]) for j in range(n)]
        means.append(_mean(col))
        stds.append(_std(col))
        mins.append(min(col))
        maxs.append(max(col))
    return {"mean_36": means, "std_36": stds, "min_36": mins, "max_36": maxs}


def compute_rms_features(
    sample_36: Sequence[float],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Dict[str, float]:
    """
    Compute RMS features on the 36-channel vector.
    """
    if len(sample_36) != 2 * channels_per_sensor:
        raise ValueError(f"Expected {2 * channels_per_sensor} channels, got {len(sample_36)}")
    s0, s1 = _split_sensors(sample_36, channels_per_sensor=channels_per_sensor)
    return {
        "rms_total": _rms(sample_36),
        "rms_sensor_0": _rms(s0),
        "rms_sensor_1": _rms(s1),
    }


def extract_features_from_burst(
    samples: Sequence[Sequence[float]],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Dict[str, object]:
    """
    Extract a feature dict from a burst (list of samples).
    """
    basic = compute_burst_basic_features(samples)
    # Use mean_36 as representative vector for RMS calculations (baseline choice).
    rms = compute_rms_features(basic["mean_36"], channels_per_sensor=channels_per_sensor)
    return {**basic, **rms}


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Extract Triad features from a raw CSV (skeleton).")
    parser.add_argument("--raw_csv", type=str, required=False, help="Path to raw triad CSV file.")
    parser.add_argument("--print_keys", action="store_true", help="Print feature keys and exit.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.print_keys:
        example = {
            **compute_burst_basic_features([[0.0] * 36, [0.0] * 36]),
            **compute_rms_features([0.0] * 36),
        }
        print("Feature keys:", ", ".join(sorted(example.keys())))
        return 0

    if not args.raw_csv:
        print("No --raw_csv provided. Example usage:")
        print("  python ml/training/extract_triad_features.py --raw_csv ml/datasets/examples/example_triad.csv")
        return 0

    raw_path = Path(args.raw_csv)
    samples = read_triad_raw_csv(raw_path)
    features = extract_features_from_burst(samples)

    print(f"Read {len(samples)} samples from {raw_path}")
    for k, v in features.items():
        if isinstance(v, list):
            print(f"{k}: len={len(v)} first3={v[:3]}")
        else:
            print(f"{k}: {v}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

