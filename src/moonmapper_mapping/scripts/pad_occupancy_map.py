#!/usr/bin/env python3
"""Pad a ROS map_server-style occupancy map (YAML + PGM) with free cells.

Adds free-space border (default 2.0 m) on all sides, updates map origin so the
original map cells stay at the same world positions. Suitable for Nav2 when
the robot or sensor would otherwise leave the saved map bounds.
"""

from __future__ import annotations

import argparse
import math
import os
from collections import Counter
from pathlib import Path

import numpy as np
import yaml


def _read_pgm(path: Path) -> tuple[int, int, int, np.ndarray]:
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"Expected binary PGM P5, got: {path}")
    i = data.index(b"\n", 0) + 1
    while data[i : i + 1] == b"#":
        i = data.index(b"\n", i) + 1
    j = data.index(b"\n", i)
    w, h = map(int, data[i:j].split())
    i = j + 1
    j = data.index(b"\n", i)
    maxval = int(data[i:j])
    if maxval > 255:
        raise ValueError("Only 8-bit PGM supported")
    i = j + 1
    pixels = np.frombuffer(data[i : i + w * h], dtype=np.uint8).reshape((h, w))
    if data[i + w * h :].strip():
        raise ValueError("Trailing bytes in PGM")
    return w, h, maxval, pixels


def _write_pgm(path: Path, img: np.ndarray, maxval: int = 255) -> None:
    h, w = img.shape
    header = f"P5\n{w} {h}\n{maxval}\n".encode("ascii")
    path.write_bytes(header + img.tobytes(order="C"))


def _pick_free_value(img: np.ndarray, maxval: int) -> int:
    flat = img.ravel()
    # Prefer high (free) values seen on the map border
    border = np.concatenate([img[0, :], img[-1, :], img[:, 0], img[:, -1]])
    c = Counter(int(x) for x in border.tolist())
    if c:
        # Most common non-zero on border
        return max(c.items(), key=lambda kv: kv[1])[0]
    # fallback: typical slam_toolbox free
    return min(254, maxval)


def _rotate_delta(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    wx = dx * c - dy * s
    wy = dx * s + dy * c
    return wx, wy


def pad_map(
    input_yaml: Path,
    output_dir: Path,
    output_stem: str,
    padding_m: float,
    free_value: int | None,
) -> tuple[Path, Path]:
    meta = yaml.safe_load(input_yaml.read_text())
    if "image" not in meta or "resolution" not in meta or "origin" not in meta:
        raise ValueError("YAML must contain image, resolution, origin")

    res = float(meta["resolution"])
    origin = meta["origin"]
    if isinstance(origin, dict):
        ox = float(origin.get("x", 0.0))
        oy = float(origin.get("y", 0.0))
        yaw = float(origin.get("yaw", 0.0))
    elif isinstance(origin, (list, tuple)) and len(origin) >= 2:
        ox, oy = float(origin[0]), float(origin[1])
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
    else:
        raise ValueError("origin must be [x,y,yaw] list or dict with x,y,yaw")

    pgm_in = (input_yaml.parent / str(meta["image"])).resolve()
    if not pgm_in.is_file():
        raise FileNotFoundError(f"PGM not found: {pgm_in}")

    w, h, maxval, img = _read_pgm(pgm_in)
    pad_cells = max(1, int(math.ceil(padding_m / res)))
    pad_px = float(pad_cells) * res

    fill = int(free_value) if free_value is not None else int(_pick_free_value(img, maxval))
    fill = max(0, min(maxval, fill))

    out = np.full((h + 2 * pad_cells, w + 2 * pad_cells), fill, dtype=np.uint8)
    out[pad_cells : pad_cells + h, pad_cells : pad_cells + w] = img

    dmx = pad_px
    dmy = pad_px
    wx, wy = _rotate_delta(dmx, dmy, yaw)
    new_ox = ox - wx
    new_oy = oy - wy

    out_yaml = output_dir / f"{output_stem}.yaml"
    out_pgm = output_dir / f"{output_stem}.pgm"

    output_dir.mkdir(parents=True, exist_ok=True)
    # Single-line origin for broad map_server / tool compatibility
    neg = int(meta.get("negate", 0))
    occ_t = meta.get("occupied_thresh", 0.65)
    free_t = meta.get("free_thresh", 0.196)
    mode = meta.get("mode", "trinary")
    ytxt = "\n".join(
        [
            f"image: {out_pgm.name}",
            f"mode: {mode}",
            f"resolution: {res}",
            f"origin: [{new_ox}, {new_oy}, {yaw}]",
            f"negate: {neg}",
            f"occupied_thresh: {occ_t}",
            f"free_thresh: {free_t}",
        ]
    )
    out_yaml.write_text(ytxt + "\n")
    _write_pgm(out_pgm, out, maxval=maxval)

    print(
        f"Padded map: cells +{pad_cells} per side ({pad_px:.3f} m >= {padding_m} m requested)\n"
        f"  size {w}x{h} -> {out.shape[1]}x{out.shape[0]}\n"
        f"  origin ({ox:.6f}, {oy:.6f}, {yaw:.6f}) -> ({new_ox:.6f}, {new_oy:.6f}, {yaw:.6f})\n"
        f"  free fill value: {fill}\n"
        f"  wrote:\n    {out_yaml}\n    {out_pgm}"
    )
    return out_yaml, out_pgm


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--input-yaml",
        type=Path,
        required=True,
        help="Path to existing map.yaml (PGM path taken from its image: field).",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for padded .yaml and .pgm",
    )
    ap.add_argument(
        "--output-stem",
        type=str,
        default="moonmapper_test_map_padded",
        help="Output base name without extension (default: moonmapper_test_map_padded).",
    )
    ap.add_argument(
        "--padding-m",
        type=float,
        default=2.0,
        help="Minimum free padding on each side in metres (default: 2.0).",
    )
    ap.add_argument(
        "--free-value",
        type=int,
        default=None,
        help="PGM byte for padded cells (default: auto from map border).",
    )
    args = ap.parse_args()

    pad_map(
        args.input_yaml.resolve(),
        args.output_dir.resolve(),
        args.output_stem,
        float(args.padding_m),
        args.free_value,
    )


if __name__ == "__main__":
    main()
