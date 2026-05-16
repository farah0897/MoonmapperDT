"""OccupancyGrid sanitization and MAP_DEBUG helpers (V1, no NumPy dependency)."""

from __future__ import annotations

from collections import Counter
from math import hypot
from typing import List, Sequence, Tuple

from moonmapper_nav2.frontier_utils import cell_value as grid_cell_value, world_to_map


def sanitize_occ_grid_data(seq: Sequence) -> List[int]:
    """Convert OccupancyGrid.data to Python ints; fix uint8-style wrap (-1 must not stay 255)."""
    out: List[int] = []
    for x in seq:
        v = int(x)
        if v > 127:
            v -= 256
        elif v < -128:
            v = ((v % 256) + 128) % 256 - 128
        out.append(v)
    return out


def map_value_range_str(data: Sequence[int]) -> str:
    if not data:
        return "MAP_VALUE_RANGE min=(empty) max=(empty) unique_sample=[]"
    lo = min(data)
    hi = max(data)
    ctr = Counter(data)
    uniq = sorted(ctr.keys())
    samp = uniq[:15]
    suf = " ..." if len(uniq) > 15 else ""
    return f"MAP_VALUE_RANGE min={lo} max={hi} unique_sample={samp}{suf}"


def _is_occ(v: int, occ_th: int) -> bool:
    return int(v) >= occ_th


def classify_map_cells(
    data: Sequence[int], unk: int, free_th: int, occ_th: int
) -> Tuple[int, int, int, int]:
    """
    Buckets aligned with frontier V1 semantics:
      unknown_count: v == unk
      free_count: v != unk, v >= free_th (usually 0), v < occupied_threshold
      occupied_count: v >= occupied_threshold
      other_count: all else (typically 0 when unk=-1 occ=65)
    """
    unknown_c = free_c = occ_c = other_c = 0
    for vv in data:
        v = int(vv)
        if v == unk:
            unknown_c += 1
        elif _is_occ(v, occ_th):
            occ_c += 1
        elif unk < v < occ_th and v >= free_th:
            free_c += 1
        else:
            other_c += 1
    return unknown_c, free_c, occ_c, other_c


def _cell_center_world(mx: int, my: int, ox: float, oy: float, res: float) -> Tuple[float, float]:
    return ox + (mx + 0.5) * res, oy + (my + 0.5) * res


def local_radius_stats(
    data: Sequence[int],
    w: int,
    h: int,
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
    unk: int,
    free_th: int,
    occ_th: int,
    radii_m: Tuple[float, ...],
) -> str:
    mx0, my0 = world_to_map(rx, ry, ox, oy, res)
    lines: List[str] = []
    for rm in radii_m:
        f_c = uk_c = o_c = ot_c = 0
        r_cells = int(rm / max(res, 1e-9)) + 2
        for my in range(max(0, my0 - r_cells), min(h, my0 + r_cells + 1)):
            for mx in range(max(0, mx0 - r_cells), min(w, mx0 + r_cells + 1)):
                cx, cy = _cell_center_world(mx, my, ox, oy, res)
                if hypot(cx - rx, cy - ry) > rm:
                    continue
                v = int(data[my * w + mx])
                if v == unk:
                    uk_c += 1
                elif _is_occ(v, occ_th):
                    o_c += 1
                elif unk < v < occ_th and v >= free_th:
                    f_c += 1
                else:
                    ot_c += 1
        lines.append(
            f"LOCAL_STATS r={rm}m free={f_c} unknown={uk_c} occupied={o_c} other={ot_c}"
        )
    metric = "(Euclidean meters to cell centers; consistent with centroid offset (mx+0.5)*res)"
    return " | ".join(lines) + " " + metric


def robot_cell_debug_line(
    data: Sequence[int],
    w: int,
    h: int,
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
) -> Tuple[str, Tuple[int, int], bool]:
    mx, my = world_to_map(rx, ry, ox, oy, res)
    inside = 0 <= mx < w and 0 <= my < h
    if inside:
        idx = my * w + mx
        cv = int(data[idx])
    else:
        idx = -1
        cv = -999
    gv = grid_cell_value(list(data), w, h, mx, my)
    vv = int(gv) if gv is not None else cv
    line = (
        f"ROBOT_DEBUG world=({rx:.3f},{ry:.3f}) cell=({mx},{my}) "
        f"inside={inside} index={idx} cell_value={vv}"
    )
    return line, (mx, my), inside


def format_map_debug_header(
    w: int,
    h: int,
    res: float,
    ox: float,
    oy: float,
    frame_id: str,
    stamp_sec: int,
    stamp_nsec: int,
    map_seq: int,
) -> str:
    return (
        "MAP_DEBUG "
        f"size={w}x{h} res={res:.4f} origin=({ox:.2f},{oy:.2f}) "
        f"frame={frame_id} stamp={stamp_sec}.{stamp_nsec:09d} seq={map_seq}"
    )


def format_map_stats_line(
    w: int, h: int, unknown_c: int, free_c: int, occupied_c: int, other_c: int
) -> str:
    tot = w * h
    return (
        "MAP_STATS "
        f"total={tot} free={free_c} unknown={unknown_c} occupied={occupied_c} other={other_c}"
    )
