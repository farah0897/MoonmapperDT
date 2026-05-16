"""Grid operations: frontiers, passable mask, BFS reachability, goal validation (V1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from moonmapper_nav2.frontier_utils import (
    FrontierCluster,
    cell_value,
    _is_free,
    _is_unknown,
    _is_occupied,
    world_to_map,
)


class RejectReason(str, Enum):
    NO_APPROACH_CELL = "no_approach_cell"


@dataclass
class ApproachPickStats:
    candidates_total: int = 0
    rejected_out_of_bounds: int = 0
    rejected_annulus: int = 0
    rejected_unknown: int = 0
    rejected_occupied: int = 0
    rejected_not_reachable: int = 0
    rejected_not_passable: int = 0
    rejected_too_close_obstacle: int = 0
    rejected_blacklist: int = 0
    rejected_robot_distance: int = 0

    def summary_line(self) -> str:
        parts = [
            f"candidates_total={self.candidates_total}",
            f"rejected_unknown={self.rejected_unknown}",
            f"rejected_occupied={self.rejected_occupied}",
            f"rejected_not_reachable={self.rejected_not_reachable}",
            f"rejected_not_passable={self.rejected_not_passable}",
            f"rejected_too_close_obstacle={self.rejected_too_close_obstacle}",
            f"rejected_blacklist={self.rejected_blacklist}",
            f"rejected_robot_distance={self.rejected_robot_distance}",
        ]
        return " ".join(parts)


@dataclass
class ValidatedGoal:
    wx: float
    wy: float
    yaw: float
    approach_ixy: Tuple[int, int]
    cluster: FrontierCluster
    approach_method: str


def _idx(mx: int, my: int, w: int) -> int:
    return my * w + mx


def collect_frontier_clusters(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
    min_cluster_size: int,
) -> List[FrontierCluster]:
    """Free cells (known, not occupied) with at least one unknown 4-neighbor."""
    vis = [False] * (w * h)
    clusters: List[FrontierCluster] = []
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            v = int(data[i])
            if vis[i]:
                continue
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            touches_unknown = False
            for dx, dy in neigh:
                nv = cell_value(list(data), w, h, mx + dx, my + dy)
                if nv is not None and int(nv) == unknown_value:
                    touches_unknown = True
                    break
            if not touches_unknown:
                continue
            stack = [(mx, my)]
            vis[i] = True
            cells: List[Tuple[int, int]] = []
            while stack:
                cx, cy = stack.pop()
                cells.append((cx, cy))
                for dx, dy in neigh:
                    nx, ny = cx + dx, cy + dy
                    ni = _idx(nx, ny, w)
                    if nx < 0 or ny < 0 or nx >= w or ny >= h or vis[ni]:
                        continue
                    tv = int(data[ni])
                    if not _is_free(tv, free_threshold, occupied_threshold, unknown_value):
                        continue
                    touches_u = False
                    for ddx, ddy in neigh:
                        uv = cell_value(list(data), w, h, nx + ddx, ny + ddy)
                        if uv is not None and int(uv) == unknown_value:
                            touches_u = True
                            break
                    if not touches_u:
                        continue
                    vis[ni] = True
                    stack.append((nx, ny))
            if len(cells) >= min_cluster_size:
                clusters.append(FrontierCluster(cells=cells))
    return clusters


def build_passable_mask(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    occupied_threshold: int,
    free_threshold: int,
    unknown_as_blocked: bool,
    inflation_radius_m: float,
    resolution: float,
) -> Tuple[List[bool], List[bool]]:
    """Known-free traversable cells with obstacle/unknown erosion ( inflation )."""
    n = w * h
    raw_blocked = [False] * n
    for i in range(n):
        v = int(data[i])
        if _is_unknown(v, unknown_value):
            raw_blocked[i] = unknown_as_blocked
        elif _is_occupied(v, occupied_threshold):
            raw_blocked[i] = True
        elif not _is_free(v, free_threshold, occupied_threshold, unknown_value):
            raw_blocked[i] = True

    steps = max(0, int(round(float(inflation_radius_m) / max(resolution, 1e-6))))
    blocked = [bool(x) for x in raw_blocked]
    tmp = [False] * n
    for _ in range(steps):
        for my in range(h):
            for mx in range(w):
                i = _idx(mx, my, w)
                if raw_blocked[i]:
                    tmp[i] = True
                    continue
                neigh_blk = False
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = mx + dx, my + dy
                        if nx < 0 or ny < 0 or nx >= w or ny >= h:
                            continue
                        if blocked[_idx(nx, ny, w)]:
                            neigh_blk = True
                            break
                    if neigh_blk:
                        break
                tmp[i] = neigh_blk
        blocked, tmp = tmp, [False] * n

    passable = [False] * n
    for i in range(n):
        v = int(data[i])
        if _is_unknown(v, unknown_value):
            passable[i] = False
        elif _is_occupied(v, occupied_threshold):
            passable[i] = False
        elif not _is_free(v, free_threshold, occupied_threshold, unknown_value):
            passable[i] = False
        else:
            passable[i] = not blocked[i]
    inflation_dbg = blocked
    return passable, inflation_dbg


def _bfs_mask(
    seed: Optional[Tuple[int, int]], passable: Sequence[bool], w: int, h: int
) -> List[bool]:
    reach = [False] * (w * h)
    if seed is None:
        return reach
    sx, sy = seed
    if sx < 0 or sy < 0 or sx >= w or sy >= h:
        return reach
    si = _idx(sx, sy, w)
    if not passable[si]:
        return reach
    q = [seed]
    reach[si] = True
    head = 0
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while head < len(q):
        cx, cy = q[head]
        head += 1
        for dx, dy in neigh:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            ni = _idx(nx, ny, w)
            if reach[ni] or not passable[ni]:
                continue
            reach[ni] = True
            q.append((nx, ny))
    return reach




def build_bfs_seed_radii_m(max_m: float, step_m: float) -> Tuple[float, ...]:
    checkpoints = [0.25, 0.5, 1.0, 1.5, 2.0, 3.0]
    max_m = float(max(max_m, 0.0))
    rings = {float(r) for r in checkpoints if r <= max_m + 1e-9}
    step = float(max(step_m, 0.05))
    cur = step
    while cur <= max_m + 1e-9:
        rings.add(round(cur, 4))
        cur += step
    out = tuple(sorted(rings))
    if out:
        return out
    if max_m <= 0.0:
        return (0.25,)
    return tuple(sorted({min(0.25, max_m), max_m}))


def count_raw_frontier_cells(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> int:
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    n_cells = 0
    lst = list(data)
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            v = int(lst[i])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            touches_unknown = False
            for dx, dy in neigh:
                nv = cell_value(lst, w, h, mx + dx, my + dy)
                if nv is not None and int(nv) == unknown_value:
                    touches_unknown = True
                    break
            if touches_unknown:
                n_cells += 1
    return n_cells


def global_nearest_free_cell(
    rx: float,
    ry: float,
    data: Sequence[int],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[Optional[Tuple[int, int]], float]:
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            v = int(data[_idx(mx, my, w)])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def _grid_bounds_for_disk(
    rx: float,
    ry: float,
    rd: float,
    ox: float,
    oy: float,
    res: float,
    w: int,
    h: int,
    margin_cells: int,
) -> Tuple[int, int, int, int]:
    """Inclusive mx/my bounds covering all cells whose centers can lie within rd of (rx,ry)."""
    res = max(float(res), 1e-9)
    mx_lo = int(math.ceil((rx - rd - ox) / res - 0.5 - 1e-9))
    mx_hi = int(math.floor((rx + rd - ox) / res - 0.5 + 1e-9))
    my_lo = int(math.ceil((ry - rd - oy) / res - 0.5 - 1e-9))
    my_hi = int(math.floor((ry + rd - oy) / res - 0.5 + 1e-9))
    m = max(0, margin_cells)
    mx_lo -= m
    mx_hi += m
    my_lo -= m
    my_hi += m
    mx_lo = max(0, mx_lo)
    mx_hi = min(w - 1, mx_hi)
    my_lo = max(0, my_lo)
    my_hi = min(h - 1, my_hi)
    if mx_lo > mx_hi or my_lo > my_hi:
        return 0, w - 1, 0, h - 1
    return mx_lo, mx_hi, my_lo, my_hi


def global_nearest_passable_cell(
    rx: float,
    ry: float,
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
) -> Tuple[Optional[Tuple[int, int]], float]:
    """Nearest cell with passable_bfs True (Euclidean distance to cell center in map plane)."""
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if not passable_bfs[i]:
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def global_nearest_passable_known_free_cell(
    rx: float,
    ry: float,
    data: Sequence[int],
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[Optional[Tuple[int, int]], float]:
    """Nearest passable_bfs cell that is also known-free in occupancy (not unknown overlay-only)."""
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if not passable_bfs[i]:
                continue
            v = int(data[i])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def find_nearest_bfs_seed(
    robot_ixy: Tuple[int, int],
    rx: float,
    ry: float,
    data: Sequence[int],
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
    max_radius_m: float,
    step_m: float,
) -> Tuple[Optional[Tuple[int, int]], float, str]:
    """Nearest known-free + passable_bfs cell inside expanding Euclidean discs."""
    _ = robot_ixy
    for rd in build_bfs_seed_radii_m(max_radius_m, step_m):
        best_d = 1e18
        best: Optional[Tuple[int, int]] = None
        mx_lo, mx_hi, my_lo, my_hi = _grid_bounds_for_disk(rx, ry, rd, ox, oy, res, w, h, 1)
        for my in range(my_lo, my_hi + 1):
            for mx in range(mx_lo, mx_hi + 1):
                wc_x = ox + (mx + 0.5) * res
                wc_y = oy + (my + 0.5) * res
                dm = math.hypot(wc_x - rx, wc_y - ry)
                if dm > rd + 1e-6:
                    continue
                idx = _idx(mx, my, w)
                if not passable_bfs[idx]:
                    continue
                v = int(data[idx])
                if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                    continue
                if dm < best_d:
                    best_d = dm
                    best = (mx, my)
        if best is not None:
            return best, best_d, "nearest_known_free_radius"
    return None, 0.0, "none"


def _count_reachable(reach: Sequence[bool]) -> int:
    return sum(1 for x in reach if x)


def bridge_passable_bfs_to_known_free(
    pass_bfs: Sequence[bool],
    data: Sequence[int],
    w: int,
    h: int,
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
    unknown_val: int,
    free_th: int,
    occ_th: int,
    radius_m: float,
) -> List[bool]:
    """Connect BFS graph to explored known-free cells near the robot (local mask only)."""
    out = [bool(x) for x in pass_bfs]
    if radius_m <= 0.0:
        return out
    r_cells = int(math.ceil(float(radius_m) / max(res, 1e-9))) + 2
    mx0, my0 = world_to_map(rx, ry, ox, oy, res)
    for my in range(max(0, my0 - r_cells), min(h, my0 + r_cells + 1)):
        for mx in range(max(0, mx0 - r_cells), min(w, mx0 + r_cells + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if math.hypot(wx - rx, wy - ry) > radius_m + 1e-6:
                continue
            i = _idx(mx, my, w)
            if _is_free(int(data[i]), free_th, occ_th, unknown_val) and not out[i]:
                out[i] = True
    return out


def apply_robot_footprint_clearing(
    passable: Sequence[bool],
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
    w: int,
    h: int,
    radius_m: float,
) -> List[bool]:
    """Writable copy with extra passable disk around robot pose (explorer-local only)."""
    out = [bool(x) for x in passable]
    r_cells = int(math.ceil(float(radius_m) / max(res, 1e-9))) + 2
    mx0, my0 = world_to_map(rx, ry, ox, oy, res)
    for my in range(max(0, my0 - r_cells), min(h, my0 + r_cells + 1)):
        for mx in range(max(0, mx0 - r_cells), min(w, mx0 + r_cells + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if math.hypot(wx - rx, wy - ry) <= radius_m + 1e-6:
                out[_idx(mx, my, w)] = True
    return out


def resolve_bfs_seed_and_masks(
    robot_ixy: Tuple[int, int],
    robot_xy: Tuple[float, float],
    data: Sequence[int],
    w: int,
    h: int,
    passable_bfs_main: Sequence[bool],
    passable_bfs_staging: Sequence[bool],
    ox: float,
    oy: float,
    res: float,
    unknown_val: int,
    free_th: int,
    occ_th: int,
    bfs_seed_radius_m: float,
    bfs_seed_step_m: float,
    min_reachable_cells: int = 500,
) -> Tuple[Optional[Tuple[int, int]], List[bool], List[bool], float, str]:
    rx, ry = robot_xy
    seed, sd_m, smeth = find_nearest_bfs_seed(
        robot_ixy,
        rx,
        ry,
        data,
        passable_bfs_main,
        w,
        h,
        ox,
        oy,
        res,
        unknown_val,
        free_th,
        occ_th,
        bfs_seed_radius_m,
        bfs_seed_step_m,
    )
    if seed is None:
        gp, gd = global_nearest_passable_known_free_cell(
            rx, ry, data, passable_bfs_main, w, h, ox, oy, res, unknown_val, free_th, occ_th
        )
        if gp is not None:
            seed, sd_m, smeth = gp, gd, "global_nearest_passable_known_free"
    rm = _bfs_mask(seed, passable_bfs_main, w, h)
    rs = _bfs_mask(seed, passable_bfs_staging, w, h)
    if seed is not None and _count_reachable(rm) < min_reachable_cells:
        gp, gd = global_nearest_passable_known_free_cell(
            rx, ry, data, passable_bfs_main, w, h, ox, oy, res, unknown_val, free_th, occ_th
        )
        if gp is not None and gp != seed:
            rm_try = _bfs_mask(gp, passable_bfs_main, w, h)
            if _count_reachable(rm_try) > _count_reachable(rm):
                seed, sd_m, smeth = gp, gd, "seed_reseed_low_reach"
                rm = rm_try
                rs = _bfs_mask(seed, passable_bfs_staging, w, h)
    return seed, rm, rs, sd_m, smeth


def format_reachability_debug_line(
    robot_xy: Tuple[float, float],
    robot_ixy: Optional[Tuple[int, int]],
    data: Sequence[int],
    w: int,
    h: int,
    passable: Sequence[bool],
    passable_staging: Sequence[bool],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    seed: Optional[Tuple[int, int]],
    seed_dist_m: float,
    seed_method: str,
    reachable_main: Sequence[bool],
    reachable_staging: Sequence[bool],
    local_radius_m: float,
) -> str:
    _ = (data, w, h, ox, oy, res, occ_th, free_th, unknown_val, passable, passable_staging)
    rx, ry = robot_xy
    ri = robot_ixy
    cnt_m = sum(1 for x in reachable_main if x)
    cnt_s = sum(1 for x in reachable_staging if x)
    return (
        f"REACHABILITY robot=({rx:.2f},{ry:.2f}) cell={ri} seed={seed} "
        f"seed_d={seed_dist_m:.2f}m method={seed_method} "
        f"reachable_cells={cnt_m} staging_reach={cnt_s} r_local={local_radius_m:.2f}"
    )


def score_cluster_distance(
    cl: FrontierCluster,
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    size_weight: float,
) -> float:
    cx, cy = cl.centroid_map
    wx = ox + (cx + 0.5) * res
    wy = oy + (cy + 0.5) * res
    dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
    return dist - size_weight * math.sqrt(float(cl.size))


def _blacklisted(wx: float, wy: float, blacklist: Sequence[Tuple[float, float, float]], rad: float) -> bool:
    for bx, by, _t in blacklist:
        if math.hypot(wx - bx, wy - by) < rad:
            return True
    return False


def format_strict_mask_debug(
    data: Sequence[int],
    w: int,
    h: int,
    passable: Sequence[bool],
    pass_bfs: Sequence[bool],
    reach_bfs: Sequence[bool],
    unk: int,
    free: int,
    occ: int,
) -> str:
    """Per-filter counts showing why reach_strict may be 0 while reach_bfs > 0."""
    n = w * h
    reachable_bfs_count = sum(1 for x in reach_bfs if x)
    after_obstacle_inflation_count = sum(1 for x in pass_bfs if x)
    after_unknown_clearance_count = 0
    after_goal_inflation_count = sum(1 for x in passable if x)
    for i in range(n):
        if not reach_bfs[i]:
            continue
        if _is_free(int(data[i]), free, occ, unk):
            after_unknown_clearance_count += 1
    final_reach_strict_count = sum(
        1 for i in range(n) if passable[i] and reach_bfs[i]
    )
    return (
        "STRICT_MASK_DEBUG "
        f"reachable_bfs_count={reachable_bfs_count} "
        f"after_obstacle_inflation_count={after_obstacle_inflation_count} "
        f"after_unknown_clearance_count={after_unknown_clearance_count} "
        f"after_goal_inflation_count={after_goal_inflation_count} "
        f"final_reach_strict_count={final_reach_strict_count}"
    )


def build_goal_reach_mask(
    passable: Sequence[bool],
    reach_bfs: Sequence[bool],
    require_strict: bool,
) -> List[bool]:
    """V1: use BFS reach when strict mask is empty or strict reach not required."""
    n = min(len(passable), len(reach_bfs))
    reach_strict_count = sum(
        1 for i in range(n) if passable[i] and reach_bfs[i]
    )
    reach_bfs_count = sum(1 for x in reach_bfs if x)
    if not require_strict or (reach_strict_count == 0 and reach_bfs_count > 0):
        return [bool(reach_bfs[i]) for i in range(n)]
    return [bool(passable[i] and reach_bfs[i]) for i in range(n)]


def _occupied_clearance_ok(
    gix: int,
    giy: int,
    data: Sequence[int],
    w: int,
    h: int,
    occ_th: int,
    clearance_m: float,
    res: float,
) -> bool:
    if clearance_m <= 0.0:
        return True
    r = int(math.ceil(clearance_m / max(res, 1e-9)))
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            cx, cy = gix + dx, giy + dy
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                continue
            if _is_occupied(int(data[_idx(cx, cy, w)]), occ_th):
                return False
    return True


def _validate_approach_candidate(
    gix: int,
    giy: int,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    require_passable: bool,
    occ_th: int,
    unknown_val: int,
    free_th: int,
    min_occupied_clearance_m: float,
    res: float,
) -> Optional[str]:
    if gix < 0 or giy < 0 or gix >= w or giy >= h:
        return "out_of_bounds"
    gi = _idx(gix, giy, w)
    v = int(data[gi])
    if _is_unknown(v, unknown_val):
        return "unknown"
    if _is_occupied(v, occ_th):
        return "occupied"
    if not _is_free(v, free_th, occ_th, unknown_val):
        return "not_free"
    if gi >= len(goal_reach_mask) or not goal_reach_mask[gi]:
        return "not_reachable"
    if require_passable and passable is not None:
        if gi >= len(passable) or not passable[gi]:
            return "not_passable"
    if not _occupied_clearance_ok(
        gix, giy, data, w, h, occ_th, min_occupied_clearance_m, res
    ):
        return "too_close_obstacle"
    _ = free_th
    return None


def _cells_in_annulus(
    cx: float,
    cy: float,
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    radius_min_m: float,
    radius_max_m: float,
) -> List[Tuple[float, float, int, int]]:
    """Return (dist_to_centroid_m, wx, wy, mx, my) for map cells in annulus."""
    r_min_c = int(math.floor(radius_min_m / max(res, 1e-9)))
    r_max_c = int(math.ceil(radius_max_m / max(res, 1e-9))) + 1
    mx0 = int(math.floor(cx))
    my0 = int(math.floor(cy))
    out: List[Tuple[float, float, int, int]] = []
    for my in range(max(0, my0 - r_max_c), min(h, my0 + r_max_c + 1)):
        for mx in range(max(0, mx0 - r_max_c), min(w, mx0 + r_max_c + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - (ox + (cx + 0.5) * res), wy - (oy + (cy + 0.5) * res))
            if d < radius_min_m - 1e-6 or d > radius_max_m + 1e-6:
                continue
            out.append((d, wx, wy, mx, my))
    return out


def pick_cluster_approach_cell(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    approach_radius_min_m: float,
    approach_radius_max_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
) -> Tuple[Optional[Tuple[int, int]], str, ApproachPickStats]:
    """V1: search known-free reachable cells in annulus around cluster centroid."""
    rx, ry = robot_xy
    ccx, ccy = cl.centroid_map
    stats = ApproachPickStats()
    annulus = _cells_in_annulus(
        ccx, ccy, w, h, ox, oy, res, approach_radius_min_m, approach_radius_max_m
    )
    stats.candidates_total = len(annulus)
    valid: List[Tuple[float, float, int, int]] = []
    for _dc, wx, wy, mx, my in annulus:
        rej = _validate_approach_candidate(
            mx,
            my,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            require_passable_for_approach,
            occ_th,
            unknown_val,
            free_th,
            min_occupied_clearance_m,
            res,
        )
        if rej is not None:
            if rej == "unknown":
                stats.rejected_unknown += 1
            elif rej == "occupied":
                stats.rejected_occupied += 1
            elif rej == "not_reachable":
                stats.rejected_not_reachable += 1
            elif rej == "not_passable":
                stats.rejected_not_passable += 1
            elif rej == "too_close_obstacle":
                stats.rejected_too_close_obstacle += 1
            elif rej == "out_of_bounds":
                stats.rejected_out_of_bounds += 1
            continue
        if _blacklisted(wx, wy, blacklist, blacklist_radius):
            stats.rejected_blacklist += 1
            continue
        dr = math.hypot(wx - rx, wy - ry)
        if dr < min_robot_dist_m or dr > max_robot_dist_m:
            stats.rejected_robot_distance += 1
            continue
        valid.append((_dc, wx, wy, mx, my))
    if not valid:
        return None, "no_cluster_approach", stats
    valid.sort(key=lambda t: t[0])
    _dc, _wx, _wy, mx, my = valid[0]
    return (mx, my), "centroid_annulus", stats


def pick_cluster_neighbor_fallback(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    neighbor_radius_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
) -> Tuple[Optional[Tuple[int, int]], str, ApproachPickStats]:
    """Nearest reachable known-free cell within neighbor_radius_m of any frontier cell."""
    rx, ry = robot_xy
    stats = ApproachPickStats()
    r_cells = int(math.ceil(neighbor_radius_m / max(res, 1e-9))) + 1
    best: Optional[Tuple[float, float, int, int]] = None
    for fmx, fmy in cl.cells:
        for my in range(max(0, fmy - r_cells), min(h, fmy + r_cells + 1)):
            for mx in range(max(0, fmx - r_cells), min(w, fmx + r_cells + 1)):
                if (mx, my) in cl.cells:
                    continue
                stats.candidates_total += 1
                wx = ox + (mx + 0.5) * res
                wy = oy + (my + 0.5) * res
                if math.hypot(wx - (ox + (fmx + 0.5) * res), wy - (oy + (fmy + 0.5) * res)) > (
                    neighbor_radius_m + 1e-6
                ):
                    stats.rejected_annulus += 1
                    continue
                rej = _validate_approach_candidate(
                    mx,
                    my,
                    data,
                    w,
                    h,
                    goal_reach_mask,
                    passable,
                    require_passable_for_approach,
                    occ_th,
                    unknown_val,
                    free_th,
                    min_occupied_clearance_m,
                    res,
                )
                if rej is not None:
                    if rej == "unknown":
                        stats.rejected_unknown += 1
                    elif rej == "occupied":
                        stats.rejected_occupied += 1
                    elif rej == "not_reachable":
                        stats.rejected_not_reachable += 1
                    elif rej == "not_passable":
                        stats.rejected_not_passable += 1
                    elif rej == "too_close_obstacle":
                        stats.rejected_too_close_obstacle += 1
                    continue
                if _blacklisted(wx, wy, blacklist, blacklist_radius):
                    stats.rejected_blacklist += 1
                    continue
                dr = math.hypot(wx - rx, wy - ry)
                if dr < min_robot_dist_m or dr > max_robot_dist_m:
                    stats.rejected_robot_distance += 1
                    continue
                if best is None or dr < best[0]:
                    best = (dr, wx, wy, mx, my)
    if best is None:
        return None, "neighbor_fallback_failed", stats
    return (best[3], best[4]), "neighbor_fallback", stats


def validate_and_build_goal(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    robot_xy: Tuple[float, float],
    robot_ixy: Optional[Tuple[int, int]],
    ox: float,
    oy: float,
    res: float,
    passable: Sequence[bool],
    goal_reach_mask: Sequence[bool],
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    approach_radius_min_m: float,
    approach_radius_max_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
    neighbor_fallback_m: float,
    _fallback_radius_min_m: float,
    _fallback_radius_max_m: float,
    enable_staging_goal: bool,
    _staging_min_distance_m: float,
    _staging_max_distance_m: float,
    _staging_fan_angles_deg: Optional[Tuple[float, ...]],
    _staging_fan_distances_m: Optional[Tuple[float, ...]],
    _staging_require_free_value_zero: bool,
    passable_staging: Optional[List[bool]],
    reachable_staging_mask: Optional[List[bool]],
) -> Tuple[Optional[ValidatedGoal], Optional[RejectReason], str, ApproachPickStats]:
    _ = (
        robot_ixy,
        enable_staging_goal,
        _staging_fan_angles_deg,
        _staging_fan_distances_m,
        _staging_require_free_value_zero,
        passable_staging,
        reachable_staging_mask,
        _fallback_radius_min_m,
        _fallback_radius_max_m,
    )
    ccx, ccy = cl.centroid_map
    cwx = ox + (ccx + 0.5) * res
    cwy = oy + (ccy + 0.5) * res

    picked, method, stats = pick_cluster_approach_cell(
        cl,
        data,
        w,
        h,
        goal_reach_mask,
        passable,
        robot_xy,
        ox,
        oy,
        res,
        occ_th,
        free_th,
        unknown_val,
        blacklist,
        blacklist_radius,
        min_robot_dist_m,
        max_robot_dist_m,
        approach_radius_min_m,
        approach_radius_max_m,
        min_occupied_clearance_m,
        require_passable_for_approach,
    )
    if picked is None:
        picked, method, stats = pick_cluster_neighbor_fallback(
            cl,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            robot_xy,
            ox,
            oy,
            res,
            occ_th,
            free_th,
            unknown_val,
            blacklist,
            blacklist_radius,
            min_robot_dist_m,
            max_robot_dist_m,
            neighbor_fallback_m,
            min_occupied_clearance_m,
            require_passable_for_approach,
        )
        if picked is not None:
            method = "CLUSTER_APPROACH_FAILED: using nearest reachable free neighbor fallback"
    if picked is None:
        return None, RejectReason.NO_APPROACH_CELL, stats.summary_line(), stats
    gix, giy = picked
    wx = ox + (gix + 0.5) * res
    wy = oy + (giy + 0.5) * res

    yaw = math.atan2(cwy - wy, cwx - wx)
    return (
        ValidatedGoal(
            wx=wx,
            wy=wy,
            yaw=yaw,
            approach_ixy=(gix, giy),
            cluster=cl,
            approach_method=method,
        ),
        None,
        method,
        stats,
    )
