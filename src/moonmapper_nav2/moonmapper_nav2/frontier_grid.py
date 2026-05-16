"""Grid operations: frontiers, passable mask, BFS reachability, goal validation (V1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

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
    rings = {float(r) for r in checkpoints if r <= max_m + 1e-9}
    step = float(max(step_m, 0.05))
    cur = step
    while cur <= max_m + 1e-9:
        rings.add(round(cur, 4))
        cur += step
    return tuple(sorted(rings))


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


def find_nearest_bfs_seed(
    robot_ixy: Tuple[int, int],
    rx: float,
    ry: float,
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    max_radius_m: float,
    step_m: float,
) -> Tuple[Optional[Tuple[int, int]], float, str]:
    """Nearest cell with passable_bfs True inside expanding Euclidean discs (radii meters)."""
    rmx, rmy = robot_ixy
    for rd in build_bfs_seed_radii_m(max_radius_m, step_m):
        best_d = 1e18
        best: Optional[Tuple[int, int]] = None
        r_cells = int(rd / max(res, 1e-9)) + 3
        for my in range(max(0, rmy - r_cells), min(h, rmy + r_cells + 1)):
            for mx in range(max(0, rmx - r_cells), min(w, rmx + r_cells + 1)):
                wc_x = ox + (mx + 0.5) * res
                wc_y = oy + (my + 0.5) * res
                dm = math.hypot(wc_x - rx, wc_y - ry)
                if dm > rd + 1e-6:
                    continue
                idx = _idx(mx, my, w)
                if not passable_bfs[idx]:
                    continue
                if dm < best_d:
                    best_d = dm
                    best = (mx, my)
        if best is not None:
            return best, best_d, "nearest_free_radius"
    return None, 0.0, "none"


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
) -> Tuple[Optional[Tuple[int, int]], List[bool], List[bool], float, str]:
    rx, ry = robot_xy
    _ = data, unknown_val, free_th, occ_th
    seed, sd_m, smeth = find_nearest_bfs_seed(
        robot_ixy,
        rx,
        ry,
        passable_bfs_main,
        w,
        h,
        ox,
        oy,
        res,
        bfs_seed_radius_m,
        bfs_seed_step_m,
    )
    rm = _bfs_mask(seed, passable_bfs_main, w, h)
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
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_stage_dist_m: float,
    retreat_steps: int,
    _fallback_radius_min_m: float,
    _fallback_radius_max_m: float,
    enable_staging_goal: bool,
    _staging_min_distance_m: float,
    _staging_max_distance_m: float,
    _staging_fan_angles_deg: Optional[Tuple[float, ...]],
    _staging_fan_distances_m: Optional[Tuple[float, ...]],
    _staging_require_free_value_zero: bool,
    passable_staging: Optional[List[bool]],
    reachable_main_mask: Sequence[bool],
    reachable_staging_mask: Optional[List[bool]],
) -> Tuple[Optional[ValidatedGoal], Optional[RejectReason], str]:
    _ = (
        enable_staging_goal,
        _staging_fan_angles_deg,
        _staging_fan_distances_m,
        _staging_require_free_value_zero,
        passable_staging,
        reachable_staging_mask,
    )
    rx, ry = robot_xy
    ccx, ccy = cl.centroid_map
    cwx = ox + (ccx + 0.5) * res
    cwy = oy + (ccy + 0.5) * res

    if robot_ixy is not None:
        rfx = float(robot_ixy[0]) + 0.5
        rfy = float(robot_ixy[1]) + 0.5
    else:
        rfx = (rx - ox) / res
        rfy = (ry - oy) / res

    gx, gy = float(ccx), float(ccy)
    vx = ccx - rfx
    vy = ccy - rfy
    norm = math.hypot(vx, vy) or 1.0
    vx /= norm
    vy /= norm
    for _ in range(max(0, retreat_steps)):
        gx -= vx
        gy -= vy

    gix, giy = int(round(gx)), int(round(gy))
    if gix < 0 or giy < 0 or gix >= w or giy >= h:
        return None, RejectReason.NO_APPROACH_CELL, "goal_out_of_bounds"
    gi = _idx(gix, giy, w)
    v = int(data[gi])
    if _is_unknown(v, unknown_val):
        return None, RejectReason.NO_APPROACH_CELL, "unknown_or_not_free"
    if _is_occupied(v, occ_th):
        return None, RejectReason.NO_APPROACH_CELL, "occupied"
    if not passable[gi]:
        return None, RejectReason.NO_APPROACH_CELL, "not_passable"
    if gi >= len(reachable_main_mask) or not reachable_main_mask[gi]:
        return None, RejectReason.NO_APPROACH_CELL, "not_reachable"

    wx = ox + (gix + 0.5) * res
    wy = oy + (giy + 0.5) * res
    d = math.hypot(wx - rx, wy - ry)
    if d < min_robot_dist_m:
        return None, RejectReason.NO_APPROACH_CELL, "too_close"
    if d > max_stage_dist_m:
        return None, RejectReason.NO_APPROACH_CELL, "too_far"
    if _blacklisted(wx, wy, blacklist, blacklist_radius):
        return None, RejectReason.NO_APPROACH_CELL, "blacklisted"
    _ = free_th

    yaw = math.atan2(cwy - wy, cwx - wx)
    method = "retreat" if retreat_steps > 0 else "direct"
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
        "",
    )
