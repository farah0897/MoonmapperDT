"""Small helpers for V1 occupancy-grid frontier logic."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class FrontierCluster:
    cells: List[Tuple[int, int]]

    @property
    def size(self) -> int:
        return len(self.cells)

    @property
    def centroid_map(self) -> Tuple[float, float]:
        if not self.cells:
            return 0.0, 0.0
        sx = sum(c[0] for c in self.cells)
        sy = sum(c[1] for c in self.cells)
        n = float(len(self.cells))
        return sx / n, sy / n


def cell_value(data: List[int], w: int, h: int, mx: int, my: int) -> Optional[int]:
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return None
    return int(data[my * w + mx])


def world_to_map(wx: float, wy: float, ox: float, oy: float, res: float) -> Tuple[int, int]:
    mx = int((wx - ox) / res)
    my = int((wy - oy) / res)
    return mx, my


def _is_unknown(v: int, unk: int) -> bool:
    return v == unk


def _is_occupied(v: int, occ_th: int) -> bool:
    return v >= occ_th


def _is_free(v: int, _free_th: int, occ_th: int, unk: int) -> bool:
    if _is_unknown(v, unk):
        return False
    return v < occ_th
