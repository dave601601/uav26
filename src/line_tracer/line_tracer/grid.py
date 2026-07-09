"""격자 그래프 모델.

Competition world (official spec: 3 m cell; default 30 × 21 m — arena
dims are an assumption until the rules confirm) 의 격자선을 그래프로 표현:
  - 노드: 격자 교차점 (i, j)  → world (xs[i], ys[j])
  - 엣지: 같은 i 또는 같은 j 의 인접 노드 (4-connectivity)

격자선 위치는 floor_tex.py 와 동일 규칙으로 만든다
(0, cell, 2*cell, ..., 마지막 < extent 이면 extent 추가).
즉 30 m / 3 m 격자 → x = 0,3,6,...,30 (11 col),
   21 m / 3 m 격자 → y = 0,3,6,...,21 (8 row).

마커 위치는 외부에서 (id → (x, y)) 로 전달; 보통 aruco_layout.yaml
값을 그대로 넘겨준다. 모든 marker 가 격자 교차점 위에 있다고 가정한다
(없으면 marker_node() 에서 ValueError).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Tuple

Node = Tuple[int, int]   # (i, j) — index into xs, ys


def line_positions(extent: float, cell: float) -> List[float]:
    """0, cell, 2*cell, ..., (마지막 < extent 면 extent 추가)."""
    n = int(extent / cell + 1e-9)
    out = [i * cell for i in range(n + 1)]
    if abs(out[-1] - extent) > 1e-6:
        out.append(extent)
    return out


@dataclass(frozen=True)
class Grid:
    width: float
    depth: float
    cell: float
    xs: Tuple[float, ...]
    ys: Tuple[float, ...]
    marker_xy: Dict[int, Tuple[float, float]] = field(default_factory=dict)

    @classmethod
    def from_extents(
        cls,
        width: float = 30.0,
        depth: float = 21.0,
        cell: float = 3.0,
        marker_xy: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> "Grid":
        xs = tuple(line_positions(width, cell))
        ys = tuple(line_positions(depth, cell))
        return cls(width, depth, cell, xs, ys, dict(marker_xy or {}))

    # -------- shape -------------------------------------------------------

    @property
    def shape(self) -> Tuple[int, int]:
        return (len(self.xs), len(self.ys))

    def nodes(self) -> Iterator[Node]:
        for i in range(len(self.xs)):
            for j in range(len(self.ys)):
                yield (i, j)

    def in_bounds(self, node: Node) -> bool:
        i, j = node
        return 0 <= i < len(self.xs) and 0 <= j < len(self.ys)

    # -------- world ↔ node ------------------------------------------------

    def world(self, node: Node) -> Tuple[float, float]:
        i, j = node
        return (self.xs[i], self.ys[j])

    def nearest_node(self, x: float, y: float) -> Node:
        i = min(range(len(self.xs)), key=lambda k: abs(self.xs[k] - x))
        j = min(range(len(self.ys)), key=lambda k: abs(self.ys[k] - y))
        return (i, j)

    # -------- adjacency ---------------------------------------------------

    def neighbors(self, node: Node) -> List[Node]:
        i, j = node
        cands = [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
        return [n for n in cands if self.in_bounds(n)]

    # -------- markers -----------------------------------------------------

    def marker_node(self, marker_id: int, tol: float = 1e-3) -> Node:
        """marker_id 의 (x, y) 와 가장 가까운 노드 반환.

        해당 좌표가 격자 교차점 위에 있지 않으면 ValueError.
        """
        if marker_id not in self.marker_xy:
            raise KeyError(f"marker id {marker_id} not in layout")
        x, y = self.marker_xy[marker_id]
        n = self.nearest_node(x, y)
        nx, ny = self.world(n)
        if abs(nx - x) > tol or abs(ny - y) > tol:
            raise ValueError(
                f"marker {marker_id} at ({x}, {y}) is not on a grid intersection "
                f"(closest = ({nx}, {ny}))"
            )
        return n
