"""경로 / 미션 플래너 (격자 그래프 위 BFS).

이 turn 의 범위:
  - shortest_path : 두 노드 간 최단 경로 (BFS, uniform edge cost)
  - visit_in_order: 시작 노드 + 순서 있는 waypoint 리스트 → 합쳐진 경로
  - arrange_by_id : marker id 순서대로 marker 노드를 방문

다음 turn 에서:
  - executor (path → 교차점별 액션 / yaw 명령)
  - localization (DR + ArUco-PnP) 와 결합

설계 원칙: 순수 함수, 사이드이펙트 없음. node 은 grid.Node = (i, j).
"""
from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence

from .grid import Grid, Node


def shortest_path(grid: Grid, start: Node, goal: Node) -> List[Node]:
    """BFS 로 start → goal 최단 경로. start==goal 이면 [start].

    도달 불가하면 ValueError (4-connectivity 격자에서는 사실상
    in_bounds 노드면 항상 도달 가능하므로 OOB 일 때만 raise 됨).
    """
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        raise ValueError(f"node out of bounds: start={start}, goal={goal}")
    if start == goal:
        return [start]

    came_from: Dict[Node, Optional[Node]] = {start: None}
    queue: deque[Node] = deque([start])
    while queue:
        cur = queue.popleft()
        if cur == goal:
            return _reconstruct(came_from, cur)
        for nb in grid.neighbors(cur):
            if nb not in came_from:
                came_from[nb] = cur
                queue.append(nb)
    raise ValueError(f"no path from {start} to {goal}")


def visit_in_order(
    grid: Grid, start: Node, waypoints: Sequence[Node]
) -> List[Node]:
    """start → wp[0] → wp[1] → ... 순서로 BFS 경로를 이어 붙임.

    각 segment 의 끝 노드와 다음 segment 의 시작 노드는 동일하므로
    중복 1개를 제거해서 합친다. waypoints 가 비어 있으면 [start].
    """
    if not waypoints:
        return [start]
    full: List[Node] = []
    cursor = start
    for wp in waypoints:
        seg = shortest_path(grid, cursor, wp)
        if full and full[-1] == seg[0]:
            seg = seg[1:]
        full.extend(seg)
        cursor = wp
    return full


def arrange_by_id(
    grid: Grid, start: Node, marker_ids: Sequence[int]
) -> List[Node]:
    """주어진 marker id 순서대로 marker 노드를 방문하는 경로."""
    waypoints = [grid.marker_node(mid) for mid in marker_ids]
    return visit_in_order(grid, start, waypoints)


def path_length_m(grid: Grid, path: Sequence[Node]) -> float:
    """경로의 총 길이 [m]. 빈/단일 노드는 0."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        ax, ay = grid.world(a)
        bx, by = grid.world(b)
        total += abs(ax - bx) + abs(ay - by)   # 격자 위 → Manhattan == 실거리
    return total


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _reconstruct(came_from: Dict[Node, Optional[Node]], goal: Node) -> List[Node]:
    path: List[Node] = []
    cur: Optional[Node] = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path
