"""line_tracer.planner 단위 테스트."""
from __future__ import annotations

import pytest

from line_tracer.grid import Grid
from line_tracer.planner import (
    arrange_by_id,
    path_length_m,
    shortest_path,
    visit_in_order,
)

# competition aruco_layout.yaml 와 동일
COMPETITION_LAYOUT = {
    0: (4.0, 4.0),
    1: (12.0, 4.0),
    2: (20.0, 4.0),
    3: (28.0, 4.0),
    4: (8.0, 12.0),
    5: (16.0, 12.0),
    6: (24.0, 12.0),
    7: (12.0, 16.0),
    8: (20.0, 16.0),
}


# ---------------------------------------------------------------------------
# shortest_path
# ---------------------------------------------------------------------------

def test_shortest_path_same_node():
    g = Grid.from_extents(30, 20, 4)
    assert shortest_path(g, (0, 0), (0, 0)) == [(0, 0)]


def test_shortest_path_straight_line():
    g = Grid.from_extents(30, 20, 4)
    assert shortest_path(g, (0, 0), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_shortest_path_l_shape_length_only():
    """대각 (2, 3) 까지: Manhattan 5 → 노드 6개. 두 가지 L-경로 모두 허용."""
    g = Grid.from_extents(30, 20, 4)
    p = shortest_path(g, (0, 0), (2, 3))
    assert len(p) == 6
    assert p[0] == (0, 0) and p[-1] == (2, 3)
    # 인접성 확인
    for a, b in zip(p[:-1], p[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def test_shortest_path_oob_raises():
    g = Grid.from_extents(30, 20, 4)
    with pytest.raises(ValueError):
        shortest_path(g, (-1, 0), (0, 0))
    with pytest.raises(ValueError):
        shortest_path(g, (0, 0), (99, 0))


# ---------------------------------------------------------------------------
# visit_in_order
# ---------------------------------------------------------------------------

def test_visit_in_order_empty():
    g = Grid.from_extents(30, 20, 4)
    assert visit_in_order(g, (2, 1), waypoints=[]) == [(2, 1)]


def test_visit_in_order_dedup_seam():
    """segment 경계 노드가 중복되지 않아야 함."""
    g = Grid.from_extents(30, 20, 4)
    p = visit_in_order(g, (0, 0), [(2, 0), (2, 2)])
    assert p == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]


def test_visit_in_order_three_waypoints():
    g = Grid.from_extents(30, 20, 4)
    p = visit_in_order(g, (0, 0), [(1, 0), (1, 1), (0, 1)])
    assert p == [(0, 0), (1, 0), (1, 1), (0, 1)]


# ---------------------------------------------------------------------------
# arrange_by_id
# ---------------------------------------------------------------------------

def test_arrange_by_id_along_y4_row():
    g = Grid.from_extents(30, 20, 4, marker_xy=COMPETITION_LAYOUT)
    p = arrange_by_id(g, start=(0, 1), marker_ids=[0, 1, 2])
    # marker0=(1,1), marker1=(3,1), marker2=(5,1) — 모두 y=4 줄 위
    assert p == [(0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (5, 1)]


def test_arrange_by_id_changes_row():
    g = Grid.from_extents(30, 20, 4, marker_xy=COMPETITION_LAYOUT)
    # start=(0,1)=(0,4); marker0=(1,1)=(4,4); marker4=(2,3)=(8,12).
    # leg1 Manhattan = 1, leg2 Manhattan = 1+2 = 3 → 1 + 1 + 3 = 5 노드.
    p = arrange_by_id(g, start=(0, 1), marker_ids=[0, 4])
    assert p[0] == (0, 1) and p[-1] == (2, 3)
    assert len(p) == 5
    for a, b in zip(p[:-1], p[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


# ---------------------------------------------------------------------------
# path_length_m
# ---------------------------------------------------------------------------

def test_path_length_zero_for_short_inputs():
    g = Grid.from_extents(30, 20, 4)
    assert path_length_m(g, []) == 0.0
    assert path_length_m(g, [(2, 2)]) == 0.0


def test_path_length_straight():
    g = Grid.from_extents(30, 20, 4)
    p = [(0, 0), (1, 0), (2, 0)]   # 0 → 4 → 8 : 8 m
    assert path_length_m(g, p) == pytest.approx(8.0)


def test_path_length_partial_last_cell():
    """x=28 → x=30 셀은 2 m (나머지 4 m 셀 다음에 붙는 부분)."""
    g = Grid.from_extents(30, 20, 4)
    p = [(7, 0), (8, 0)]   # (28, 0) → (30, 0)
    assert path_length_m(g, p) == pytest.approx(2.0)
