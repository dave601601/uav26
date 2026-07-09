"""line_tracer.grid 단위 테스트."""
from __future__ import annotations

import pytest

from line_tracer.grid import Grid, line_positions


# ---------------------------------------------------------------------------
# line_positions
# ---------------------------------------------------------------------------

def test_line_positions_exact_division():
    assert line_positions(20.0, 4.0) == [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]


def test_line_positions_partial_last_cell():
    # 30 / 4 = 7.5 → [0,4,...,28] + extent(30)
    assert line_positions(30.0, 4.0) == [0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 30.0]


# ---------------------------------------------------------------------------
# Grid shape
# ---------------------------------------------------------------------------

def test_grid_extents_30x20():
    g = Grid.from_extents(30, 20, 4)
    assert g.shape == (9, 6)
    assert g.xs == (0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 30.0)
    assert g.ys == (0.0, 4.0, 8.0, 12.0, 16.0, 20.0)


def test_grid_in_bounds():
    g = Grid.from_extents(30, 20, 4)
    assert g.in_bounds((0, 0))
    assert g.in_bounds((8, 5))
    assert not g.in_bounds((-1, 0))
    assert not g.in_bounds((9, 0))
    assert not g.in_bounds((0, 6))


def test_grid_world_lookup():
    g = Grid.from_extents(30, 20, 4)
    assert g.world((0, 0)) == (0.0, 0.0)
    assert g.world((1, 1)) == (4.0, 4.0)
    assert g.world((8, 5)) == (30.0, 20.0)


def test_grid_nearest_node_round_trip():
    g = Grid.from_extents(30, 20, 4)
    assert g.nearest_node(4.0, 4.0) == (1, 1)
    assert g.nearest_node(4.49, 4.49) == (1, 1)   # 4.49 rounds to 4
    assert g.nearest_node(5.51, 5.51) == (1, 1)   # 5.51 still closer to 4 than 8


# ---------------------------------------------------------------------------
# neighbors
# ---------------------------------------------------------------------------

def test_neighbors_corner():
    g = Grid.from_extents(30, 20, 4)
    assert set(g.neighbors((0, 0))) == {(1, 0), (0, 1)}


def test_neighbors_edge():
    g = Grid.from_extents(30, 20, 4)
    assert set(g.neighbors((0, 1))) == {(0, 0), (0, 2), (1, 1)}


def test_neighbors_interior():
    g = Grid.from_extents(30, 20, 4)
    assert set(g.neighbors((4, 3))) == {(3, 3), (5, 3), (4, 2), (4, 4)}


def test_neighbors_far_corner():
    g = Grid.from_extents(30, 20, 4)
    assert set(g.neighbors((8, 5))) == {(7, 5), (8, 4)}


# ---------------------------------------------------------------------------
# markers
# ---------------------------------------------------------------------------

# competition aruco_layout.yaml 의 9 개 마커
# Official-spec shape: 30x21 m, 3 m cells, interior vertices only,
# 4 unique IDs from 0..49.
COMPETITION_LAYOUT = {
    17: (21.0, 15.0),
    15: (6.0, 6.0),
    14: (3.0, 6.0),
    8: (24.0, 18.0),
}


def test_marker_node_competition_layout():
    g = Grid.from_extents(30, 21, 3, marker_xy=COMPETITION_LAYOUT)
    assert g.marker_node(17) == (7, 5)   # (21, 15)
    assert g.marker_node(15) == (2, 2)   # (6, 6)
    assert g.marker_node(14) == (1, 2)   # (3, 6)
    assert g.marker_node(8) == (8, 6)    # (24, 18)


def test_marker_node_unknown_id():
    g = Grid.from_extents(30, 21, 3, marker_xy=COMPETITION_LAYOUT)
    with pytest.raises(KeyError):
        g.marker_node(99)


def test_marker_node_off_grid_intersection():
    g = Grid.from_extents(30, 20, 4, marker_xy={42: (5.0, 5.0)})  # 격자 위 아님
    with pytest.raises(ValueError):
        g.marker_node(42)
