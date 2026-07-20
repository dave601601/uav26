"""Unit tests for line_tracer.mission_adapter.select_front_hint.

The selector is pure geometry: given voted front-camera candidates (each with
a grid node and a world xy), the drone position and its travel direction, it
returns the nearest candidate that lies ahead on the current row line. Recorded
-id exclusion is the mission layer's job, so the selector still orders by
distance and lets duplicates through.
"""
from __future__ import annotations

from types import SimpleNamespace

from line_tracer.mission import MoveDirection
from line_tracer.mission_adapter import select_front_hint


def _cand(node, x, y):
    """Minimal stand-in for side_camera.Candidate (only .node and .xy used)."""
    return SimpleNamespace(node=node, xy=(x, y))


def test_none_when_no_candidates_or_no_dr():
    assert select_front_hint({}, (0.0, 0.0), MoveDirection.X_POS) is None
    assert (
        select_front_hint({5: _cand((2, 0), 6.0, 0.0)}, None, MoveDirection.X_POS)
        is None
    )


def test_ahead_candidate_selected_distance_is_along_track():
    # X_POS travel, drone at x=0; candidate 6 m ahead on the same row.
    cands = {5: _cand((2, 0), 6.0, 0.0)}
    hit = select_front_hint(cands, (0.0, 0.0), MoveDirection.X_POS)
    assert hit is not None
    marker_id, node, dist = hit
    assert marker_id == 5 and node == (2, 0)
    assert abs(dist - 6.0) < 1e-9


def test_behind_candidate_rejected():
    # Candidate 3 m behind the drone along +x.
    cands = {5: _cand((0, 0), -3.0, 0.0)}
    assert select_front_hint(cands, (0.0, 0.0), MoveDirection.X_POS) is None


def test_lateral_tolerance_excludes_off_row():
    # 6 m ahead but 2 m off the row line; default tol 1.5 rejects it.
    cands = {5: _cand((2, 1), 6.0, 2.0)}
    assert select_front_hint(cands, (0.0, 0.0), MoveDirection.X_POS) is None
    # A wider tolerance admits it.
    hit = select_front_hint(
        cands, (0.0, 0.0), MoveDirection.X_POS, row_tolerance_m=2.5
    )
    assert hit is not None and hit[0] == 5


def test_nearest_wins_and_recorded_not_excluded_here():
    # Two ahead candidates; the nearest along-track wins. The selector does
    # NOT exclude recorded ids (that is the mission layer's job).
    cands = {
        7: _cand((3, 0), 9.0, 0.0),
        4: _cand((1, 0), 3.0, 0.0),
    }
    hit = select_front_hint(cands, (0.0, 0.0), MoveDirection.X_POS)
    assert hit is not None
    assert hit[0] == 4 and abs(hit[2] - 3.0) < 1e-9


def test_direction_flip_x_neg():
    # X_NEG: ahead means smaller x. Drone at x=6, candidate at x=0 is 6 m ahead.
    cands = {5: _cand((0, 0), 0.0, 0.0)}
    hit = select_front_hint(cands, (6.0, 0.0), MoveDirection.X_NEG)
    assert hit is not None and hit[0] == 5 and abs(hit[2] - 6.0) < 1e-9
    # A candidate at x=9 (past the drone along +x) is behind under X_NEG.
    behind = {5: _cand((3, 0), 9.0, 0.0)}
    assert select_front_hint(behind, (6.0, 0.0), MoveDirection.X_NEG) is None


def test_direction_flip_y_neg():
    # Y_NEG: ahead means smaller y; lateral is the x offset from the column.
    cands = {8: _cand((0, 0), 0.0, 0.0)}
    hit = select_front_hint(cands, (0.0, 6.0), MoveDirection.Y_NEG)
    assert hit is not None and hit[0] == 8 and abs(hit[2] - 6.0) < 1e-9
    # Off-column laterally beyond tolerance -> rejected.
    off = {8: _cand((1, 0), 2.0, 0.0)}
    assert select_front_hint(off, (0.0, 6.0), MoveDirection.Y_NEG) is None


def test_y_pos_ahead_and_along_track_distance():
    cands = {8: _cand((0, 2), 0.0, 6.0)}
    hit = select_front_hint(cands, (0.0, 0.0), MoveDirection.Y_POS)
    assert hit is not None and hit[0] == 8 and abs(hit[2] - 6.0) < 1e-9
