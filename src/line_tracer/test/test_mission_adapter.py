"""Unit tests for line_tracer.mission_adapter (pure functions, no rclpy)."""
from __future__ import annotations

from math import isclose, pi

import pytest

from line_tracer.mission_adapter import (
    line_angle_error_rad,
    line_offsets_m,
    marker_center_errors_m,
    nearest_marker,
)


# ---------------------------------------------------------------------------
# line_offsets_m — both offsets, presence, signs
# ---------------------------------------------------------------------------

def test_line_offsets_present_signs_and_scale():
    # alt=2, fx=fy=400. du=+100 (line right of center) -> dx = -100*2/400 = -0.5
    # dv=-80 (line above center) -> dy = -(-80)*2/400 = +0.4
    dx, has_v, dy, has_h = line_offsets_m(100.0, -80.0, 2.0, 400.0, 400.0)
    assert has_v is True and has_h is True
    assert isclose(dx, -0.5, abs_tol=1e-9)
    assert isclose(dy, 0.4, abs_tol=1e-9)


def test_line_offsets_absent_are_zero_and_false():
    dx, has_v, dy, has_h = line_offsets_m(None, None, 2.0, 400.0, 400.0)
    assert (dx, has_v, dy, has_h) == (0.0, False, 0.0, False)


def test_line_offsets_one_axis_only():
    dx, has_v, dy, has_h = line_offsets_m(40.0, None, 3.0, 600.0, 600.0)
    assert has_v is True and has_h is False
    assert isclose(dx, -40.0 * 3.0 / 600.0, abs_tol=1e-9)
    assert dy == 0.0


# ---------------------------------------------------------------------------
# line_angle_error_rad — travel-axis selection + horizontal fold
# ---------------------------------------------------------------------------

def test_angle_x_travel_passes_psi_err_through():
    assert line_angle_error_rad("x", 0.123, None) == 0.123
    assert line_angle_error_rad("x", None, (0, 0, 10, 0)) is None


def test_angle_y_travel_none_without_horizontal_line():
    assert line_angle_error_rad("y", 0.5, None) is None


def test_angle_y_travel_exactly_horizontal_is_zero():
    # Perfectly horizontal segment -> 0 error.
    a = line_angle_error_rad("y", None, (0, 100, 100, 100))
    assert isclose(a, 0.0, abs_tol=1e-9)


def test_angle_y_travel_small_tilt_is_small_signed():
    # Line tilting up toward +u (image v decreases) -> small positive (+CCW).
    a = line_angle_error_rad("y", None, (0, 100, 100, 90))
    assert 0.0 < a < 0.2
    # The same physical line near pi folds to the small negative counterpart.
    b = line_angle_error_rad("y", None, (0, 90, 100, 100))
    assert -0.2 < b < 0.0
    assert isclose(a, -b, abs_tol=1e-6)


def test_angle_rejects_bad_axis():
    with pytest.raises(ValueError):
        line_angle_error_rad("z", 0.0, None)


# ---------------------------------------------------------------------------
# marker_center_errors_m — signs
# ---------------------------------------------------------------------------

def test_marker_center_errors_signs():
    # marker below+right of center (u>cx, v>cy) -> body -x and -y.
    ex, ey = marker_center_errors_m(
        u=340.0, v=220.0, cx=320.0, cy=200.0, altitude=2.0, fx=400.0, fy=400.0
    )
    # ex = -(220-200)*2/400 = -0.1 ; ey = -(340-320)*2/400 = -0.1
    assert isclose(ex, -0.1, abs_tol=1e-9)
    assert isclose(ey, -0.1, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# nearest_marker — nearest-to-center wins
# ---------------------------------------------------------------------------

def test_nearest_marker_picks_closest_to_center():
    markers = [(7, 300.0, 205.0), (3, 322.0, 201.0), (5, 100.0, 100.0)]
    chosen = nearest_marker(markers, cx=320.0, cy=200.0)
    assert chosen is not None
    assert chosen[0] == 3


def test_nearest_marker_empty_is_none():
    assert nearest_marker([], 320.0, 200.0) is None
