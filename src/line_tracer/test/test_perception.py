"""Unit tests for line_tracer.perception (no rclpy)."""
from dataclasses import replace
from math import isclose, pi

import cv2
import numpy as np
import pytest

from line_tracer.geom import CameraIntrinsics
from line_tracer.perception import (
    ArucoDetection,
    PerceptionConfig,
    classify_lines,
    compute_pixel_errors,
    detect_aruco,
    detect_lines,
    pick_nearest_line,
    process_image,
    _line_angle_in_0_pi,
    _signed_perp_du,
    _signed_perp_dv,
)


@pytest.fixture
def intr():
    return CameraIntrinsics(fx=465.6, fy=465.6, cx=320.0, cy=240.0,
                            width=640, height=480)


@pytest.fixture
def cfg():
    return PerceptionConfig()


# ---------------------------------------------------------------------------
# Pure geometry helpers
# ---------------------------------------------------------------------------

class TestLineAngle:
    def test_horizontal_line_is_zero(self):
        assert isclose(_line_angle_in_0_pi(1.0, 0.0), 0.0)

    def test_vertical_line_is_pi_over_2(self):
        # going up in image (dy=-1) is the same line as going down (dy=+1)
        assert isclose(_line_angle_in_0_pi(0.0, -1.0), pi / 2.0)
        assert isclose(_line_angle_in_0_pi(0.0, +1.0), pi / 2.0)

    def test_angle_is_invariant_to_direction_flip(self):
        a = _line_angle_in_0_pi(1.0, -2.0)
        b = _line_angle_in_0_pi(-1.0, 2.0)
        assert isclose(a, b)

    def test_angle_in_0_pi_range(self):
        for dx, dy in [(1, 1), (1, -1), (-1, -1), (-1, 1), (3, -7)]:
            a = _line_angle_in_0_pi(float(dx), float(dy))
            assert 0.0 <= a < pi


class TestSignedDu:
    def test_vertical_line_through_center_is_zero(self, intr):
        line = (320, 100, 320, 380)
        assert isclose(_signed_perp_du(line, intr.cx, intr.cy), 0.0, abs_tol=1e-6)

    def test_vertical_line_to_right_is_positive(self, intr):
        line = (450, 100, 450, 380)
        du = _signed_perp_du(line, intr.cx, intr.cy)
        assert du > 0
        assert isclose(du, 130.0, abs_tol=1e-6)

    def test_vertical_line_to_left_is_negative(self, intr):
        line = (200, 100, 200, 380)
        du = _signed_perp_du(line, intr.cx, intr.cy)
        assert du < 0
        assert isclose(du, -120.0, abs_tol=1e-6)

    def test_du_independent_of_segment_direction(self, intr):
        a = _signed_perp_du((450, 100, 450, 380), intr.cx, intr.cy)
        b = _signed_perp_du((450, 380, 450, 100), intr.cx, intr.cy)
        assert isclose(a, b, abs_tol=1e-6)


class TestSignedDv:
    def test_horizontal_through_center_is_zero(self, intr):
        line = (40, 240, 600, 240)
        assert isclose(_signed_perp_dv(line, intr.cx, intr.cy), 0.0, abs_tol=1e-6)

    def test_horizontal_below_is_positive(self, intr):
        line = (40, 400, 600, 400)
        dv = _signed_perp_dv(line, intr.cx, intr.cy)
        assert dv > 0
        assert isclose(dv, 160.0, abs_tol=1e-6)

    def test_horizontal_above_is_negative(self, intr):
        line = (40, 100, 600, 100)
        dv = _signed_perp_dv(line, intr.cx, intr.cy)
        assert dv < 0
        assert isclose(dv, -140.0, abs_tol=1e-6)


class TestComputePixelErrors:
    def test_perfect_alignment_yields_zeros(self, intr):
        v = (320, 50, 320, 430)
        h = (40, 240, 600, 240)
        du, dv, psi = compute_pixel_errors(v, h, intr)
        assert isclose(du, 0.0, abs_tol=1e-6)
        assert isclose(dv, 0.0, abs_tol=1e-6)
        assert isclose(psi, 0.0, abs_tol=1e-6)

    def test_psi_err_sign_for_tilted_top_to_right(self, intr):
        # Line whose top tilts toward +u (right) — should yield psi_err < 0
        # (so that wz = Kp*psi_err drives drone CW from above, aligning).
        v = (300, 400, 380, 80)   # bottom-left to top-right
        _, _, psi = compute_pixel_errors(v, None, intr)
        assert psi is not None and psi < 0

    def test_psi_err_sign_for_tilted_top_to_left(self, intr):
        # Line whose top tilts toward -u (left) — psi_err > 0 (CCW yaw).
        v = (380, 400, 300, 80)   # bottom-right to top-left
        _, _, psi = compute_pixel_errors(v, None, intr)
        assert psi is not None and psi > 0

    def test_missing_inputs_yield_none(self, intr):
        du, dv, psi = compute_pixel_errors(None, None, intr)
        assert du is None and dv is None and psi is None
        du, dv, psi = compute_pixel_errors((320, 50, 320, 430), None, intr)
        assert du == 0.0 and psi == 0.0 and dv is None


# ---------------------------------------------------------------------------
# Synthetic-image integration tests
# ---------------------------------------------------------------------------

def _draw_grid_image(width=640, height=480, line_thickness=10):
    """Official-spec polarity: WHITE satin lines on grass (mid-gray in
    mono) at u=320 and v=240. Canny+Hough is polarity-agnostic, but the
    fixture should mirror what the downward camera actually sees."""
    img = np.full((height, width, 3), 80, dtype=np.uint8)
    cv2.line(img, (320, 0), (320, height), (245, 245, 245), line_thickness)
    cv2.line(img, (0, 240), (width, 240), (245, 245, 245), line_thickness)
    return img


class TestDetectLines:
    def test_detects_lines_on_synthetic_grid(self, cfg):
        img = _draw_grid_image()
        lines = detect_lines(img, cfg)
        assert len(lines) > 0

    def test_classify_partitions_into_vertical_and_horizontal(self, cfg):
        img = _draw_grid_image()
        lines = detect_lines(img, cfg)
        vert, horiz = classify_lines(lines, cfg)
        assert len(vert) > 0
        assert len(horiz) > 0

    def test_pick_nearest_line_chooses_centered_line(self, cfg, intr):
        img = _draw_grid_image()
        lines = detect_lines(img, cfg)
        vert, _ = classify_lines(lines, cfg)
        v_line = pick_nearest_line(vert, intr.cx, intr.cy)
        assert v_line is not None
        # nearest centered vertical line should sit very close to u=320
        x1, _, x2, _ = v_line
        assert abs((x1 + x2) / 2.0 - intr.cx) < 8


class TestProcessImage:
    def test_centered_grid_yields_small_errors(self, cfg, intr):
        img = _draw_grid_image()
        res = process_image(img, intr, cfg)
        assert res.du is not None and abs(res.du) < 8
        assert res.dv is not None and abs(res.dv) < 8
        assert res.psi_err is not None and abs(res.psi_err) < 0.05

    def test_offset_grid_shifts_du_dv(self, cfg, intr):
        img = np.full((480, 640, 3), 255, dtype=np.uint8)
        # vertical line shifted right by 80 px, horizontal line shifted down by 60
        cv2.line(img, (400, 0), (400, 480), (0, 0, 0), 10)
        cv2.line(img, (0, 300), (640, 300), (0, 0, 0), 10)
        res = process_image(img, intr, cfg)
        assert res.du is not None and 60 < res.du < 100
        assert res.dv is not None and 40 < res.dv < 80


# ---------------------------------------------------------------------------
# ArUco
# ---------------------------------------------------------------------------

def _render_aruco_image(
    marker_id: int, size_px: int = 200, image_size=(480, 640), inverted: bool = True
):
    """Plant a marker sheet on a plain background.

    ``inverted`` renders the official spec — black sheet, white marker —
    which is what the world's textures now carry. ``inverted=False`` is
    the legacy black-on-white sheet, kept so the polarity can be pinned
    from both sides.
    """
    # Must match PerceptionConfig's default dictionary (4X4_50 — the
    # working assumption for the rules' "IDs 0..49").
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    bg = 255
    if inverted:
        marker = 255 - marker
        bg = 0
    img = np.full((*image_size, 3), bg, dtype=np.uint8)
    h, w = image_size
    y0 = (h - size_px) // 2
    x0 = (w - size_px) // 2
    img[y0:y0 + size_px, x0:x0 + size_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return img, (x0 + size_px / 2.0, y0 + size_px / 2.0)


class TestAruco:
    def test_detects_planted_marker(self, cfg):
        img, expected_center = _render_aruco_image(marker_id=7)
        dets = detect_aruco(img, cfg)
        assert len(dets) == 1
        d = dets[0]
        assert d.id == 7
        assert isclose(d.center_uv[0], expected_center[0], abs_tol=2.0)
        assert isclose(d.center_uv[1], expected_center[1], abs_tol=2.0)

    def test_no_marker_returns_empty(self, cfg):
        blank = np.full((480, 640, 3), 255, dtype=np.uint8)
        assert detect_aruco(blank, cfg) == []

    def test_polarity_is_pinned_to_the_official_spec(self, cfg):
        """Black sheet / white marker is the spec. OpenCV only builds
        candidate quads out of DARK regions and then requires a dark
        border ring, so the two polarities are mutually exclusive: with
        aruco_white_on_black the legacy black-on-white sheet must go
        undetected, and vice versa. Getting this backwards makes every
        marker invisible, which no other test would catch."""
        spec, _ = _render_aruco_image(marker_id=7, inverted=True)
        legacy, _ = _render_aruco_image(marker_id=7, inverted=False)

        assert [d.id for d in detect_aruco(spec, cfg)] == [7]
        assert detect_aruco(legacy, cfg) == []

        flipped = replace(cfg, aruco_white_on_black=False)
        assert [d.id for d in detect_aruco(legacy, flipped)] == [7]
        assert detect_aruco(spec, flipped) == []
