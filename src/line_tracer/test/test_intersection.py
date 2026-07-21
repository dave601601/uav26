"""Unit tests for the grid-crossing pulse detector (no rclpy).

Frames are synthetic white lines on a dark background, run through the real
detect_lines + classify_lines so the detector sees genuine Hough output, then
driven across simulated approach -> cross -> leave passes. The frame is 640x400
to match the spec's downward camera; the followed line and crossing line
orientations follow the travel axis (x: followed vertical, crossing horizontal;
y: followed horizontal, crossing vertical).
"""
from math import isclose

import cv2
import numpy as np
import pytest

from line_tracer.geom import CameraIntrinsics
from line_tracer.perception import (
    IntersectionConfig,
    IntersectionDetector,
    PerceptionConfig,
    classify_lines,
    detect_lines,
)

W, H = 640, 400
CX, CY = W / 2.0, H / 2.0


@pytest.fixture
def intr():
    return CameraIntrinsics(fx=600.0, fy=600.0, cx=CX, cy=CY, width=W, height=H)


@pytest.fixture
def cfg():
    return PerceptionConfig()


def _frame(segments, thickness=10):
    """Dark background with white segments; each segment is (x1,y1,x2,y2)."""
    img = np.full((H, W, 3), 60, dtype=np.uint8)
    for x1, y1, x2, y2 in segments:
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (245, 245, 245), thickness)
    return img


def _classify(img, cfg):
    return classify_lines(detect_lines(img, cfg), cfg)


def _run(detector, img, travel_axis, cfg, intr):
    vert, horiz = _classify(img, cfg)
    return detector.update(vert, horiz, travel_axis, intr)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

class TestConfig:
    def test_exit_must_exceed_enter(self):
        with pytest.raises(ValueError):
            IntersectionConfig(enter_px=80.0, exit_px=40.0)

    def test_default_bands_are_ordered(self):
        c = IntersectionConfig()
        assert c.exit_px > c.enter_px > 0


# ---------------------------------------------------------------------------
# Pulse semantics
# ---------------------------------------------------------------------------

class TestPulseXAxis:
    def _followed(self):
        return (int(CX), 0, int(CX), H)     # vertical followed line

    def test_exactly_one_pulse_per_pass(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        # crossing horizontal line sweeps top -> bottom across the center
        for v in range(20, H - 20, 12):
            img = _frame([self._followed(), (0, v, W, v)])
            ev = _run(det, img, "x", cfg, intr)
            fires += int(ev.detected)
        assert fires == 1

    def test_no_pulse_without_crossing(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        for _ in range(10):
            img = _frame([self._followed()])   # followed line only, no crossing
            ev = _run(det, img, "x", cfg, intr)
            fires += int(ev.detected)
        assert fires == 0

    def test_center_jitter_does_not_double_fire(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        seq = [40, 120, 200]                    # approach to center -> first fire
        seq += [190, 210, 200, 205, 195, 200]   # jitter inside the enter band
        for v in seq:
            img = _frame([self._followed(), (0, v, W, v)])
            ev = _run(det, img, "x", cfg, intr)
            fires += int(ev.detected)
        assert fires == 1

    def test_rearm_requires_leaving_exit_band(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        # fire at center, jitter (no re-arm), then leave past exit and return.
        seq = [40, 200, 205, 195, 360, 200]
        for v in seq:
            img = _frame([self._followed(), (0, v, W, v)])
            ev = _run(det, img, "x", cfg, intr)
            fires += int(ev.detected)
        assert fires == 2

    def test_two_crossings_two_pulses(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        vs = list(range(20, H - 20, 12)) + list(range(20, H - 20, 12))
        for v in vs:
            img = _frame([self._followed(), (0, v, W, v)])
            ev = _run(det, img, "x", cfg, intr)
            fires += int(ev.detected)
        assert fires == 2


class TestPulseYAxis:
    def _followed(self):
        return (0, int(CY), W, int(CY))     # horizontal followed line

    def test_exactly_one_pulse_per_pass(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        # crossing vertical line sweeps left -> right across the center
        for u in range(20, W - 20, 12):
            img = _frame([self._followed(), (u, 0, u, H)])
            ev = _run(det, img, "y", cfg, intr)
            fires += int(ev.detected)
        assert fires == 1

    def test_no_pulse_without_crossing(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        for _ in range(10):
            img = _frame([self._followed()])
            ev = _run(det, img, "y", cfg, intr)
            fires += int(ev.detected)
        assert fires == 0

    def test_rearm_requires_leaving_exit_band(self, cfg, intr):
        det = IntersectionDetector()
        fires = 0
        seq = [40, 320, 325, 315, 560, 320]     # fire, jitter, leave, return
        for u in seq:
            img = _frame([self._followed(), (u, 0, u, H)])
            ev = _run(det, img, "y", cfg, intr)
            fires += int(ev.detected)
        assert fires == 2


# ---------------------------------------------------------------------------
# Offset diagnostics
# ---------------------------------------------------------------------------

class TestOffsetDiagnostic:
    def test_offset_sign_flips_across_center_x(self, cfg, intr):
        det = IntersectionDetector()
        above = _run(det, _frame([(0, 80, W, 80)]), "x", cfg, intr)
        det.reset()
        below = _run(det, _frame([(0, 320, W, 320)]), "x", cfg, intr)
        assert above.offset_px is not None and above.offset_px < 0   # above center
        assert below.offset_px is not None and below.offset_px > 0   # below center

    def test_offset_none_without_crossing(self, cfg, intr):
        det = IntersectionDetector()
        ev = _run(det, _frame([(int(CX), 0, int(CX), H)]), "x", cfg, intr)
        assert ev.offset_px is None
        assert ev.crossing_line is None


# ---------------------------------------------------------------------------
# Branch flags
# ---------------------------------------------------------------------------

def _fire_at_center(det, segments, travel_axis, cfg, intr):
    """One update with the crossing centered; a fresh detector fires at once."""
    ev = _run(det, _frame(segments), travel_axis, cfg, intr)
    assert ev.detected, "expected the centered crossing to fire"
    return ev


class TestBranchFlagsXAxis:
    # travel_axis 'x': followed vertical, crossing horizontal, centered at
    # (CX, CY). forward = image up, left = image -u.
    def test_full_cross(self, cfg, intr):
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(int(CX), 0, int(CX), H), (0, int(CY), W, int(CY))],
            "x", cfg, intr,
        )
        assert ev.forward and ev.backward and ev.left and ev.right

    def test_t_from_left(self, cfg, intr):
        # crossing extends only to image left (-u): left branch, no right.
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(int(CX), 0, int(CX), H), (0, int(CY), int(CX), int(CY))],
            "x", cfg, intr,
        )
        assert ev.left and not ev.right
        assert ev.forward and ev.backward

    def test_t_from_right(self, cfg, intr):
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(int(CX), 0, int(CX), H), (int(CX), int(CY), W, int(CY))],
            "x", cfg, intr,
        )
        assert ev.right and not ev.left
        assert ev.forward and ev.backward

    def test_corner_forward_right(self, cfg, intr):
        # followed extends only up (forward), crossing only to the right: an
        # L-corner -> forward + right, no backward, no left.
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(int(CX), 0, int(CX), int(CY)), (int(CX), int(CY), W, int(CY))],
            "x", cfg, intr,
        )
        assert ev.forward and ev.right
        assert not ev.backward and not ev.left


class TestBranchFlagsYAxis:
    # travel_axis 'y': followed horizontal, crossing vertical. forward = image
    # left (-u), left = image down (+v).
    def test_full_cross(self, cfg, intr):
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(0, int(CY), W, int(CY)), (int(CX), 0, int(CX), H)],
            "y", cfg, intr,
        )
        assert ev.forward and ev.backward and ev.left and ev.right

    def test_t_down_is_left(self, cfg, intr):
        # crossing extends only image-down (+v) -> body-left for +y travel.
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(0, int(CY), W, int(CY)), (int(CX), int(CY), int(CX), H)],
            "y", cfg, intr,
        )
        assert ev.left and not ev.right
        assert ev.forward and ev.backward

    def test_t_up_is_right(self, cfg, intr):
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(0, int(CY), W, int(CY)), (int(CX), 0, int(CX), int(CY))],
            "y", cfg, intr,
        )
        assert ev.right and not ev.left
        assert ev.forward and ev.backward

    def test_corner_forward_left(self, cfg, intr):
        # followed extends only image-left (forward, +y), crossing only down
        # (body-left) -> forward + left.
        det = IntersectionDetector()
        ev = _fire_at_center(
            det,
            [(0, int(CY), int(CX), int(CY)), (int(CX), int(CY), int(CX), H)],
            "y", cfg, intr,
        )
        assert ev.forward and ev.left
        assert not ev.backward and not ev.right


# ---------------------------------------------------------------------------
# Misc
# ---------------------------------------------------------------------------

class TestMisc:
    def test_reset_re_arms(self, cfg, intr):
        det = IntersectionDetector()
        img = _frame([(int(CX), 0, int(CX), H), (0, int(CY), W, int(CY))])
        assert _run(det, img, "x", cfg, intr).detected
        assert not _run(det, img, "x", cfg, intr).detected   # still centered, disarmed
        det.reset()
        assert _run(det, img, "x", cfg, intr).detected        # re-armed

    def test_invalid_axis_raises(self, cfg, intr):
        det = IntersectionDetector()
        with pytest.raises(ValueError):
            det.update([], [], "z", intr)

    def test_short_crossing_fragment_is_ignored(self, cfg, intr):
        # a stub shorter than min_crossing_length_px near center must not fire.
        det = IntersectionDetector(IntersectionConfig(min_crossing_length_px=120.0))
        img = _frame([(int(CX), 0, int(CX), H), (300, int(CY), 340, int(CY))])
        assert not _run(det, img, "x", cfg, intr).detected


# ---------------------------------------------------------------------------
# Turns (travel-axis change)
# ---------------------------------------------------------------------------

class TestAxisChange:
    """A turn swaps the followed and crossing families. The line the drone is
    parked on inherits the crossing role at offset ~0, so without a disarm it
    fires a pulse for a crossing that was never flown."""

    def test_turn_onto_the_parked_line_does_not_fire(self, cfg, intr):
        # Cruising 'x' along a line that is vertical in the image, no crossing.
        det = IntersectionDetector()
        img = _frame([(int(CX), 0, int(CX), H)])
        assert not _run(det, img, "x", cfg, intr).detected
        assert det.armed
        # Turn onto 'y': that same line is now the crossing, dead center.
        assert not _run(det, img, "y", cfg, intr).detected

    def test_leaving_the_parked_line_re_arms_then_the_next_crossing_fires(
        self, cfg, intr
    ):
        det = IntersectionDetector()
        parked = _frame([(int(CX), 0, int(CX), H)])
        _run(det, parked, "x", cfg, intr)
        _run(det, parked, "y", cfg, intr)
        # Fly off the parked line: beyond exit_px the detector re-arms.
        off = int(CX) + 150
        assert not _run(det, _frame([(off, 0, off, H)]), "y", cfg, intr).detected
        assert det.armed
        # The next real crossing fires exactly once.
        assert _run(det, parked, "y", cfg, intr).detected
        assert not _run(det, parked, "y", cfg, intr).detected

    def test_turn_taken_mid_cell_keeps_the_next_pulse(self, cfg, intr):
        # Turning with no crossing near center must not swallow the next one.
        det = IntersectionDetector()
        _run(det, _frame([(int(CX), 0, int(CX), H)]), "x", cfg, intr)
        far = int(CX) + 150
        assert not _run(det, _frame([(far, 0, far, H)]), "y", cfg, intr).detected
        assert det.armed
        assert _run(det, _frame([(int(CX), 0, int(CX), H)]), "y", cfg, intr).detected

    def test_turning_back_does_not_double_count(self, cfg, intr):
        # x -> y -> x while parked on the same intersection: no pulse either way.
        det = IntersectionDetector()
        img = _frame([(int(CX), 0, int(CX), H), (0, int(CY), W, int(CY))])
        assert _run(det, img, "x", cfg, intr).detected     # the real crossing
        assert not _run(det, img, "y", cfg, intr).detected
        assert not _run(det, img, "x", cfg, intr).detected
