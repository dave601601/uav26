"""Unit tests for line_tracer.side_camera — projection, detection, tracker."""
import math
from math import isclose

import cv2
import numpy as np
import pytest

from line_tracer.geom import CameraIntrinsics
from line_tracer.grid import Grid
from line_tracer.side_camera import (
    Candidate,
    CandidateTracker,
    MountExtrinsics,
    SideCameraConfig,
    detect_aruco_side,
    draw_lookahead_overlay,
    project_pixel_to_ground,
)


# Sim lookahead camera: 640x400, HFOV 0.6196 rad -> f = 320/tan(0.3098).
F_PX = 320.0 / math.tan(0.3098)


@pytest.fixture
def intr() -> CameraIntrinsics:
    return CameraIntrinsics(fx=F_PX, fy=F_PX, cx=320.0, cy=200.0,
                            width=640, height=400)


@pytest.fixture
def mount() -> MountExtrinsics:
    return MountExtrinsics()   # the model.sdf lookahead pose


LEVEL = (0.0, 0.0, 0.0)


class TestProjection:
    def test_downward_mount_reproduces_node_optical_to_body_map(self, intr):
        """The (yaw=0, pitch=pi/2) degenerate case must reproduce the
        hardcoded downward map in line_tracer_node._publish_aruco_markers
        (xb=-yc, yb=-xc at depth=altitude) for any pixel — one rotation
        convention across both cameras."""
        down = MountExtrinsics(yaw=0.0, pitch=math.pi / 2, tx=0.0, ty=0.0, tz=0.0)
        d = 2.0
        for u, v in [(320.0, 200.0), (420.0, 260.0), (100.0, 50.0), (639.0, 399.0)]:
            hit = project_pixel_to_ground(u, v, intr, down, (0.0, 0.0, d), LEVEL)
            assert hit is not None
            xc = (u - intr.cx) * d / intr.fx
            yc = (v - intr.cy) * d / intr.fy
            assert isclose(hit[0], -yc, abs_tol=1e-9)
            assert isclose(hit[1], -xc, abs_tol=1e-9)

    def test_side_center_pixel_lateral_distance(self, intr, mount):
        """Level flight at h=2: boresight is depressed 22 deg, so the
        center pixel hits ty + (h + tz)/tan(0.384) in +Y and 0 in X."""
        hit = project_pixel_to_ground(320.0, 200.0, intr, mount, (0.0, 0.0, 2.0), LEVEL)
        assert hit is not None
        expected_y = 0.05 + (2.0 - 0.03) / math.tan(0.384)
        assert isclose(hit[0], 0.0, abs_tol=1e-9)
        assert isclose(hit[1], expected_y, abs_tol=1e-6)

    def test_band_edges_match_vfov(self, intr, mount):
        """Top/bottom image rows land at depression 22 -/+ 11.3 deg —
        the lateral 3..10.5 m band the row-skip design relies on."""
        half_vfov = math.atan(200.0 / F_PX)
        cam_h = 2.0 - 0.03
        top = project_pixel_to_ground(320.0, 0.0, intr, mount, (0.0, 0.0, 2.0), LEVEL)
        bot = project_pixel_to_ground(320.0, 400.0, intr, mount, (0.0, 0.0, 2.0), LEVEL)
        assert top is not None and bot is not None
        assert isclose(top[1], 0.05 + cam_h / math.tan(0.384 - half_vfov), rel_tol=1e-6)
        assert isclose(bot[1], 0.05 + cam_h / math.tan(0.384 + half_vfov), rel_tol=1e-6)
        # The adjacent sweep row (+4 m) must be inside the band.
        assert bot[1] < 4.0 < top[1]

    def test_roll_shifts_effective_depression(self, intr):
        """For the side camera, body roll r composes as Rx(r)*Rz(pi/2)*
        Ry(p) = Rz(pi/2)*Ry(p - r): rolling toward the camera side lifts
        the boresight exactly like a shallower mount. Pin the identity
        (translation zeroed so only rotations are in play)."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        for roll in (0.1, -0.15):
            rolled = project_pixel_to_ground(
                320.0, 200.0, intr, m0, (0.0, 0.0, 2.0), (roll, 0.0, 0.0)
            )
            shallower = project_pixel_to_ground(
                320.0, 200.0, intr,
                MountExtrinsics(pitch=0.384 - roll, tx=0.0, ty=0.0, tz=0.0),
                (0.0, 0.0, 2.0), LEVEL,
            )
            assert rolled is not None and shallower is not None
            assert isclose(rolled[0], shallower[0], abs_tol=1e-9)
            assert isclose(rolled[1], shallower[1], abs_tol=1e-9)

    def test_body_pitch_swings_hit_along_x(self, intr):
        """Body pitch p rotates the depressed side ray's -Z component
        into -X: the center hit moves to exactly x = -h*tan(p)
        (independent of the mount depression) while the lateral
        distance only stretches by 1/cos(p). This is the along-row
        smear the attitude compensation exists to absorb."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        p = 0.12
        pitched = project_pixel_to_ground(
            320.0, 200.0, intr, m0, (0.0, 0.0, 2.0), (0.0, p, 0.0)
        )
        assert pitched is not None
        assert isclose(pitched[0], -2.0 * math.tan(p), abs_tol=1e-9)
        assert isclose(
            pitched[1], 2.0 / (math.tan(0.384) * math.cos(p)), rel_tol=1e-9
        )

    def test_yaw_rotates_hit_into_world(self, intr):
        """Drone yawed 90 deg: the +Y_body boresight points at world -X."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        hit = project_pixel_to_ground(
            320.0, 200.0, intr, m0, (10.0, 5.0, 2.0), (0.0, 0.0, math.pi / 2)
        )
        assert hit is not None
        lateral = 2.0 / math.tan(0.384)
        assert isclose(hit[0], 10.0 - lateral, abs_tol=1e-6)
        assert isclose(hit[1], 5.0, abs_tol=1e-9)

    def test_near_horizon_ray_refused(self, intr):
        """Roll lifting the boresight to <~1 deg depression: the ground
        hit would be tens of metres out on attitude jitter — refuse."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        assert project_pixel_to_ground(
            320.0, 200.0, intr, m0, (0.0, 0.0, 2.0), (0.37, 0.0, 0.0)
        ) is None

    def test_max_range_gate(self, intr, mount):
        """Center-pixel slant at h=2 is ~5.3 m; a 3 m gate refuses it."""
        assert project_pixel_to_ground(
            320.0, 200.0, intr, mount, (0.0, 0.0, 2.0), LEVEL, max_range=3.0
        ) is None

    def test_translation_offset_applied(self, intr):
        """ty shifts the hit laterally; tz changes the camera height."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        m1 = MountExtrinsics(tx=0.0, ty=0.5, tz=0.0)
        h0 = project_pixel_to_ground(320.0, 200.0, intr, m0, (0.0, 0.0, 2.0), LEVEL)
        h1 = project_pixel_to_ground(320.0, 200.0, intr, m1, (0.0, 0.0, 2.0), LEVEL)
        assert h0 is not None and h1 is not None
        assert isclose(h1[1] - h0[1], 0.5, abs_tol=1e-9)

    def test_slant_range_returned(self, intr):
        """Third element is the metric slant range along the ray."""
        m0 = MountExtrinsics(tx=0.0, ty=0.0, tz=0.0)
        hit = project_pixel_to_ground(320.0, 200.0, intr, m0, (0.0, 0.0, 2.0), LEVEL)
        assert hit is not None
        assert isclose(hit[2], 2.0 / math.sin(0.384), rel_tol=1e-6)


# ---------------------------------------------------------------------------
# Detection on the synthesized oblique view
# ---------------------------------------------------------------------------

def _render_oblique_marker(
    marker_id: int,
    width_px: int = 100,
    squash: float = 0.45,
    image_size=(400, 640),
):
    """Marker as the side camera sees it at the +4 m band: ~100 px wide,
    vertically squashed by sin(26.6 deg) ~ 0.45, mild trapezoid. Returns
    (mono image, expected center uv)."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    src_px = 256
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, src_px)
    h, w = image_size
    img = np.full((h, w), 255, dtype=np.uint8)
    cx_, cy_ = w / 2.0, h / 2.0
    half_w = width_px / 2.0
    half_h = width_px * squash / 2.0
    taper = 4.0    # far edge (image top) slightly narrower — perspective
    dst = np.array(
        [
            [cx_ - half_w + taper, cy_ - half_h],   # top-left
            [cx_ + half_w - taper, cy_ - half_h],   # top-right
            [cx_ + half_w, cy_ + half_h],           # bottom-right
            [cx_ - half_w, cy_ + half_h],           # bottom-left
        ],
        dtype=np.float32,
    )
    src = np.array(
        [[0, 0], [src_px, 0], [src_px, src_px], [0, src_px]], dtype=np.float32
    )
    m = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(
        marker, m, (w, h),
        flags=cv2.INTER_AREA,
        borderMode=cv2.BORDER_CONSTANT, borderValue=255,
    )
    img = np.minimum(img, warped)
    return img, (cx_, cy_)


class TestDetectArucoSide:
    def test_detects_oblique_squashed_marker(self):
        """The +4 m band geometry (~6 px/module after foreshortening)
        must detect — this is the load-bearing case for the row skip."""
        img, center = _render_oblique_marker(marker_id=3)
        dets = detect_aruco_side(img, SideCameraConfig())
        assert len(dets) == 1
        assert dets[0].id == 3
        assert isclose(dets[0].center_uv[0], center[0], abs_tol=3.0)
        assert isclose(dets[0].center_uv[1], center[1], abs_tol=3.0)

    def test_accepts_bgr_input(self):
        img, _ = _render_oblique_marker(marker_id=1)
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        dets = detect_aruco_side(bgr, SideCameraConfig())
        assert [d.id for d in dets] == [1]

    def test_blank_image_no_detections(self):
        blank = np.full((400, 640), 255, dtype=np.uint8)
        assert detect_aruco_side(blank, SideCameraConfig()) == []

    def test_overlay_runs_on_mono(self):
        img, _ = _render_oblique_marker(marker_id=2)
        dets = detect_aruco_side(img, SideCameraConfig())
        out = draw_lookahead_overlay(img, dets, {2: (4.0, 8.0)})
        assert out.ndim == 3 and out.shape[:2] == img.shape


# ---------------------------------------------------------------------------
# Candidate tracker
# ---------------------------------------------------------------------------

@pytest.fixture
def grid() -> Grid:
    return Grid.from_extents(width=30.0, depth=20.0, cell=4.0)


class TestCandidateTracker:
    def test_below_threshold_not_promoted(self, grid):
        tr = CandidateTracker()
        tr.observe(5, 8.2, 7.9, 4.5, 0.0, grid)
        tr.observe(5, 7.8, 8.1, 4.4, 0.1, grid)
        assert tr.snapshot(3, grid) == {}

    def test_promoted_at_threshold_with_node_world_xy(self, grid):
        tr = CandidateTracker()
        for i in range(3):
            tr.observe(5, 8.0 + 0.1 * i, 8.0 - 0.1 * i, 4.5 - i * 0.1, float(i), grid)
        snap = tr.snapshot(3, grid)
        assert set(snap) == {5}
        cand = snap[5]
        assert cand.node == grid.nearest_node(8.0, 8.0)
        assert cand.xy == (8.0, 8.0)     # node world coords, not raw hits
        assert cand.votes == 3
        assert isclose(cand.best_range, 4.3, abs_tol=1e-9)
        assert cand.last_seen == 2.0

    def test_conflicting_nodes_majority_wins(self, grid):
        """A long-range misprojection votes a neighboring node a few
        times; the true node out-votes it."""
        tr = CandidateTracker()
        for i in range(3):
            tr.observe(5, 12.0, 8.0, 5.0, float(i), grid)       # node (3,2)
        for i in range(5):
            tr.observe(5, 8.0, 8.0, 4.5, 10.0 + i, grid)        # node (2,2)
        snap = tr.snapshot(3, grid)
        assert snap[5].node == grid.nearest_node(8.0, 8.0)
        assert snap[5].votes == 5

    def test_midcell_projection_refused(self, grid):
        """A hit >snap_max_err from every intersection (markers only sit
        on intersections) means the projection is wrong — no vote."""
        tr = CandidateTracker(snap_max_err=2.0)
        assert tr.observe(5, 10.0, 6.0, 5.0, 0.0, grid) is None   # 2.83 m out
        assert tr.snapshot(1, grid) == {}

    def test_multiple_ids_tracked_independently(self, grid):
        tr = CandidateTracker()
        for i in range(3):
            tr.observe(1, 4.0, 16.0, 4.5, float(i), grid)
            tr.observe(3, 24.0, 16.0, 4.6, float(i), grid)
        snap = tr.snapshot(3, grid)
        assert set(snap) == {1, 3}
        assert snap[1].xy == (4.0, 16.0)
        assert snap[3].xy == (24.0, 16.0)

    def test_candidate_is_frozen_dataclass(self, grid):
        tr = CandidateTracker()
        for i in range(3):
            tr.observe(1, 4.0, 8.0, 4.5, float(i), grid)
        cand = tr.snapshot(3, grid)[1]
        assert isinstance(cand, Candidate)
        with pytest.raises(Exception):
            cand.votes = 99
