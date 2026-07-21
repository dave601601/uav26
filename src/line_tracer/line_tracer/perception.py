"""Grid-line + ArUco perception (pure OpenCV, no rclpy).

Inputs are raw numpy BGR / depth frames + a `CameraIntrinsics` (see
`line_tracer.geom`). Outputs are pixel-space errors `(du, dv, psi_err)` and
a list of ArUco detections. Conversion to body-frame metric offsets lives
in the node layer (it owns the camera->body rotation and altitude estimate).

Frame conventions (camera mounted bottom of drone, optical +Z = -Z_body):
  image u right, v down. Body forward (+x_body) projects to image -v.
  - du > 0  ⇒ line is to the right of image center
              (= drone is to the left of the line, must move -y_body)
  - dv > 0  ⇒ line is below image center
              (= drone is past the line, must move -x_body to return)
  - psi_err > 0  ⇒ apply +wz (CCW around +z_body) to align body forward
                   with the line direction

These signs match `line_tracer.dead_reckoning.compute_body_velocity` once
the node maps:  dx_body = -dv * d / fy,  dy_body = -du * d / fx,
                psi_err passes through.

ArUco false-positive hazard: OpenCV builds candidate quads only out of
dark regions, so on a grass field a patch bounded by white grid lines can
qualify as a quad and decode as an exact codeword — a real false positive.
The mitigation belongs in the record path (multi-frame voting plus a
marker-size check): the downward camera currently commits a record from a
single frame with no vote and no size check.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, pi, sin
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .geom import CameraIntrinsics


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

# Param string -> cv2 constant. The rules give marker IDs 0..49 without
# naming a dictionary; 4X4_50 is the working default until they confirm.
ARUCO_DICTS = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "7X7_50": cv2.aruco.DICT_7X7_50,
}
DEFAULT_ARUCO_DICT = "4X4_50"


def resolve_aruco_dict(name: str) -> int:
    try:
        return ARUCO_DICTS[name.strip().upper()]
    except KeyError as exc:
        raise ValueError(
            f"unknown aruco dictionary {name!r}; known: {sorted(ARUCO_DICTS)}"
        ) from exc


@dataclass(frozen=True)
class PerceptionConfig:
    canny_low: int = 60
    canny_high: int = 180
    hough_rho: float = 1.0
    hough_theta: float = pi / 180.0
    hough_threshold: int = 60
    hough_min_line_length: int = 40
    hough_max_line_gap: int = 20
    # Angle-band half-widths (rad): a line whose angle in [0, pi) is within
    # this of pi/2 counts as vertical; within this of 0 or pi, horizontal.
    vertical_half_width: float = pi / 6.0     # 30°
    horizontal_half_width: float = pi / 6.0   # 30°
    aruco_dict: int = ARUCO_DICTS[DEFAULT_ARUCO_DICT]
    # False: the rules' marker is a standard ArUco (black field, white
    # cells). Set True only for a genuinely inverted marker; prefer this over
    # detectInvertedMarker, whose both-polarity mode doubles false accepts.
    aruco_white_on_black: bool = False


@dataclass(frozen=True)
class ArucoDetection:
    id: int
    center_uv: Tuple[float, float]
    corners_uv: Tuple[Tuple[float, float], ...]   # 4 corners, image order


@dataclass
class PerceptionResult:
    du: Optional[float] = None        # pixels, signed; None ⇒ no vertical line
    dv: Optional[float] = None        # pixels, signed; None ⇒ no horizontal line
    psi_err: Optional[float] = None   # rad,    signed; None ⇒ no vertical line
    aruco: List[ArucoDetection] = field(default_factory=list)
    # Diagnostics (used by debug renderer; node may ignore):
    vertical_line: Optional[Tuple[int, int, int, int]] = None
    horizontal_line: Optional[Tuple[int, int, int, int]] = None
    all_lines: List[Tuple[int, int, int, int]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Line geometry helpers
# ---------------------------------------------------------------------------

def _line_angle_in_0_pi(dx: float, dy: float) -> float:
    """Angle of a line (mod π) in 'math' convention (image v flipped).

    Returned angle ∈ [0, π).  A perfectly vertical line (along image -v
    direction) is π/2.  A perfectly horizontal line is 0.
    """
    a = atan2(-dy, dx)        # flip v so up-in-image is +y math
    if a < 0:
        a += pi               # collapse to [0, π)
    if a >= pi:
        a -= pi
    return a


def _canonical_direction(dx: float, dy: float) -> Tuple[float, float]:
    """Unit direction vector with the 'up-in-image' (dy ≤ 0) convention."""
    n = hypot(dx, dy)
    if n == 0.0:
        return 1.0, 0.0
    dx_n, dy_n = dx / n, dy / n
    if dy_n > 0:                 # flip so direction points up in image
        dx_n, dy_n = -dx_n, -dy_n
    return dx_n, dy_n


def _signed_perp_du(line: Tuple[int, int, int, int], cx: float, cy: float) -> float:
    """Signed perpendicular pixel distance to a (canonicalized) vertical line.

    Sign convention: positive when the line lies to the right of the image
    center (i.e. line_u > cx).
    """
    x1, y1, x2, y2 = line
    dx_n, dy_n = _canonical_direction(x2 - x1, y2 - y1)
    return (cx - x1) * dy_n - (cy - y1) * dx_n


def _signed_perp_dv(line: Tuple[int, int, int, int], cx: float, cy: float) -> float:
    """Signed perpendicular pixel distance to a (canonicalized) horizontal line.

    Sign convention: positive when the line lies below the image center
    (line_v > cy ≡ behind the drone, since +x_body projects to image -v).
    """
    x1, y1, x2, y2 = line
    n = hypot(x2 - x1, y2 - y1)
    if n == 0.0:
        return 0.0
    # canonical horizontal direction: dx ≥ 0
    dx_n, dy_n = (x2 - x1) / n, (y2 - y1) / n
    if dx_n < 0:
        dx_n, dy_n = -dx_n, -dy_n
    return (y1 - cy) * dx_n + (cx - x1) * dy_n


def _abs_perp_distance(line: Tuple[int, int, int, int], cx: float, cy: float) -> float:
    x1, y1, x2, y2 = line
    n = hypot(x2 - x1, y2 - y1)
    if n == 0.0:
        return float("inf")
    # |((cx-x1)*dy - (cy-y1)*dx) / n|
    return abs((cx - x1) * (y2 - y1) - (cy - y1) * (x2 - x1)) / n


# ---------------------------------------------------------------------------
# Top-level perception
# ---------------------------------------------------------------------------

def detect_lines(
    image_bgr_or_gray: np.ndarray, cfg: PerceptionConfig
) -> List[Tuple[int, int, int, int]]:
    """Canny + Probabilistic Hough; returns list of (x1,y1,x2,y2)."""
    if image_bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(image_bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = image_bgr_or_gray
    edges = cv2.Canny(gray, cfg.canny_low, cfg.canny_high)
    raw = cv2.HoughLinesP(
        edges,
        rho=cfg.hough_rho,
        theta=cfg.hough_theta,
        threshold=cfg.hough_threshold,
        minLineLength=cfg.hough_min_line_length,
        maxLineGap=cfg.hough_max_line_gap,
    )
    if raw is None:
        return []
    return [tuple(int(v) for v in line[0]) for line in raw]


def classify_lines(
    lines: Sequence[Tuple[int, int, int, int]], cfg: PerceptionConfig
) -> Tuple[List[Tuple[int, int, int, int]], List[Tuple[int, int, int, int]]]:
    """Partition lines into (vertical-ish, horizontal-ish)."""
    vert: List[Tuple[int, int, int, int]] = []
    horiz: List[Tuple[int, int, int, int]] = []
    for ln in lines:
        a = _line_angle_in_0_pi(ln[2] - ln[0], ln[3] - ln[1])
        # distance to π/2 (vertical) and to {0, π} (horizontal)
        d_vert = abs(a - pi / 2.0)
        d_horiz = min(a, pi - a)
        if d_vert <= cfg.vertical_half_width:
            vert.append(ln)
        elif d_horiz <= cfg.horizontal_half_width:
            horiz.append(ln)
    return vert, horiz


def pick_nearest_line(
    lines: Iterable[Tuple[int, int, int, int]], cx: float, cy: float
) -> Optional[Tuple[int, int, int, int]]:
    best: Optional[Tuple[int, int, int, int]] = None
    best_d = float("inf")
    for ln in lines:
        d = _abs_perp_distance(ln, cx, cy)
        if d < best_d:
            best_d = d
            best = ln
    return best


def compute_pixel_errors(
    vertical_line: Optional[Tuple[int, int, int, int]],
    horizontal_line: Optional[Tuple[int, int, int, int]],
    intr: CameraIntrinsics,
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """Returns (du, dv, psi_err) any of which may be None when missing."""
    cx, cy = intr.principal_point()
    du: Optional[float] = None
    psi_err: Optional[float] = None
    if vertical_line is not None:
        du = _signed_perp_du(vertical_line, cx, cy)
        a = _line_angle_in_0_pi(
            vertical_line[2] - vertical_line[0],
            vertical_line[3] - vertical_line[1],
        )
        psi_err = a - pi / 2.0   # 0 when line is vertical (body-fwd aligned)
    dv: Optional[float] = None
    if horizontal_line is not None:
        dv = _signed_perp_dv(horizontal_line, cx, cy)
    return du, dv, psi_err


# ---------------------------------------------------------------------------
# ArUco
# ---------------------------------------------------------------------------

def _make_aruco_detector(cfg: PerceptionConfig):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cfg.aruco_dict)
    # Newer cv2 (>=4.7) ships ArucoDetector; fall back to legacy detectMarkers.
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, params)
    return None


def detect_aruco(
    image_bgr_or_gray: np.ndarray, cfg: PerceptionConfig
) -> List[ArucoDetection]:
    if image_bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(image_bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = image_bgr_or_gray
    if cfg.aruco_white_on_black:
        gray = 255 - gray
    det = _make_aruco_detector(cfg)
    if det is not None:
        corners, ids, _ = det.detectMarkers(gray)
    else:                     # legacy API
        aruco_dict = cv2.aruco.getPredefinedDictionary(cfg.aruco_dict)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    out: List[ArucoDetection] = []
    if ids is None:
        return out
    for i, corner_set in zip(ids.flatten(), corners):
        pts = corner_set.reshape(-1, 2)
        cu = float(pts[:, 0].mean())
        cv_ = float(pts[:, 1].mean())
        out.append(
            ArucoDetection(
                id=int(i),
                center_uv=(cu, cv_),
                corners_uv=tuple((float(p[0]), float(p[1])) for p in pts),
            )
        )
    return out


# ---------------------------------------------------------------------------
# Top-level entry
# ---------------------------------------------------------------------------

def process_image(
    image_bgr: np.ndarray, intr: CameraIntrinsics, cfg: PerceptionConfig
) -> PerceptionResult:
    """Run grid-line + ArUco detection and return packaged result."""
    cx, cy = intr.principal_point()
    lines = detect_lines(image_bgr, cfg)
    vert, horiz = classify_lines(lines, cfg)
    v_line = pick_nearest_line(vert, cx, cy)
    h_line = pick_nearest_line(horiz, cx, cy)
    du, dv, psi_err = compute_pixel_errors(v_line, h_line, intr)
    aruco_dets = detect_aruco(image_bgr, cfg)
    return PerceptionResult(
        du=du,
        dv=dv,
        psi_err=psi_err,
        aruco=aruco_dets,
        vertical_line=v_line,
        horizontal_line=h_line,
        all_lines=lines,
    )


# ---------------------------------------------------------------------------
# Debug overlay
# ---------------------------------------------------------------------------

def draw_debug_overlay(
    image_bgr: np.ndarray, intr: CameraIntrinsics, result: PerceptionResult
) -> np.ndarray:
    """Annotate image: all lines (gray), picked vertical (green) / horizontal
    (cyan), ArUco markers (yellow boxes + IDs), image center cross + the
    measured du/dv as colored offsets.

    Caller owns the lifecycle of `image_bgr`; we return a fresh copy.
    """
    out = image_bgr.copy()
    cx, cy = int(round(intr.cx)), int(round(intr.cy))

    for x1, y1, x2, y2 in result.all_lines:
        cv2.line(out, (x1, y1), (x2, y2), (120, 120, 120), 1)
    if result.vertical_line is not None:
        x1, y1, x2, y2 = result.vertical_line
        cv2.line(out, (x1, y1), (x2, y2), (0, 220, 0), 2)
    if result.horizontal_line is not None:
        x1, y1, x2, y2 = result.horizontal_line
        cv2.line(out, (x1, y1), (x2, y2), (220, 220, 0), 2)

    cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    if result.du is not None:
        cv2.line(out, (cx, cy), (cx + int(result.du), cy), (0, 0, 255), 2)
    if result.dv is not None:
        cv2.line(out, (cx, cy), (cx, cy + int(result.dv)), (0, 128, 255), 2)

    for det in result.aruco:
        pts = np.array(det.corners_uv, dtype=np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 255), 2)
        cu, cv_ = int(round(det.center_uv[0])), int(round(det.center_uv[1]))
        cv2.putText(
            out, f"id={det.id}", (cu + 5, cv_ - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
        )

    txt: List[str] = []
    if result.du is not None and result.psi_err is not None:
        txt.append(f"du={result.du:+.1f}px psi_err={result.psi_err:+.3f}rad")
    if result.dv is not None:
        txt.append(f"dv={result.dv:+.1f}px")
    txt.append(f"aruco={[d.id for d in result.aruco]}")
    for i, line in enumerate(txt):
        cv2.putText(
            out, line, (10, 20 + 22 * i),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            out, line, (10, 20 + 22 * i),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA,
        )
    return out


# ---------------------------------------------------------------------------
# Intersection (grid-crossing) detection
# ---------------------------------------------------------------------------
#
# The drone follows one white grid line with yaw locked, so the followed line
# and the crossing line have fixed image orientations that depend only on the
# body axis of travel:
#   travel_axis 'x' (moving +/-x_body): followed line is vertical in the image,
#                 crossing lines are horizontal.
#   travel_axis 'y' (sideways strafe, moving +/-y_body): followed line is
#                 horizontal, crossing lines are vertical.
# The caller passes the already-classified Hough sets (lines_vertical,
# lines_horizontal) from `classify_lines`, so this stage owns only the pulse
# state machine and the branch geometry, not the Canny/Hough tuning.


def _seg_length(line: Tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = line
    return hypot(x2 - x1, y2 - y1)


def _line_intersection(
    l1: Tuple[int, int, int, int], l2: Tuple[int, int, int, int]
) -> Optional[Tuple[float, float]]:
    """Intersection of two infinite lines through the given segments.

    Returns None when the lines are (near) parallel, which for a followed
    vs crossing pair means one of them was misclassified.
    """
    x1, y1, x2, y2 = l1
    x3, y3, x4, y4 = l2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-9:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    px = (a * (x3 - x4) - (x1 - x2) * b) / denom
    py = (a * (y3 - y4) - (y1 - y2) * b) / denom
    return px, py


@dataclass(frozen=True)
class IntersectionConfig:
    """Pixel thresholds for the crossing pulse, tuned for a 640x400 downward
    frame at ~2 m altitude with fx~fy in 500-1000 and 3 m grid cells. At that
    scale one pixel is roughly alt/f = 2-4 mm on the ground and the 3 m cell
    spacing (>=750 px) means only one crossing line is ever near the center,
    so a simple nearest-line + one-shot band works.
    """

    # Crossing must reach within this of the image center (|offset| < enter_px)
    # to fire: ~8-16 cm half-band, a few white-line widths, wide enough that at
    # cruise <=1 m/s and >=15 Hz the crossing lands inside it on several frames.
    enter_px: float = 40.0
    # Crossing must then pass beyond this (|offset| > exit_px) before the next
    # pulse is allowed: ~18-36 cm, comfortably past the line and its Hough
    # fragments so centroid jitter cannot re-arm mid-junction. Must exceed enter_px.
    exit_px: float = 90.0
    # An endpoint within this of the junction counts as not extending, so a
    # T-stem that merely touches the bar is not read as crossing through.
    branch_margin_px: float = 15.0
    # Ignore crossing candidates shorter than this: a real grid crossing spans
    # much of the frame, so this rejects stray Hough fragments near the center.
    min_crossing_length_px: float = 40.0

    def __post_init__(self) -> None:
        if not self.exit_px > self.enter_px:
            raise ValueError(
                f"exit_px ({self.exit_px}) must exceed enter_px ({self.enter_px})"
            )


@dataclass
class IntersectionEvent:
    """Result of one `IntersectionDetector.update` call.

    `detected` is the pulse: true for exactly one frame per physical crossing
    (see the class docstring). `forward/backward/left/right` are meaningful only
    on the firing frame and are relative to the travel direction (see
    `IntersectionDetector`). The rest are diagnostics for the debug overlay.
    """

    detected: bool = False
    forward: bool = False
    left: bool = False
    right: bool = False
    backward: bool = False
    # Diagnostics (debug overlay / logging; consumer may ignore):
    crossing_line: Optional[Tuple[int, int, int, int]] = None
    offset_px: Optional[float] = None   # signed offset of the crossing from center
    armed: bool = True                  # detector state after this update


class IntersectionDetector:
    """Stateful pulse detector for grid crossings (hysteresis across frames).

    Feed it the classified Hough sets each frame; it emits `detected=True` for
    exactly one frame per physical crossing. A crossing (the grid line
    perpendicular to travel) fires when its signed offset from the image center
    enters the band |offset| < enter_px while the detector is armed, and the
    detector only re-arms after seeing the crossing leave the wider band
    |offset| > exit_px. It does not re-arm merely because the crossing dropped
    out of view, so a one-frame Hough miss right after firing cannot double-count.

    Branch flags are labeled relative to the positive body axis being forward:
    for travel_axis 'x', forward is +x_body (image up) and left is +y_body
    (image left); for 'y', forward is +y_body (image left) and left is -x_body
    (image down). Yaw is locked, so this image-to-body mapping is fixed. A
    caller moving in the negative direction (X_NEG / Y_NEG) swaps forward with
    backward and left with right; the detector cannot recover travel sign from
    vision alone, so it reports the +axis convention and leaves that flip to the
    mission layer, which knows the commanded MoveDirection.
    """

    def __init__(self, cfg: Optional[IntersectionConfig] = None) -> None:
        self.cfg = cfg or IntersectionConfig()
        self._armed = True
        self._last_axis: Optional[str] = None

    @property
    def armed(self) -> bool:
        return self._armed

    def reset(self) -> None:
        """Re-arm and forget history."""
        self._armed = True
        self._last_axis = None

    def update(
        self,
        lines_vertical: Sequence[Tuple[int, int, int, int]],
        lines_horizontal: Sequence[Tuple[int, int, int, int]],
        travel_axis: str,
        intr: CameraIntrinsics,
    ) -> IntersectionEvent:
        cx, cy = intr.principal_point()
        if travel_axis == "x":
            followed_lines, crossing_lines = lines_vertical, lines_horizontal
        elif travel_axis == "y":
            followed_lines, crossing_lines = lines_horizontal, lines_vertical
        else:
            raise ValueError(f"travel_axis must be 'x' or 'y', got {travel_axis!r}")

        # A turn swaps which family counts as the crossing, handing the role to
        # the line the drone is parked on. Disarm so it must leave the exit band
        # first; a turn taken mid-cell re-arms again below on the same frame.
        if self._last_axis is not None and travel_axis != self._last_axis:
            self._armed = False
        self._last_axis = travel_axis

        crossing = self._pick_crossing(crossing_lines, cx, cy)
        followed = pick_nearest_line(followed_lines, cx, cy)

        offset: Optional[float] = None
        if crossing is not None:
            if travel_axis == "x":
                offset = _signed_perp_dv(crossing, cx, cy)   # + when below center
            else:
                offset = _signed_perp_du(crossing, cx, cy)   # + when right of center

        # Re-arm only on a crossing seen beyond the exit band, never on a mere
        # dropout, so a flicker right after firing cannot re-trigger.
        if offset is not None and abs(offset) > self.cfg.exit_px:
            self._armed = True

        detected = False
        fwd = bwd = left = right = False
        if self._armed and offset is not None and abs(offset) < self.cfg.enter_px:
            detected = True
            self._armed = False
            fwd, bwd, left, right = self._branch_flags(
                crossing, followed, travel_axis, cx, cy
            )

        return IntersectionEvent(
            detected=detected,
            forward=fwd,
            left=left,
            right=right,
            backward=bwd,
            crossing_line=crossing,
            offset_px=offset,
            armed=self._armed,
        )

    def _pick_crossing(
        self,
        lines: Sequence[Tuple[int, int, int, int]],
        cx: float,
        cy: float,
    ) -> Optional[Tuple[int, int, int, int]]:
        best: Optional[Tuple[int, int, int, int]] = None
        best_d = float("inf")
        for ln in lines:
            if _seg_length(ln) < self.cfg.min_crossing_length_px:
                continue
            d = _abs_perp_distance(ln, cx, cy)
            if d < best_d:
                best_d = d
                best = ln
        return best

    def _branch_flags(
        self,
        crossing: Tuple[int, int, int, int],
        followed: Optional[Tuple[int, int, int, int]],
        travel_axis: str,
        cx: float,
        cy: float,
    ) -> Tuple[bool, bool, bool, bool]:
        """Which of forward/backward/left/right have a line segment leaving the
        junction, from the Hough endpoints. See the class docstring for the sign
        convention. Returns (forward, backward, left, right).
        """
        margin = self.cfg.branch_margin_px
        junction = _line_intersection(followed, crossing) if followed is not None else None
        jx, jy = junction if junction is not None else (cx, cy)

        cxs = (crossing[0], crossing[2])
        cys = (crossing[1], crossing[3])
        fwd = bwd = left = right = False
        if travel_axis == "x":
            # Crossing is horizontal: it can reach body-left (image -u) or
            # body-right (image +u) of the junction.
            left = min(cxs) < jx - margin
            right = max(cxs) > jx + margin
            # Followed is vertical: forward is image up (-v), backward image down.
            if followed is not None:
                fys = (followed[1], followed[3])
                fwd = min(fys) < jy - margin
                bwd = max(fys) > jy + margin
        else:
            # Crossing is vertical: body-left is image down (+v), body-right up.
            left = max(cys) > jy + margin
            right = min(cys) < jy - margin
            # Followed is horizontal: forward is image left (-u), backward right.
            if followed is not None:
                fxs = (followed[0], followed[2])
                fwd = min(fxs) < jx - margin
                bwd = max(fxs) > jx + margin
        return fwd, bwd, left, right
