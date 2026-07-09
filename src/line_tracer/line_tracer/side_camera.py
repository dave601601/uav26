"""Sideways lookahead camera perception (pure OpenCV + numpy, no rclpy).

The drone carries a second camera (OV9281 + 6 mm lens) whose boresight is
body +Y depressed ~26 deg below horizontal. While the serpentine sweep
traverses row y, this camera observes the intersections of row y+3 (one
cell over, on the official 3 m grid) on the unexplored side, letting the
sweep skip every other row. Detections here are NAVIGATION HINTS
("candidate at node N"), never records: the actual marker recording still
happens with the downward camera + intersection snap during
WAYPOINT_VISIT.

Pipeline per frame:
  1. detect_aruco_side()      — ArUco detection tuned for small/oblique
                                markers on the mono image.
  2. project_pixel_to_ground()— attitude-compensated ray-cast of the
                                marker center onto the z=0 ground plane.
                                NOT the downward camera's depth=altitude
                                shortcut: for an oblique ray the ground
                                distance moves ~h/sin^2(depression) per
                                radian of attitude error (~10 m/rad at
                                the near band), so live roll/pitch/yaw
                                must enter the rotation.
  3. CandidateTracker.observe()— vote the projection onto its nearest
                                grid intersection; an id becomes a
                                candidate after enough votes agree.

Frames: image u right, v down. Camera optical frame per ROS REP-103
(+X right, +Y down, +Z = view axis). Gazebo sensor frame is +X view,
+Y left, +Z up; the fixed change of basis is ``_R_SENSOR_OPTICAL``.
Mount rotation composes like the SDF pose it models:
``R_body_sensor = Rz(yaw) @ Ry(pitch)`` (extrinsic fixed-axis rpy).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, hypot, sin
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .geom import CameraIntrinsics
from .perception import ARUCO_DICTS, DEFAULT_ARUCO_DICT, ArucoDetection

if False:  # TYPE_CHECKING without the import cost at runtime
    from .grid import Grid, Node


# ---------------------------------------------------------------------------
# Rotations
# ---------------------------------------------------------------------------

def _rot_x(a: float) -> np.ndarray:
    c, s = cos(a), sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = cos(a), sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = cos(a), sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Optical (X right, Y down, Z view) -> gz sensor (X view, Y left, Z up).
# Columns are the optical axes expressed in sensor coordinates.
_R_SENSOR_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


@dataclass(frozen=True)
class MountExtrinsics:
    """Camera mount on the body, mirroring the SDF ``<pose>`` of the sensor.

    Defaults are the lookahead camera in model.sdf: boresight +Y_body
    depressed 26 deg (yaw pi/2 then pitch 0.4538, extrinsic fixed-axis),
    5 cm left / 3 cm below the body origin. 26 deg centers the VFOV band
    between the adjacent row (3 m -> depression 33.3 deg) and the next
    row (6 m -> 18.2 deg) of the official 3 m grid. The downward camera
    is the degenerate case (yaw=0, pitch=pi/2) — kept exercisable so a
    unit test can pin this rotation against the hardcoded optical->body
    map in ``line_tracer_node._publish_aruco_markers``.
    """
    yaw: float = 1.5707963267948966
    pitch: float = 0.4538
    tx: float = 0.0
    ty: float = 0.05
    tz: float = -0.03

    def rotation_body_optical(self) -> np.ndarray:
        """3x3: optical-frame vector -> body FLU vector."""
        return _rot_z(self.yaw) @ _rot_y(self.pitch) @ _R_SENSOR_OPTICAL

    def translation_body(self) -> np.ndarray:
        return np.array([self.tx, self.ty, self.tz])


def project_pixel_to_ground(
    u: float,
    v: float,
    intr: CameraIntrinsics,
    mount: MountExtrinsics,
    drone_xyz: Tuple[float, float, float],
    drone_rpy: Tuple[float, float, float],
    max_range: float = 9.0,
    min_ray_down: float = 0.02,
) -> Optional[Tuple[float, float, float]]:
    """Ray-cast pixel (u, v) onto the world ground plane z=0.

    drone_rpy is the live attitude (ENU/FLU, extrinsic xyz = yaw about
    world Z applied last) — in sim from the /odom_truth quaternion.
    Returns (x_world, y_world, slant_range_m), or None when the ray is
    too close to the horizon (|down-component| < min_ray_down: the
    ground intersection explodes laterally and one attitude jitter
    frame would vote nonsense) or the hit is beyond max_range.
    """
    ray_o = np.array([
        (u - intr.cx) / intr.fx,
        (v - intr.cy) / intr.fy,
        1.0,
    ])
    ray_o /= np.linalg.norm(ray_o)
    roll, pitch, yaw = drone_rpy
    r_wb = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
    ray_w = r_wb @ mount.rotation_body_optical() @ ray_o
    cam_w = np.asarray(drone_xyz, dtype=float) + r_wb @ mount.translation_body()
    if ray_w[2] >= -min_ray_down:
        return None
    s = -cam_w[2] / ray_w[2]
    if s <= 0.0 or s > max_range:
        return None
    hit = cam_w + s * ray_w
    return (float(hit[0]), float(hit[1]), float(s))


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SideCameraConfig:
    """ArUco detection tuned for the oblique side view.

    Markers appear foreshortened (vertical extent ~ sin(depression)) and
    small: the +3 m row at 640x400 / f=1000 is ~61 px tall (~10.2 px per
    module — the code now fills the 0.4 m sheet edge to edge, so the
    module pitch is 0.4/6 = 6.67 cm, a third coarser than the old
    1-module-margin texture and correspondingly easier to read far off).
    Deviations from OpenCV defaults:
      - adaptiveThreshWinSizeStep 10 -> 4: more threshold scales so the
        thin foreshortened quad survives binarization.
      - minMarkerPerimeterRate 0.03 -> 0.02: keep small far quads.
      - polygonalApproxAccuracyRate 0.03 -> 0.05: the trapezoid corners
        deviate more from a square's polygon fit.
      - cornerRefinementMethod SUBPIX: center accuracy feeds the ground
        projection (1 px ~= 6 cm at the near band).
      - errorCorrectionRate 0.6 -> 0.8: bits sampled at ~10 px/module
        flip more easily; the vote threshold filters residual misreads.
        Note DICT_4X4_50 has maxCorrectionBits=1, so int(1*0.8)=0 bits
        are actually corrected — the rate buys nothing here and every
        accepted quad is an exact codeword match.
    """
    aruco_dict: int = ARUCO_DICTS[DEFAULT_ARUCO_DICT]
    # Same physical sheet the downward camera sees: a standard ArUco
    # (black field, white cells), so no negation. See
    # perception.PerceptionConfig.aruco_white_on_black.
    aruco_white_on_black: bool = False
    adaptive_thresh_win_min: int = 3
    adaptive_thresh_win_max: int = 23
    adaptive_thresh_win_step: int = 4
    min_marker_perimeter_rate: float = 0.02
    polygonal_approx_accuracy_rate: float = 0.05
    perspective_remove_pixel_per_cell: int = 8
    error_correction_rate: float = 0.8
    # IPM (rectify to a synthetic nadir view before detecting) is not
    # needed while only the +4 m band is load-bearing; revisit if a
    # longer-range band comes back (full-res sensor or higher altitude).
    use_ipm: bool = False


def _detector_params(cfg: SideCameraConfig):
    params = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "DetectorParameters")
        else cv2.aruco.DetectorParameters_create()
    )
    params.adaptiveThreshWinSizeMin = cfg.adaptive_thresh_win_min
    params.adaptiveThreshWinSizeMax = cfg.adaptive_thresh_win_max
    params.adaptiveThreshWinSizeStep = cfg.adaptive_thresh_win_step
    params.minMarkerPerimeterRate = cfg.min_marker_perimeter_rate
    params.polygonalApproxAccuracyRate = cfg.polygonal_approx_accuracy_rate
    params.perspectiveRemovePixelPerCell = cfg.perspective_remove_pixel_per_cell
    params.errorCorrectionRate = cfg.error_correction_rate
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_aruco_side(
    image: np.ndarray, cfg: SideCameraConfig
) -> List[ArucoDetection]:
    """ArUco detection on the (mono or BGR) side-camera frame."""
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    if cfg.aruco_white_on_black:
        gray = 255 - gray
    aruco_dict = cv2.aruco.getPredefinedDictionary(cfg.aruco_dict)
    params = _detector_params(cfg)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(
            aruco_dict, params
        ).detectMarkers(gray)
    else:                     # legacy API
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=params
        )
    out: List[ArucoDetection] = []
    if ids is None:
        return out
    for i, corner_set in zip(ids.flatten(), corners):
        pts = corner_set.reshape(-1, 2)
        out.append(
            ArucoDetection(
                id=int(i),
                center_uv=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
                corners_uv=tuple((float(p[0]), float(p[1])) for p in pts),
            )
        )
    return out


# ---------------------------------------------------------------------------
# Candidate tracking
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Candidate:
    """A marker id whose position is believed (not yet recorded)."""
    node: "Node"
    xy: Tuple[float, float]
    votes: int
    last_seen: float
    best_range: float


class CandidateTracker:
    """Vote per (id, nearest-intersection) pair; majority node wins.

    Deliberately dumb: no knowledge of records or dropped candidates —
    all mission-level filtering (already recorded / given up) lives in
    the FSM, which owns that state. The tracker only answers "where do
    repeated sightings of id K agree K sits?". Voting on the NODE (not
    raw xy) absorbs projection error: anything within snap_max_err of
    the true intersection votes the same way, mirroring the downward
    camera's snap_to_intersection tolerance.
    """

    def __init__(self, snap_max_err: float = 2.0) -> None:
        self.snap_max_err = snap_max_err
        # id -> node -> [votes, last_seen]
        self._votes: Dict[int, Dict["Node", List[float]]] = {}
        self._best_range: Dict[int, float] = {}

    def observe(
        self,
        marker_id: int,
        x_world: float,
        y_world: float,
        slant_range: float,
        t: float,
        grid: "Grid",
    ) -> Optional["Node"]:
        """Register one projected sighting. Returns the voted node, or
        None when the projection is too far from any intersection to
        trust (markers only sit on intersections, so a mid-cell hit
        means the projection itself is off)."""
        node = grid.nearest_node(x_world, y_world)
        nx, ny = grid.world(node)
        if hypot(nx - x_world, ny - y_world) > self.snap_max_err:
            return None
        per_id = self._votes.setdefault(marker_id, {})
        entry = per_id.setdefault(node, [0, t])
        entry[0] += 1
        entry[1] = t
        prev = self._best_range.get(marker_id)
        if prev is None or slant_range < prev:
            self._best_range[marker_id] = slant_range
        return node

    def snapshot(self, min_votes: int, grid: "Grid") -> Dict[int, Candidate]:
        """Candidates whose winning node reached ``min_votes``. The
        winning node is the most-voted (ties: most recently seen)."""
        out: Dict[int, Candidate] = {}
        for marker_id, per_node in self._votes.items():
            node, (votes, last_seen) = max(
                per_node.items(), key=lambda kv: (kv[1][0], kv[1][1])
            )
            if votes < min_votes:
                continue
            out[marker_id] = Candidate(
                node=node,
                xy=grid.world(node),
                votes=int(votes),
                last_seen=last_seen,
                best_range=self._best_range.get(marker_id, float("inf")),
            )
        return out


# ---------------------------------------------------------------------------
# Debug overlay
# ---------------------------------------------------------------------------

def draw_lookahead_overlay(
    image: np.ndarray,
    detections: List[ArucoDetection],
    projections: Dict[int, Tuple[float, float]],
    note: str = "",
) -> np.ndarray:
    """Yellow marker boxes + id and the projected world (x, y) when the
    ground ray-cast produced one. Returns a fresh BGR copy.

    ``note`` is stamped in amber under the id list. The node uses it to
    say the detector is not running for this frame — without it, a bare
    frame published during TAKEOFF or the retrieval tour would look like
    a frame in which the side camera genuinely saw nothing.
    """
    if image.ndim == 2:
        out = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        out = image.copy()
    for det in detections:
        pts = np.array(det.corners_uv, dtype=np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 255), 2)
        cu, cv_ = int(round(det.center_uv[0])), int(round(det.center_uv[1]))
        label = f"id={det.id}"
        if det.id in projections:
            px, py = projections[det.id]
            label += f" ({px:+.1f},{py:+.1f})"
        cv2.putText(
            out, label, (cu + 5, cv_ - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA,
        )
    cv2.putText(
        out, f"lookahead aruco={[d.id for d in detections]}", (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA,
    )
    if note:
        cv2.putText(
            out, note, (10, 42),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2, cv2.LINE_AA,
        )
    return out
