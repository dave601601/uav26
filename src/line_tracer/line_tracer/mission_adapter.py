"""Pixel-space perception -> body-frame metric conversions for the skeleton
mission backend. Pure functions (no rclpy, no OpenCV): the line_tracer node
calls these to turn a perception.PerceptionResult into the metric values that
MissionManager's PerceptionData / McuCommand expect.

Frame conventions are the ones documented in perception.py (downward camera,
image u right / v down, body forward +x_body projects to image -v):

  - The nearest VERTICAL grid line gives the body +y offset line_dx = -du*alt/fx.
  - The nearest HORIZONTAL grid line gives the body +x offset line_dy = -dv*alt/fy.
    Both offsets are always computed (the [dx, dy, flag] contract); the MCU
    selects one by travel axis. Presence travels as has_vertical / has_horizontal.
  - line_angle_error is the followed line's heading vs the travel axis, and IS
    travel-axis selected: +/-x travel uses perception's psi_err (already the
    vertical line's deviation from vertical, FLU +CCW); +/-y travel uses the
    horizontal line's own angle folded to (-pi/2, pi/2] (0 when the line is
    exactly horizontal in the image).
  - Marker center errors: body +x = -(v-cy)*alt/fy, body +y = -(u-cx)*alt/fx;
    the detection nearest the image center wins.

Line confidence has no model yet: the caller sets 1.0 when the followed line
for the current axis is present, 0.0 otherwise.
"""
from __future__ import annotations

from math import atan2, pi
from typing import Dict, List, Optional, Sequence, Tuple

from .mission import MoveDirection, move_direction_vector


def _line_angle_0_pi(x1: float, y1: float, x2: float, y2: float) -> float:
    """Line angle mod pi in perception.py's convention (image v flipped so
    up-in-image is +y math). Returns [0, pi): horizontal is 0, vertical pi/2."""
    a = atan2(-(y2 - y1), (x2 - x1))
    if a < 0.0:
        a += pi
    if a >= pi:
        a -= pi
    return a


def line_offsets_m(
    du: Optional[float],
    dv: Optional[float],
    altitude: float,
    fx: float,
    fy: float,
) -> Tuple[float, bool, float, bool]:
    """Both grid-line offsets in body meters, with presence.

    Returns (dx, has_vertical, dy, has_horizontal). dx = -du*alt/fx (nearest
    vertical line, body +y), dy = -dv*alt/fy (nearest horizontal line, body +x).
    A missing pixel error yields 0.0 with its presence flag False.
    """
    if du is None:
        dx, has_vertical = 0.0, False
    else:
        dx, has_vertical = -du * altitude / fx, True
    if dv is None:
        dy, has_horizontal = 0.0, False
    else:
        dy, has_horizontal = -dv * altitude / fy, True
    return dx, has_vertical, dy, has_horizontal


def line_angle_error_rad(
    travel_axis: str,
    psi_err: Optional[float],
    horizontal_line: Optional[Tuple[int, int, int, int]],
) -> Optional[float]:
    """Followed-line heading error vs travel axis, rad FLU +CCW.

    +/-x travel: perception's psi_err (may be None when no vertical line).
    +/-y travel: the horizontal line's angle folded to (-pi/2, pi/2], or None
    when no horizontal line was picked.
    """
    if travel_axis == "x":
        return psi_err
    if travel_axis == "y":
        if horizontal_line is None:
            return None
        a = _line_angle_0_pi(*horizontal_line)
        if a > pi / 2.0:
            a -= pi          # fold pi-neighborhood to a small negative error
        return a
    raise ValueError(f"travel_axis must be 'x' or 'y', got {travel_axis!r}")


def marker_center_errors_m(
    u: float,
    v: float,
    cx: float,
    cy: float,
    altitude: float,
    fx: float,
    fy: float,
) -> Tuple[float, float]:
    """Marker center pixel (u, v) -> body-frame metric offsets.

    Returns (error_x, error_y): body +x = -(v-cy)*alt/fy, body +y = -(u-cx)*alt/fx.
    """
    error_x = -(v - cy) * altitude / fy
    error_y = -(u - cx) * altitude / fx
    return error_x, error_y


def nearest_marker(
    markers: Sequence[Tuple[int, float, float]],
    cx: float,
    cy: float,
) -> Optional[Tuple[int, float, float]]:
    """Pick the (id, u, v) whose center is nearest the image center, or None."""
    best: Optional[Tuple[int, float, float]] = None
    best_d = float("inf")
    for mid, u, v in markers:
        d = (u - cx) ** 2 + (v - cy) ** 2
        if d < best_d:
            best_d = d
            best = (mid, u, v)
    return best


def select_front_hint(
    candidates: "Dict[int, object]",
    dr_xy: Optional[Tuple[float, float]],
    move_direction: MoveDirection,
    row_tolerance_m: float = 1.5,
) -> Optional[Tuple[int, object, float]]:
    """Nearest front-camera candidate AHEAD on the current row, or None.

    candidates maps marker id -> a voted candidate carrying .node (grid node)
    and .xy (world meters). dr_xy is the drone world position; move_direction is
    the travel axis. A candidate qualifies when it is ahead of the drone along
    the travel direction (positive along-track distance) and within
    row_tolerance_m laterally of the current row line. The nearest qualifier
    (smallest along-track distance) wins and its along-track component is the
    returned distance. Recorded-id filtering is the mission layer's job; the
    selector only orders by distance.
    """
    if not candidates or dr_xy is None:
        return None
    ux, uy = move_direction_vector(move_direction)   # unit travel vector
    x0, y0 = float(dr_xy[0]), float(dr_xy[1])
    best: Optional[Tuple[int, object, float]] = None
    for marker_id, cand in candidates.items():
        cx, cy = cand.xy
        rx, ry = cx - x0, cy - y0
        along = rx * ux + ry * uy               # + = ahead along travel
        lateral = abs(-rx * uy + ry * ux)       # perpendicular distance to row
        if along <= 0.0 or lateral > row_tolerance_m:
            continue
        if best is None or along < best[2]:
            best = (marker_id, cand.node, along)
    return best
