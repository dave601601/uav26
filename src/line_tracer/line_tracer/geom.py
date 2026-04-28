"""Camera intrinsics + pixel <-> metric projection.

The drone carries a single downward-facing RealSense D435. For a depth value
d at a pixel offset (Δu, Δv) from the principal point, the metric offset on
the imaged plane (in the camera's own frame) is:

    Δx_cam = Δu * d / fx
    Δy_cam = Δv * d / fy

Mapping camera-frame (Δx_cam, Δy_cam) to body-frame (forward, left) for
control commands is left to the dead-reckoning / perception layers, which
own the camera-to-body rotation.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Tuple


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, msg) -> "CameraIntrinsics":
        # sensor_msgs/CameraInfo.k is a flat 3x3 row-major matrix.
        k = msg.k
        return cls(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )

    def principal_point(self) -> Tuple[float, float]:
        return self.cx, self.cy


def pixel_offset_to_meters(
    du: float,
    dv: float,
    depth: float,
    intr: CameraIntrinsics,
) -> Tuple[float, float]:
    """Convert a pixel-space offset (Δu, Δv) at distance `depth` to meters.

    Returns (Δx_cam, Δy_cam) on the plane perpendicular to the optical axis.
    Raises ValueError on non-finite or non-positive depth, or non-positive
    focal length.
    """
    if not isfinite(depth) or depth <= 0.0:
        raise ValueError(f"depth must be finite and > 0, got {depth!r}")
    if intr.fx <= 0.0 or intr.fy <= 0.0:
        raise ValueError(f"focal lengths must be > 0, got fx={intr.fx} fy={intr.fy}")
    return du * depth / intr.fx, dv * depth / intr.fy


def pixel_to_principal_offset(
    u: float, v: float, intr: CameraIntrinsics
) -> Tuple[float, float]:
    """Return (u - cx, v - cy)."""
    return u - intr.cx, v - intr.cy
