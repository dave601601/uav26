"""Unit tests for line_tracer.geom."""
from math import isclose
from types import SimpleNamespace

import pytest

from line_tracer.geom import (
    CameraIntrinsics,
    pixel_offset_to_meters,
    pixel_to_principal_offset,
)


@pytest.fixture
def intr_d435():
    # Representative D435 color intrinsics @ 640x480.
    return CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0,
                            width=640, height=480)


def test_zero_offset_yields_zero_meters(intr_d435):
    dx, dy = pixel_offset_to_meters(0.0, 0.0, depth=2.0, intr=intr_d435)
    assert dx == 0.0 and dy == 0.0


def test_md_formula_matches(intr_d435):
    # md spec: dx = du * d / fx
    dx, dy = pixel_offset_to_meters(100.0, -50.0, depth=2.0, intr=intr_d435)
    assert isclose(dx, 100.0 * 2.0 / 600.0)
    assert isclose(dy, -50.0 * 2.0 / 600.0)


def test_anisotropic_focal(intr_d435):
    intr = CameraIntrinsics(fx=400.0, fy=800.0, cx=320.0, cy=240.0,
                            width=640, height=480)
    dx, dy = pixel_offset_to_meters(40.0, 80.0, depth=1.0, intr=intr)
    assert isclose(dx, 0.1) and isclose(dy, 0.1)


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf")])
def test_invalid_depth_raises(intr_d435, bad):
    with pytest.raises(ValueError):
        pixel_offset_to_meters(10.0, 10.0, depth=bad, intr=intr_d435)


def test_zero_focal_raises():
    intr = CameraIntrinsics(fx=0.0, fy=600.0, cx=320.0, cy=240.0,
                            width=640, height=480)
    with pytest.raises(ValueError):
        pixel_offset_to_meters(10.0, 10.0, depth=2.0, intr=intr)


def test_pixel_to_principal_offset(intr_d435):
    du, dv = pixel_to_principal_offset(420.0, 200.0, intr_d435)
    assert du == 100.0 and dv == -40.0


def test_from_camera_info():
    # Mimic sensor_msgs/CameraInfo (only the fields we use).
    msg = SimpleNamespace(
        k=[600.0, 0.0, 320.0,
           0.0, 600.0, 240.0,
           0.0, 0.0, 1.0],
        width=640,
        height=480,
    )
    intr = CameraIntrinsics.from_camera_info(msg)
    assert intr.fx == 600.0 and intr.fy == 600.0
    assert intr.cx == 320.0 and intr.cy == 240.0
    assert intr.width == 640 and intr.height == 480
