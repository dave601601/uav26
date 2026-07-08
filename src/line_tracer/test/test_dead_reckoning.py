"""Unit tests for line_tracer.dead_reckoning."""
from math import isclose, pi

import pytest

from line_tracer.dead_reckoning import (
    BodyVelocity,
    DeadReckoning,
    Gains,
    State,
    clamp,
    compute_body_velocity,
    integrate,
    resolve_locked_yaw_error,
    snap_to_intersection,
    wrap_angle,
)
from line_tracer.grid import Grid


@pytest.fixture
def gains():
    return Gains(
        kp_xy=0.8,
        kp_yaw=1.2,
        kp_z=0.6,
        max_vxy=0.6,
        max_wz=1.0,
        target_altitude=2.0,
    )


# ---------- helpers ----------


@pytest.mark.parametrize("a,wrapped", [
    (0.0, 0.0),
    (pi, pi),
    (-pi, pi),
    (1.5 * pi, -0.5 * pi),
    (-1.5 * pi, 0.5 * pi),
    (3.0 * pi, pi),
])
def test_wrap_angle(a, wrapped):
    assert isclose(wrap_angle(a), wrapped, abs_tol=1e-12)


def test_clamp():
    assert clamp(0.5, -1.0, 1.0) == 0.5
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(5.0, -1.0, 1.0) == 1.0


# ---------- compute_body_velocity ----------


def test_zero_error_zero_velocity_at_target_altitude(gains):
    vel = compute_body_velocity(0.0, 0.0, 0.0, z_hat=2.0, gains=gains)
    assert vel == BodyVelocity(0.0, 0.0, 0.0, 0.0)


def test_md_p_gain_formula(gains):
    vel = compute_body_velocity(
        dx_body_m=0.5, dy_body_m=-0.25, psi_err=0.1,
        z_hat=2.0, gains=gains,
    )
    assert isclose(vel.vx, 0.8 * 0.5)
    assert isclose(vel.vy, 0.8 * -0.25)
    assert isclose(vel.wz, 1.2 * 0.1)
    assert vel.vz == 0.0


def test_vxy_clamped_to_max(gains):
    """vx/vy are clamped as a *vector magnitude*, not axis-wise. With
    equal-magnitude inputs the components share max_vxy evenly along the
    body diagonal (max_vxy / sqrt(2) each), and the direction is
    preserved. The earlier axis-wise clamp froze the body direction at
    45° whenever both axes saturated, which made the world-frame
    LINE_FOLLOW cruise circle the drone (r32)."""
    vel = compute_body_velocity(100.0, -100.0, 0.0, z_hat=2.0, gains=gains)
    mag = (vel.vx**2 + vel.vy**2) ** 0.5
    assert isclose(mag, gains.max_vxy)
    assert vel.vx > 0.0
    assert vel.vy < 0.0
    assert isclose(vel.vx, -vel.vy)


def test_wz_clamped(gains):
    vel = compute_body_velocity(0.0, 0.0, 50.0, z_hat=2.0, gains=gains)
    assert vel.wz == gains.max_wz
    vel_neg = compute_body_velocity(0.0, 0.0, -50.0, z_hat=2.0, gains=gains)
    assert vel_neg.wz == -gains.max_wz


def test_vz_drives_toward_target_altitude(gains):
    # Below target -> vz > 0 (climb).
    vel_below = compute_body_velocity(0.0, 0.0, 0.0, z_hat=1.0, gains=gains)
    assert vel_below.vz > 0.0
    assert isclose(vel_below.vz, 0.6 * (2.0 - 1.0))
    # Above target -> vz < 0 (descend).
    vel_above = compute_body_velocity(0.0, 0.0, 0.0, z_hat=3.0, gains=gains)
    assert vel_above.vz < 0.0


def test_vz_also_clamped_by_max_vxy(gains):
    # Huge altitude error: vz should clamp to ±max_vxy.
    vel = compute_body_velocity(0.0, 0.0, 0.0, z_hat=-1000.0, gains=gains)
    assert vel.vz == gains.max_vxy


# ---------- integrate ----------


def test_integrate_forward_at_zero_yaw():
    s = State(x=0.0, y=0.0, z=0.0, yaw=0.0)
    out = integrate(s, BodyVelocity(vx=1.0, vy=0.0, vz=0.0, wz=0.0), dt=0.5)
    assert isclose(out.x, 0.5) and isclose(out.y, 0.0)
    assert out.yaw == 0.0


def test_integrate_forward_at_quarter_turn():
    # yaw=+pi/2 -> body +x maps to world +y.
    s = State(yaw=pi / 2)
    out = integrate(s, BodyVelocity(vx=1.0, vy=0.0, vz=0.0, wz=0.0), dt=1.0)
    assert isclose(out.x, 0.0, abs_tol=1e-12)
    assert isclose(out.y, 1.0)


def test_integrate_left_at_zero_yaw():
    # body +y is left, world frame at yaw=0 -> world +y.
    s = State()
    out = integrate(s, BodyVelocity(vx=0.0, vy=1.0, vz=0.0, wz=0.0), dt=2.0)
    assert isclose(out.x, 0.0) and isclose(out.y, 2.0)


def test_integrate_z_independent_of_yaw():
    s = State(yaw=1.234)
    out = integrate(s, BodyVelocity(vx=0.0, vy=0.0, vz=0.3, wz=0.0), dt=2.0)
    assert isclose(out.z, 0.6)


def test_integrate_yaw_wraps():
    s = State(yaw=pi - 0.1)
    out = integrate(s, BodyVelocity(0.0, 0.0, 0.0, wz=1.0), dt=1.0)
    # pi - 0.1 + 1.0 = pi + 0.9 -> wraps to -pi + 0.9
    assert isclose(out.yaw, -pi + 0.9)


def test_integrate_zero_dt_is_noop():
    s = State(x=1.0, y=2.0, z=3.0, yaw=0.5)
    out = integrate(s, BodyVelocity(1.0, 1.0, 1.0, 1.0), dt=0.0)
    assert (out.x, out.y, out.z, out.yaw) == (1.0, 2.0, 3.0, 0.5)


# ---------- DeadReckoning orchestrator ----------


def test_dr_step_combines_velocity_and_integration(gains):
    dr = DeadReckoning(gains, State(z=2.0))  # at target altitude
    vel, state = dr.step(dx_body_m=0.1, dy_body_m=0.0, psi_err=0.0, dt=0.5)
    # vel.vx = 0.8 * 0.1 = 0.08; integrated x = 0.08 * 0.5 = 0.04
    assert isclose(vel.vx, 0.08)
    assert isclose(state.x, 0.04)
    assert isclose(state.z, 2.0)  # at target, vz=0


def test_dr_step_drives_altitude_toward_target(gains):
    dr = DeadReckoning(gains, State(z=0.0))  # ground
    _, state1 = dr.step(0.0, 0.0, 0.0, dt=0.5)
    _, state2 = dr.step(0.0, 0.0, 0.0, dt=0.5)
    # First step: vz = 0.6 * (2-0) = 1.2 -> clamped to 0.6, dz = 0.3
    assert isclose(state1.z, 0.3)
    # Second step: z=0.3, vz = 0.6 * (2-0.3) = 1.02 -> clamped to 0.6, dz = 0.3
    assert isclose(state2.z, 0.6)


def test_dr_reset(gains):
    dr = DeadReckoning(gains, State(x=5.0, y=5.0, z=5.0, yaw=1.0))
    dr.reset()
    assert dr.state == State()
    dr.reset(State(x=1.0))
    assert dr.state.x == 1.0


# ---------- snap_to_intersection ----------


@pytest.fixture
def default_grid():
    # 30 x 20 m floor, 4 m cells -> xs at 0,4,8,...,28,30; ys at 0,4,...,16,20
    return Grid.from_extents(width=30.0, depth=20.0, cell=4.0)


def test_snap_within_max_err_pulls_to_intersection(default_grid):
    s = State(x=4.3, y=7.7, z=2.0, yaw=0.5)
    out = snap_to_intersection(s, default_grid, max_err=2.0)
    assert out.x == 4.0 and out.y == 8.0
    assert out.z == 2.0 and out.yaw == 0.5     # preserved


def test_snap_outside_max_err_leaves_state_unchanged(default_grid):
    # (2, 2) is 2*sqrt(2)=2.83 m from (0,0) or (4,4); both > max_err=2.0
    s = State(x=2.0, y=2.0, z=2.0, yaw=0.0)
    out = snap_to_intersection(s, default_grid, max_err=2.0)
    assert out is s or (out.x == s.x and out.y == s.y)


def test_snap_exact_intersection_is_idempotent(default_grid):
    s = State(x=12.0, y=16.0, z=2.0, yaw=0.0)
    out = snap_to_intersection(s, default_grid, max_err=0.1)
    assert out.x == 12.0 and out.y == 16.0


def test_snap_preserves_z_and_yaw(default_grid):
    s = State(x=8.1, y=4.05, z=1.85, yaw=-1.3)
    out = snap_to_intersection(s, default_grid, max_err=1.0)
    assert isclose(out.x, 8.0) and isclose(out.y, 4.0)
    assert isclose(out.z, 1.85) and isclose(out.yaw, -1.3)


class TestResolveLockedYawError:
    """The mod-pi blindness fix: perception's line-alignment psi_err
    only fine-trims inside the vertical-band width; larger lock errors
    (90/180-degree flips, r61's marker-edge spin) are unwound by the
    absolute lock, and the 180-degree antipode unwinds in a
    deterministic direction instead of dithering."""

    def test_small_lock_error_passes_perception_through(self):
        assert resolve_locked_yaw_error(0.05, 0.0, 0.1) == 0.05

    def test_no_perception_falls_back_to_lock_error(self):
        got = resolve_locked_yaw_error(None, 0.0, 0.1)
        assert isclose(got, -0.1, abs_tol=1e-12)

    def test_exact_zero_perception_treated_as_absent(self):
        # Pre-existing convention: psi == 0.0 means "nothing fresh".
        got = resolve_locked_yaw_error(0.0, 0.0, 0.2)
        assert isclose(got, -0.2, abs_tol=1e-12)

    def test_large_lock_error_overrides_perception(self):
        """Drone spun 90 degrees: the perpendicular grid line reads as
        a small psi_err (mod-pi), but the lock must win."""
        got = resolve_locked_yaw_error(0.03, 0.0, pi / 2)
        assert isclose(got, -pi / 2, abs_tol=1e-12)

    def test_threshold_edge_inside_uses_perception(self):
        assert resolve_locked_yaw_error(0.1, 0.0, 0.55) == 0.1

    def test_antipode_unwinds_deterministically(self):
        """At yaw ~ 180 deg the wrapped error alternates sign with tiny
        wobbles; both sides of the flip must command the SAME direction
        so the P controller actually unwinds (r60/r61 dithered at
        +-max_wz forever)."""
        just_under = resolve_locked_yaw_error(None, 0.0, pi - 0.02)
        just_over = resolve_locked_yaw_error(None, 0.0, -(pi - 0.02))
        assert just_under < 0 or isclose(just_under, pi, abs_tol=0.15)
        # yaw = pi - 0.02 -> err = -(pi - 0.02): inside the fold band?
        # band is 0.1: -(pi - 0.02) < -(pi - 0.1) -> folded to +pi.
        assert isclose(just_under, pi, abs_tol=1e-12)
        # yaw just past the flip: err = +(pi - 0.02), kept positive.
        assert isclose(just_over, pi - 0.02, abs_tol=1e-12)
        # Same sign on both sides -> no dithering.
        assert just_under * just_over > 0

    def test_exact_antipode_is_positive_pi(self):
        got = resolve_locked_yaw_error(None, 0.0, pi)
        assert isclose(got, pi, abs_tol=1e-12)

    def test_respects_nonzero_start_yaw(self):
        got = resolve_locked_yaw_error(None, 1.0, 1.2)
        assert isclose(got, -0.2, abs_tol=1e-12)
