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
    wrap_angle,
)


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
    vel = compute_body_velocity(100.0, -100.0, 0.0, z_hat=2.0, gains=gains)
    assert vel.vx == gains.max_vxy
    assert vel.vy == -gains.max_vxy


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
