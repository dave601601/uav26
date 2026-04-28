"""Dead-reckoning controller stub.

Pure-Python (no rclpy/ROS msg deps) so it is unit-testable without a ROS env.
The node layer (line_tracer_node) converts between pixel-error topics, this
module's metric inputs, and ROS Twist/Odometry messages.

Frame conventions
-----------------
Body frame is FLU (REP-103): +x forward, +y left, +z up.
World frame is ENU. Body velocity (vx, vy) is rotated by current yaw to
update world position. wz is yaw rate.

Inputs to ``compute_body_velocity`` are *body-frame* metric offsets — the
caller is responsible for the camera->body rotation (mount geometry lives
outside this module).
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from math import cos, pi, sin
from typing import Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = (a + pi) % (2.0 * pi) - pi
    # Python's % returns 0 instead of 2pi for exact multiples; flip -pi to +pi
    # to make the interval (-pi, pi].
    return pi if a == -pi else a


@dataclass(frozen=True)
class Gains:
    kp_xy: float
    kp_yaw: float
    kp_z: float
    max_vxy: float
    max_wz: float
    target_altitude: float


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class BodyVelocity:
    vx: float
    vy: float
    vz: float
    wz: float


def compute_body_velocity(
    dx_body_m: float,
    dy_body_m: float,
    psi_err: float,
    z_hat: float,
    gains: Gains,
) -> BodyVelocity:
    """Map metric body-frame offsets + altitude error to a clamped body Twist.

    vx/vy: P on lateral position error (driving the offset to zero).
    vz:    P on (target_altitude - z_hat).
    wz:    P on yaw error (psi_err is the heading error to remove).
    """
    vx = clamp(gains.kp_xy * dx_body_m, -gains.max_vxy, gains.max_vxy)
    vy = clamp(gains.kp_xy * dy_body_m, -gains.max_vxy, gains.max_vxy)
    vz = clamp(gains.kp_z * (gains.target_altitude - z_hat),
               -gains.max_vxy, gains.max_vxy)
    wz = clamp(gains.kp_yaw * psi_err, -gains.max_wz, gains.max_wz)
    return BodyVelocity(vx=vx, vy=vy, vz=vz, wz=wz)


def integrate(state: State, vel: BodyVelocity, dt: float) -> State:
    """Forward-Euler integration of a body Twist into the world State.

    Body (vx, vy) is rotated by current yaw to produce world (dx, dy).
    """
    if dt <= 0.0:
        return replace(state)
    c, s = cos(state.yaw), sin(state.yaw)
    dx_w = (vel.vx * c - vel.vy * s) * dt
    dy_w = (vel.vx * s + vel.vy * c) * dt
    return State(
        x=state.x + dx_w,
        y=state.y + dy_w,
        z=state.z + vel.vz * dt,
        yaw=wrap_angle(state.yaw + vel.wz * dt),
    )


class DeadReckoning:
    """Stateful wrapper. Holds gains + current State, exposes one ``step``."""

    def __init__(self, gains: Gains, state: State | None = None) -> None:
        self.gains = gains
        self.state = state if state is not None else State()

    def reset(self, state: State | None = None) -> None:
        self.state = state if state is not None else State()

    def step(
        self,
        dx_body_m: float,
        dy_body_m: float,
        psi_err: float,
        dt: float,
    ) -> Tuple[BodyVelocity, State]:
        vel = compute_body_velocity(
            dx_body_m, dy_body_m, psi_err, self.state.z, self.gains
        )
        self.state = integrate(self.state, vel, dt)
        return vel, self.state
