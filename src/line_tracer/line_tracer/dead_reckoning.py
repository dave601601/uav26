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
from math import cos, hypot, pi, sin
from typing import TYPE_CHECKING, Tuple

if TYPE_CHECKING:
    from .grid import Grid


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


@dataclass(frozen=True)
class AttiThrCmd:
    """Attitude + thrust setpoint, in the FC's reference frame. The
    rclpy node copies these fields into an fc_sim_msgs/Setpoint.
    ``armed=False`` means touchdown is complete: motors off, no
    attitude authority requested."""
    pitch_sp: float
    roll_sp: float
    yawrate_sp: float
    thrust_norm: float
    armed: bool = True


@dataclass(frozen=True)
class SetpointGains:
    # thrust_norm scale: the 2212-920KV/4S power train delivers 35.3 N
    # at thrust_norm=1.0 (900 gf x 4, fc_core max_thrust_g_per_motor).
    # Hover for the 1.182 kg frame sits at 0.33. All norms here are the
    # pre-2026-07 600 g-train values rescaled by 0.667 so the commanded
    # forces (in newtons) are unchanged.
    hover_thrust_norm: float = 0.33      # sim hover point
    kp_alt_thrust: float = 0.17          # thrust_norm per metre of alt_err
    kd_alt_thrust: float = 0.20          # thrust_norm per (m/s) of vz
    max_atti_setpoint_rad: float = 0.15  # roll / pitch clamp (~8.6°)
    thrust_min: float = 0.28
    # 0.60 = 21 N = 1.8x weight. The old force-parity value (0.47,
    # matching the 600 g train's 0.70) could not reliably arrest a
    # fast descent (r48 sank from 2.4 m into the floor); the 4S train
    # has 3x hover headroom, so use some of it for braking authority.
    thrust_max: float = 0.60
    # Takeoff burst — mirrors fc_sim/scripts/hover_pub.py. Plain PD at
    # thrust_max turned out to be insufficient to break the sphere
    # body_collision contact against the ground_plane in DartSim. When
    # the drone is on the floor and barely moving, we open-loop a
    # stronger thrust until vz indicates liftoff; the PD takes over once
    # the drone is rising or above takeoff_z_threshold. r24 showed that
    # bursting over a 0.30 m gap left ~3 m/s upward momentum that the
    # PD clamp couldn't brake — drone overshot to 10 m. The 0.43 / 0.15
    # pair (15.3 N, well above the 11.6 N weight) matches the force of
    # the proven 0.65 / 0.15 tuning on the old train.
    takeoff_z_threshold: float = 0.15
    takeoff_thrust_norm: float = 0.43
    # Body-velocity feedback: attitude rad per (m/s) of velocity error.
    # pitch_sp = kp_vel * (vx_cmd - vx_meas) closes the loop that the
    # open-loop pitch_sp = vx/g mapping leaves open — with zero drag in
    # gz, attitude proportional to *commanded velocity* is an
    # acceleration command, so speed integrates without bound (r39: the
    # drone exited the arena and slid 200 m along the ground after
    # touchdown). tau = 1/(g*kp_vel) ~= 1.0 s.
    kp_vel: float = 0.10
    # Touchdown cutoff: once LAND (target_alt ~ 0) has brought the
    # drone below this altitude, cut thrust entirely and disarm.
    # Without it the thrust clamp floor keeps the props near hover
    # force after touchdown and the near-weightless drone skates on
    # ground contact (r40: ~0.25 m/s creep after landing).
    land_cutoff_alt: float = 0.12
    # LAND descends on a VELOCITY command, not the altitude P law.
    # The altitude loop is P-only, so any feed-forward mis-trim turns
    # into a permanent offset: with hover_thrust_norm 0.046 above the
    # plant's true hover, every setpoint realized at target+0.27 m and
    # LAND parked at 0.27 m forever, above the cutoff (r52). Tracking
    # a descent rate instead is trim-independent: thrust settles at
    # whatever value actually sustains -0.3 m/s.
    land_descent_vz: float = -0.30


def body_vel_to_atti_thr(
    vel: BodyVelocity,
    target_alt: float,
    altitude: float,
    vz_truth: float,
    gains: SetpointGains,
    vx_meas: float | None = None,
    vy_meas: float | None = None,
) -> AttiThrCmd:
    """Map a body-frame velocity intent to (pitch, roll, yawrate, thrust).

    Sign convention matches the sim's empirical 2026-05-25 pitch-shim
    fix (`pitch_sp=+0.1` -> drone slides +X in FLU, `roll_sp=+0.1` ->
    drone slides -Y in FLU). The thrust formula is a PD on the world-Z
    error, clamped to [thrust_min, thrust_max] so a start-up alt error
    doesn't slam the FC's attitude loop into oscillation. The takeoff
    burst is the one path that bypasses the clamp — without it the PD
    saturates at thrust_max which is calibrated for in-flight altitude
    hold, not for breaking ground contact (see hover_pub.py:86).

    When a measured body velocity (vx_meas, vy_meas) is available the
    attitude command is a P on the *velocity error*, which is a real
    velocity loop: level attitude once the drone moves at the commanded
    speed, opposing attitude when it overshoots (this is what brakes
    the drone in LAND). Without a measurement the legacy open-loop
    vx/g mapping applies — that path commands a constant acceleration
    per unit of commanded velocity and must only be used where drag or
    short exposure bounds the speed.
    """
    g = 9.80665
    # Touchdown: LAND drives target_alt to 0; once the drone is on the
    # floor there is nothing left to control — motors off. Ordered
    # before everything else so no clamp can resurrect the thrust.
    if target_alt <= 0.05 and altitude <= gains.land_cutoff_alt:
        return AttiThrCmd(
            pitch_sp=0.0, roll_sp=0.0, yawrate_sp=0.0,
            thrust_norm=0.0, armed=False,
        )
    if vx_meas is not None and vy_meas is not None:
        pitch_sp = +gains.kp_vel * (vel.vx - vx_meas)
        roll_sp = -gains.kp_vel * (vel.vy - vy_meas)
    else:
        pitch_sp = +vel.vx / g
        roll_sp = -vel.vy / g
    pitch_sp = clamp(pitch_sp,
                     -gains.max_atti_setpoint_rad,
                     +gains.max_atti_setpoint_rad)
    roll_sp = clamp(roll_sp,
                    -gains.max_atti_setpoint_rad,
                    +gains.max_atti_setpoint_rad)
    alt_err = target_alt - altitude
    # LAND (target ~ ground): track a fixed descent rate instead of the
    # P law — see SetpointGains.land_descent_vz. Horizontal braking
    # (pitch/roll above) still applies during the descent.
    if target_alt <= 0.05:
        thrust = (gains.hover_thrust_norm
                  + gains.kd_alt_thrust * (gains.land_descent_vz - vz_truth))
        thrust = clamp(thrust, gains.thrust_min, gains.thrust_max)
        return AttiThrCmd(
            pitch_sp=pitch_sp,
            roll_sp=roll_sp,
            yawrate_sp=vel.wz,
            thrust_norm=thrust,
        )
    # Burst fires when drone is below threshold AND not already rising.
    # The earlier `abs(vz_truth) < 0.2` guard worked when vz_truth was
    # the (wrong-frame) gz twist value: small at startup so burst fired.
    # After switching to a true world-Z derivative, |vz| during the
    # initial spawn fall reads ~2 m/s and that guard blocks the burst —
    # drone hits ground at thrust_min, stuck (r19/r20/r21). Allow burst
    # while falling/stationary; only suppress it once vz > +0.2 m/s so
    # the PD takes over after liftoff.
    if (altitude < gains.takeoff_z_threshold
            and vz_truth < 0.2
            and alt_err > 0.5):
        thrust = gains.takeoff_thrust_norm
    else:
        thrust = (gains.hover_thrust_norm
                  + gains.kp_alt_thrust * alt_err
                  - gains.kd_alt_thrust * vz_truth)
        thrust = clamp(thrust, gains.thrust_min, gains.thrust_max)
    return AttiThrCmd(
        pitch_sp=pitch_sp,
        roll_sp=roll_sp,
        yawrate_sp=vel.wz,
        thrust_norm=thrust,
    )


def compute_body_velocity(
    dx_body_m: float,
    dy_body_m: float,
    psi_err: float,
    z_hat: float,
    gains: Gains,
) -> BodyVelocity:
    """Map metric body-frame offsets + altitude error to a clamped body Twist.

    vx/vy: P on lateral position error (driving the offset to zero).
           The clamp is a *vector magnitude* limit, not an axis-wise
           clamp — if dx_body and dy_body would both saturate, their
           ratio is preserved. The axis-wise clamp froze the direction
           at 45° body regardless of yaw (r32: drone circled CCW
           because world_to_body's correct body command became a
           saturated diagonal that rotated with the drone's yaw
           instead of pointing world +X).
    vz:    P on (target_altitude - z_hat).
    wz:    P on yaw error (psi_err is the heading error to remove).
    """
    vx_raw = gains.kp_xy * dx_body_m
    vy_raw = gains.kp_xy * dy_body_m
    mag = (vx_raw * vx_raw + vy_raw * vy_raw) ** 0.5
    if mag > gains.max_vxy and mag > 0.0:
        scale = gains.max_vxy / mag
        vx = vx_raw * scale
        vy = vy_raw * scale
    else:
        vx = vx_raw
        vy = vy_raw
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


def world_to_body(err_x_world: float, err_y_world: float, yaw: float) -> Tuple[float, float]:
    """Rotate a world-frame XY error into the body FLU frame.

    Inverse of the rotation in :func:`integrate`: body forward (+x_body)
    points along world yaw, so to project a world delta onto body axes:

        dx_body =  cos(yaw) * dx_world + sin(yaw) * dy_world
        dy_body = -sin(yaw) * dx_world + cos(yaw) * dy_world
    """
    c, s = cos(yaw), sin(yaw)
    return c * err_x_world + s * err_y_world, -s * err_x_world + c * err_y_world


def snap_to_intersection(
    state: State, grid: "Grid", max_err: float = 2.0
) -> State:
    """Snap (x, y) to the nearest grid intersection if within ``max_err`` m.

    Used during WAYPOINT_VISIT when an ArUco marker is centered in the
    downward camera: the marker is known to sit on a grid intersection, so
    the sighting is an absolute XY fix. Returns a new ``State`` with z and
    yaw preserved; if the nearest intersection is farther than ``max_err``
    the original state is returned unchanged (refuses to snap when we have
    no reason to believe we're actually over an intersection).
    """
    node = grid.nearest_node(state.x, state.y)
    nx, ny = grid.world(node)
    if hypot(nx - state.x, ny - state.y) > max_err:
        return state
    return State(x=nx, y=ny, z=state.z, yaw=state.yaw)
