"""Closed-loop mission tests — no rclpy, no gz, no sim binaries.

Stitches together:
  * MockDrone     — point-mass dynamics that consumes an
                    ``AttiThrCmd`` (pitch_sp, roll_sp, yawrate_sp,
                    thrust_norm) and integrates a 3-DOF world state.
                    Attitude tracking is treated as instantaneous: the
                    test isolates whether the FSM + setpoint mapping
                    produce the *right intent*, not whether the inner
                    attitude loop tracks that intent.
  * SyntheticCam  — emits a PerceptionResult that contains an
                    ArucoDetection whenever the drone is over (within
                    a configurable radius of) one of the planted
                    markers. Grid-line errors are deliberately left
                    None so the test exercises the cruise_vx /
                    target_xy_world paths, not the dv-based perception
                    that the M-A LINE_FOLLOW intentionally turned off.

The tests then drive ``StateMachine.tick`` +
``dead_reckoning.body_vel_to_atti_thr`` in a loop and assert that the
mission walks all phases TAKEOFF -> LAND, that the four marker IDs
are recorded, and that the recorded XY are within ``snap_max_err`` of
the ground-truth markers.

These are the missing tests called out after r10/r14 visualizations
showed the existing 104 unit tests passing while the actual sim run
left the drone outside the mission area.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import pytest

from line_tracer.dead_reckoning import (
    AttiThrCmd,
    BodyVelocity,
    SetpointGains,
    State,
    body_vel_to_atti_thr,
    compute_body_velocity,
    Gains,
    wrap_angle,
    world_to_body,
)
from line_tracer.grid import Grid
from line_tracer.perception import ArucoDetection, PerceptionResult
from line_tracer.side_camera import CandidateTracker
from line_tracer.state_machine import (
    MissionContext,
    StateMachine,
    StateName,
)


GRAVITY = 9.80665
DRONE_MASS = 1.182
# fc_core's mapping; thrust_norm=1.0 -> this many N. 900 gf per motor
# matches the 2212-920KV / 4S power train (controller.c
# max_thrust_g_per_motor).
THRUST_FULL_NORM_TO_N = 4.0 * 0.9 * GRAVITY
HOVER_NORM = DRONE_MASS * GRAVITY / THRUST_FULL_NORM_TO_N    # ~0.328


# ---------------------------------------------------------------------------
# Mock drone — instantaneous-attitude point mass.
# ---------------------------------------------------------------------------


@dataclass
class MockDrone:
    """Point-mass drone with instantaneous attitude tracking + light drag.

    Convention matches the sim's empirical pitch / roll signs after the
    2026-05-25 pitch-shim fix:

        ax_body = +g * pitch_sp     (so +pitch -> +X_body acceleration)
        ay_body = -g * roll_sp      (so +roll  -> -Y_body acceleration)

    Yaw integrates the commanded ``yawrate_sp`` directly. Linear drag
    (``drag_coef`` per (m/s)) is kept SMALL on purpose: the gz model has
    rotorDragCoefficient=0, so a mock with strong drag validates a
    braking behaviour the real sim doesn't have. r39 proved the point —
    the old drag_coef=1.0 mock passed missions while the gz drone
    exited the arena and slid 200 m after touchdown. Convergence must
    come from the velocity feedback in body_vel_to_atti_thr, not from
    the mock's aerodynamics.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0      # world frame
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    drag_coef: float = 0.1    # 1/s; token residual air drag only.

    def step(self, cmd: AttiThrCmd, dt: float) -> None:
        ax_body = +GRAVITY * math.sin(cmd.pitch_sp)
        ay_body = -GRAVITY * math.sin(cmd.roll_sp)

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        ax = ax_body * cy - ay_body * sy
        ay = ax_body * sy + ay_body * cy

        f_up = cmd.thrust_norm * THRUST_FULL_NORM_TO_N
        az = (f_up - DRONE_MASS * GRAVITY) / DRONE_MASS

        # Apply input acceleration, then linear drag on world velocity.
        self.vx = (self.vx + ax * dt) * max(0.0, 1.0 - self.drag_coef * dt)
        self.vy = (self.vy + ay * dt) * max(0.0, 1.0 - self.drag_coef * dt)
        self.vz = (self.vz + az * dt) * max(0.0, 1.0 - self.drag_coef * dt)

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z = max(0.0, self.z + self.vz * dt)
        if self.z == 0.0 and self.vz < 0.0:
            # Park on the floor instead of going negative.
            self.vz = 0.0
        self.yaw += cmd.yawrate_sp * dt

    def to_dr_state(self) -> State:
        return State(x=self.x, y=self.y, z=self.z, yaw=self.yaw)


# ---------------------------------------------------------------------------
# Synthetic perception
# ---------------------------------------------------------------------------


@dataclass
class SyntheticCam:
    """Returns a PerceptionResult with one ArucoDetection per marker
    within ``view_radius_m`` of the drone's xy. Line errors stay None
    (drone follows cruise_vx, not perception, for LINE_FOLLOW)."""
    markers: dict[int, tuple[float, float]] = field(default_factory=dict)
    view_radius_m: float = 1.5
    min_altitude: float = 1.0    # below this, camera is too close to detect

    def perceive(self, drone: MockDrone) -> PerceptionResult:
        if drone.z < self.min_altitude:
            return PerceptionResult()
        dets: list[ArucoDetection] = []
        for mid, (mx, my) in self.markers.items():
            if math.hypot(mx - drone.x, my - drone.y) <= self.view_radius_m:
                dets.append(ArucoDetection(
                    id=mid,
                    center_uv=(320.0, 240.0),    # image center
                    corners_uv=((0.0, 0.0),) * 4,
                ))
        return PerceptionResult(aruco=dets)


@dataclass
class SideCam:
    """Synthetic sideways lookahead camera (the OV9281+6mm model).

    Mirrors the sim geometry at 2 m altitude: markers whose +Y offset
    from the drone falls inside the VFOV ground band [3.0, 10.6] m and
    within the 35.5 deg HFOV cone are 'observed' at their ground-truth
    xy (projection accuracy is unit-tested separately in
    test_side_camera.py; this model exercises tracker + FSM).
    ``blind_ids`` simulates missed detections — the safety-net fallback
    sweep must still find those markers."""
    markers: dict[int, tuple[float, float]] = field(default_factory=dict)
    band_near_m: float = 3.0
    band_far_m: float = 10.6
    hfov_half_tan: float = 0.3204     # tan(17.75 deg)
    min_altitude: float = 1.5
    blind_ids: frozenset = frozenset()

    def observe_into(
        self, drone: MockDrone, tracker: CandidateTracker,
        now: float, grid: Grid,
    ) -> None:
        if drone.z < self.min_altitude:
            return
        for mid, (mx, my) in self.markers.items():
            if mid in self.blind_ids:
                continue
            lateral = my - drone.y     # camera faces body +Y (yaw locked)
            if not (self.band_near_m <= lateral <= self.band_far_m):
                continue
            if abs(mx - drone.x) > lateral * self.hfov_half_tan:
                continue
            slant = math.hypot(lateral, drone.z)
            tracker.observe(mid, mx, my, slant, now, grid)


# ---------------------------------------------------------------------------
# Closed-loop driver
# ---------------------------------------------------------------------------


@dataclass
class MissionRunRecord:
    final_state: StateName
    end_pose: tuple[float, float, float]
    records: dict[int, tuple[float, float]]
    state_sequence: list[StateName]
    elapsed_s: float
    fsm: StateMachine
    drone: MockDrone


def run_mission(
    markers: dict[int, tuple[float, float]],
    *,
    start_xy: tuple[float, float] = (2.0, 4.0),
    start_z: float = 1.5,
    start_yaw: float = 0.0,
    yaw_drift_per_s: float = 0.0,    # synthetic firmware-side yaw drift
    dt: float = 0.025,
    max_seconds: float = 200.0,
    grid: Optional[Grid] = None,
    cruise_vx_override: Optional[float] = None,
    max_records: Optional[int] = None,
    use_lookahead: bool = False,
    sweep_row_step: int = 1,
    side_blind_ids: frozenset = frozenset(),
) -> MissionRunRecord:
    """Replicate the node's per-tick body of ``_on_dr_tick`` without rclpy."""
    if grid is None:
        grid = Grid.from_extents(width=30.0, depth=20.0, cell=4.0)

    drone = MockDrone(x=start_xy[0], y=start_xy[1], z=start_z, yaw=start_yaw)
    cam = SyntheticCam(markers=markers)
    side_cam = (
        SideCam(markers=markers, blind_ids=side_blind_ids)
        if use_lookahead else None
    )
    tracker = CandidateTracker()

    # Default: the FSM waits for as many records as we planted. Caller
    # can override (e.g. takeoff-only tests use a large value so the
    # mission stays in LINE_FOLLOW rather than triggering an immediate
    # retrieval on len(records) >= 0).
    effective_max = max_records if max_records is not None else max(1, len(markers))

    ctx = MissionContext(
        grid=grid,
        max_records=effective_max,
        takeoff_streak_required=5,
        waypoint_hover_seconds=1.0,
        waypoint_arrival_dist=0.5,
        return_arrival_dist=0.5,
        snap_max_err=2.0,
        sweep_row_step=sweep_row_step,
        candidate_wait_seconds=2.0,
    )
    fsm = StateMachine(initial=StateName.TAKEOFF, target_altitude=2.0, context=ctx)

    gains = Gains(
        kp_xy=0.8, kp_yaw=1.0, kp_z=0.6,
        max_vxy=1.0, max_wz=1.0, target_altitude=2.0,
    )
    sp_gains = SetpointGains()

    now = 0.0
    state_sequence: list[StateName] = []

    while now < max_seconds:
        # Mirror line_tracer_node._on_dr_tick (+ _on_lookahead).
        perception = cam.perceive(drone)
        candidates = None
        if side_cam is not None:
            side_cam.observe_into(drone, tracker, now, grid)
            candidates = tracker.snapshot(3, grid)
        tick_res = fsm.tick(now=now,
                            dr_state=drone.to_dr_state(),
                            perception=perception,
                            altitude=drone.z,
                            candidates=candidates)
        if not state_sequence or state_sequence[-1] is not tick_res.state:
            state_sequence.append(tick_res.state)
        behavior = tick_res.behavior

        # behavior + (optional) world target -> body offsets
        dx_body = 0.0
        dy_body = 0.0
        if tick_res.target_xy_world is not None:
            tx, ty = tick_res.target_xy_world
            dx_body, dy_body = world_to_body(tx - drone.x, ty - drone.y, drone.yaw)
        elif (behavior.cruise_vx != 0.0
              and not behavior.use_forward_error
              and gains.kp_xy != 0.0):
            cruise = (cruise_vx_override
                      if cruise_vx_override is not None
                      else behavior.cruise_vx)
            cruise_mag = cruise / gains.kp_xy
            # World-frame cruise rotated into body so yaw drift doesn't
            # bend the track (mirrors line_tracer_node._on_dr_tick).
            if ctx.start_yaw is not None:
                dx_w = cruise_mag * math.cos(ctx.start_yaw)
                dy_w = cruise_mag * math.sin(ctx.start_yaw)
                dx_body, dy_body = world_to_body(dx_w, dy_w, drone.yaw)
            else:
                dx_body = cruise_mag

        # Yaw-lock fallback (mirrors line_tracer_node._on_dr_tick).
        psi = 0.0
        if (behavior.lock_yaw_to_initial
                and ctx.start_yaw is not None):
            psi = wrap_angle(ctx.start_yaw - drone.yaw)

        # vel + atti / thrust
        gains_now = Gains(
            kp_xy=gains.kp_xy, kp_yaw=gains.kp_yaw, kp_z=gains.kp_z,
            max_vxy=gains.max_vxy, max_wz=gains.max_wz,
            target_altitude=behavior.target_altitude,
        )
        vel = compute_body_velocity(dx_body, dy_body, psi, drone.z, gains_now)
        # Measured body velocity — mirrors line_tracer_node._build_setpoint
        # (world /odom_truth derivative rotated into body FLU).
        vx_meas, vy_meas = world_to_body(drone.vx, drone.vy, drone.yaw)
        cmd = body_vel_to_atti_thr(
            vel=vel,
            target_alt=behavior.target_altitude,
            altitude=drone.z,
            vz_truth=drone.vz,
            gains=sp_gains,
            vx_meas=vx_meas,
            vy_meas=vy_meas,
        )
        drone.step(cmd, dt)
        # Synthetic firmware-side yaw drift, applied AFTER the
        # controller acted so the lock has something to fight.
        if yaw_drift_per_s != 0.0:
            drone.yaw += yaw_drift_per_s * dt
        now += dt

        if tick_res.state is StateName.LAND and drone.z < 0.1:
            break

    return MissionRunRecord(
        final_state=fsm.state,
        end_pose=(drone.x, drone.y, drone.z),
        records=dict(ctx.records),
        state_sequence=state_sequence,
        elapsed_s=now,
        fsm=fsm,
        drone=drone,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestMissionClosedLoop:
    """Each test runs a complete mission in-process and checks the
    invariants the existing 'tick() with synthetic inputs' tests miss."""

    def _markers_on_x_axis(self) -> dict[int, tuple[float, float]]:
        # All four markers on the y=4 line so a forward-cruising drone
        # actually flies over them.
        return {0: (8.0, 4.0), 1: (12.0, 4.0), 2: (16.0, 4.0), 3: (20.0, 4.0)}

    def test_drone_takes_off(self):
        """The first thing M-A's headless run failed at: drone never
        crossed alt=1.8 m. Verify the closed-loop driver gets there.

        Uses max_records=99 (unreachable) so the FSM doesn't fire
        retrieval immediately when len(records) >= 0 would be true on
        an empty marker set."""
        result = run_mission({}, max_seconds=10.0, max_records=99)
        assert StateName.LINE_FOLLOW in result.state_sequence, (
            f"never left TAKEOFF; state seq = {[s.name for s in result.state_sequence]}"
        )
        assert result.drone.z >= 1.5, (
            f"drone did not reach the takeoff altitude band (z={result.drone.z:.2f})"
        )

    def test_all_four_markers_recorded(self):
        result = run_mission(self._markers_on_x_axis(), max_seconds=120.0)
        assert len(result.records) == 4, (
            f"records={result.records}, expected 4 unique ids"
        )
        assert set(result.records.keys()) == {0, 1, 2, 3}

    def test_recorded_xy_near_ground_truth(self):
        markers = self._markers_on_x_axis()
        result = run_mission(markers, max_seconds=120.0)
        for mid, gt in markers.items():
            rec = result.records.get(mid)
            assert rec is not None, f"id {mid} never recorded"
            err = math.hypot(rec[0] - gt[0], rec[1] - gt[1])
            assert err < 2.0, (   # within snap_max_err
                f"id {mid}: recorded {rec} vs gt {gt}, err {err:.2f} > 2.0"
            )

    def test_fsm_reaches_land(self):
        result = run_mission(self._markers_on_x_axis(), max_seconds=200.0)
        assert result.final_state is StateName.LAND, (
            f"final state {result.final_state.name}; "
            f"state seq = {[s.name for s in result.state_sequence]}"
        )

    def test_state_sequence_in_order(self):
        result = run_mission(self._markers_on_x_axis(), max_seconds=200.0)
        # Required milestones; intermediate LINE_FOLLOW <-> WAYPOINT_VISIT
        # oscillation is OK, this just checks ordering.
        required = [
            StateName.TAKEOFF, StateName.LINE_FOLLOW,
            StateName.WAYPOINT_VISIT,
            StateName.ARRANGE_BY_ID, StateName.RETURN_PATH, StateName.LAND,
        ]
        seq = result.state_sequence
        for name in required:
            assert name in seq, (
                f"{name.name} missing from state sequence "
                f"{[s.name for s in seq]}"
            )
        # Ordering: each milestone's first appearance must be in the
        # required order.
        firsts = [seq.index(name) for name in required]
        assert firsts == sorted(firsts), (
            f"states out of order: firsts = {firsts}, "
            f"seq = {[s.name for s in seq]}"
        )

    def test_lands_near_start(self):
        result = run_mission(self._markers_on_x_axis(), max_seconds=200.0)
        ex, ey, _ = result.end_pose
        dist = math.hypot(ex - 2.0, ey - 4.0)
        assert dist < 3.0, (   # the return_arrival_dist + brake overshoot
            f"final xy ({ex:.2f},{ey:.2f}) too far from start (2,4): "
            f"{dist:.2f} m"
        )


class TestAttiThrSign:
    """Pin the sign + clamp behaviour of body_vel_to_atti_thr — the M-A
    pitch-sign bug only became visible after a sim run, never in a
    unit test."""

    def _gains(self) -> SetpointGains:
        return SetpointGains()

    def test_positive_vx_yields_positive_pitch(self):
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.5, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
        )
        assert cmd.pitch_sp > 0, (
            "vx=+0.5 must map to +pitch_sp (the sim convention since the "
            "2026-05-25 pitch-shim fix is pitch_sp=+0.1 -> +X)"
        )

    def test_positive_vy_yields_negative_roll(self):
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.5, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
        )
        assert cmd.roll_sp < 0, "vy=+0.5 must map to -roll_sp"

    def test_velocity_feedback_brakes_overshoot(self):
        """Command zero velocity while the drone still moves forward:
        the velocity loop must pitch BACK (negative) to brake. The
        open-loop mapping can't do this — it commanded level attitude
        for vx=0 and let the drone coast (r39: 200 m ground slide)."""
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
            vx_meas=1.0, vy_meas=0.0,
        )
        assert cmd.pitch_sp < 0, (
            "vx_cmd=0 with vx_meas=+1 must command negative pitch to brake"
        )

    def test_velocity_feedback_level_when_tracking(self):
        """At the commanded speed the attitude must return to level —
        that is what makes it a velocity loop instead of an
        acceleration command."""
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.5, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
            vx_meas=0.5, vy_meas=0.0,
        )
        assert math.isclose(cmd.pitch_sp, 0.0, abs_tol=1e-9)
        assert math.isclose(cmd.roll_sp, 0.0, abs_tol=1e-9)

    def test_velocity_feedback_roll_sign(self):
        """+vy error (commanded left faster than measured) -> -roll_sp,
        same sign convention as the open-loop path."""
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.5, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
            vx_meas=0.0, vy_meas=0.0,
        )
        assert cmd.roll_sp < 0

    def test_open_loop_fallback_without_measurement(self):
        """No measurement (hardware without an estimator, or stale
        odom) -> legacy vx/g mapping still applies."""
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.5, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=g,
        )
        assert math.isclose(cmd.pitch_sp, 0.5 / 9.80665, rel_tol=1e-6)

    def test_pitch_clamped_to_max(self):
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=100.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=self._gains(),
        )
        g = self._gains()
        assert math.isclose(cmd.pitch_sp, g.max_atti_setpoint_rad)

    def test_thrust_clamped_to_band(self):
        g = self._gains()
        # Above takeoff_z_threshold (no burst path): a far-above-target
        # altitude error must clamp at thrust_max.
        hi = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=20.0, altitude=1.5, vz_truth=0.0, gains=g,
        )
        assert math.isclose(hi.thrust_norm, g.thrust_max)
        lo = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=0.0, altitude=10.0, vz_truth=0.0, gains=g,
        )
        assert math.isclose(lo.thrust_norm, g.thrust_min)

    def test_hover_thrust_when_at_target(self):
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=0.0, gains=g,
        )
        assert math.isclose(cmd.thrust_norm, g.hover_thrust_norm)

    def test_descending_vz_increases_thrust(self):
        """The kd term should fight a downward velocity, not amplify it."""
        g = self._gains()
        # vz_truth = -0.5 (drone falling). thrust should rise above hover.
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=2.0, vz_truth=-0.5, gains=g,
        )
        assert cmd.thrust_norm > g.hover_thrust_norm


class TestTakeoffBurst:
    """Pin the open-loop takeoff branch in body_vel_to_atti_thr — the
    PD-clamped 0.70 thrust_max isn't enough to break sphere/ground
    contact in DartSim (see hover_pub.py:86 and the 2026-05-25
    ground-stick failure analysis)."""

    def _gains(self) -> SetpointGains:
        return SetpointGains()

    def test_burst_when_on_ground_and_below_target(self):
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=0.05, vz_truth=0.0, gains=g,
        )
        assert math.isclose(cmd.thrust_norm, g.takeoff_thrust_norm), (
            f"on-ground takeoff must emit {g.takeoff_thrust_norm}, "
            f"got {cmd.thrust_norm}"
        )
        assert cmd.thrust_norm > g.hover_thrust_norm, (
            "burst must exceed hover_thrust_norm so the drone actually "
            "lifts off the ground"
        )

    def test_burst_fires_while_falling(self):
        """Regression: the original guard was abs(vz_truth) < 0.2, which
        suppressed the burst during the initial spawn fall (vz ~= -2 m/s)
        and produced the r19..r22 ground-stick. The drone must burst
        while falling so it has a chance to arrest the fall."""
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=0.10, vz_truth=-2.0, gains=g,
        )
        assert math.isclose(cmd.thrust_norm, g.takeoff_thrust_norm), (
            f"falling drone below threshold must emit burst "
            f"{g.takeoff_thrust_norm}, got {cmd.thrust_norm}. The pre-r23 "
            f"abs() guard would have failed this case."
        )

    def test_no_burst_when_airborne(self):
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=1.5, vz_truth=0.0, gains=g,
        )
        assert cmd.thrust_norm <= g.thrust_max, (
            "above takeoff_z_threshold the PD clamp must apply"
        )

    def test_no_burst_when_already_rising(self):
        """Once vz_truth indicates liftoff the PD takes over so the
        drone doesn't continue accelerating after it's airborne."""
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=0.10, vz_truth=0.5, gains=g,
        )
        assert cmd.thrust_norm <= g.thrust_max

    def test_no_burst_when_target_below_self(self):
        """Negative alt_err (e.g. LAND state, target_alt=0, drone in
        air) must never trigger the burst — burst is a TAKEOFF-only
        artefact."""
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=0.0, altitude=0.10, vz_truth=0.0, gains=g,
        )
        assert cmd.thrust_norm <= g.thrust_max


class TestLandCutoff:
    """Touchdown must kill the motors. r40 showed the thrust clamp
    floor keeping the landed drone near-weightless — it skated along
    the ground at ~0.25 m/s indefinitely."""

    def _gains(self) -> SetpointGains:
        return SetpointGains()

    def test_cutoff_on_touchdown(self):
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=0.0, altitude=0.05, vz_truth=0.0, gains=g,
        )
        assert cmd.thrust_norm == 0.0
        assert cmd.armed is False
        assert cmd.pitch_sp == 0.0 and cmd.roll_sp == 0.0

    def test_no_cutoff_while_descending_in_air(self):
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=0.0, altitude=0.50, vz_truth=-0.3, gains=g,
        )
        assert cmd.armed is True
        assert cmd.thrust_norm >= g.thrust_min

    def test_no_cutoff_during_takeoff_on_ground(self):
        """TAKEOFF (target well above ground) at spawn altitude must
        burst, not cut off — the cutoff is exclusively a LAND path."""
        g = self._gains()
        cmd = body_vel_to_atti_thr(
            vel=BodyVelocity(vx=0.0, vy=0.0, vz=0.0, wz=0.0),
            target_alt=2.0, altitude=0.05, vz_truth=0.0, gains=g,
        )
        assert cmd.armed is True
        assert math.isclose(cmd.thrust_norm, g.takeoff_thrust_norm)


class TestYawLock:
    """The mission FSM marks TAKEOFF / LINE_FOLLOW / WAYPOINT_VISIT /
    ARRANGE_BY_ID / RETURN_PATH with ``lock_yaw_to_initial=True`` so a
    firmware-side yaw drift doesn't twist the cruise heading off-axis
    (r15 showed ~20° drift sending the drone to (148, -57))."""

    def _markers_on_x_axis(self) -> dict[int, tuple[float, float]]:
        return {0: (8.0, 4.0), 1: (12.0, 4.0), 2: (16.0, 4.0), 3: (20.0, 4.0)}

    def test_yaw_stays_near_initial_with_constant_drift(self):
        """With a constant 0.10 rad/s firmware drift, the closed loop
        should hold yaw within ~0.2 rad (~11°) of the start heading
        across the whole takeoff + line follow + visit cycle."""
        result = run_mission(
            self._markers_on_x_axis(),
            yaw_drift_per_s=0.10,
            max_seconds=30.0,
            max_records=99,    # stay in LINE_FOLLOW, don't trigger ARRANGE
        )
        # Sample yaw at the final tick; the lock loop is P-only so the
        # steady-state error = drift / (kp_yaw / 1) = 0.10 / 1.0 = 0.10.
        # Allow 2x headroom for transients + integration noise.
        assert abs(result.drone.yaw) < 0.2, (
            f"yaw locked to {result.drone.yaw:+.3f} rad after 30 s of "
            f"0.10 rad/s drift; expected within ±0.2"
        )

    def test_cruise_track_stays_in_x_direction(self):
        """With yaw locked, cruise_vx in body +X should produce a
        trajectory close to a +X straight line in world frame."""
        result = run_mission(
            self._markers_on_x_axis(),
            yaw_drift_per_s=0.10,
            max_seconds=30.0,
            max_records=99,
        )
        # Trajectory should be predominantly +X. |y - start_y| should
        # stay small (the lock can't be perfect, but the y deviation
        # has to be much smaller than the x progress).
        x_progress = result.drone.x - 2.0     # start_xy[0]
        y_deviation = abs(result.drone.y - 4.0)
        assert x_progress > 5.0, (
            f"drone barely moved in +X: only {x_progress:.2f} m progress"
        )
        assert y_deviation < x_progress * 0.5, (
            f"y deviation {y_deviation:.2f} too large vs. x progress "
            f"{x_progress:.2f} — yaw lock isn't holding"
        )

    def test_no_lock_means_drift_compounds(self):
        """Sanity check: disabling the lock on LINE_FOLLOW lets the
        synthetic drift turn the cruise track sideways."""
        from line_tracer import state_machine as sm

        # Monkey-patch the LINE_FOLLOW behavior for this test only.
        original = sm._BEHAVIORS[sm.StateName.LINE_FOLLOW]
        sm._BEHAVIORS[sm.StateName.LINE_FOLLOW] = replace_behavior(
            original, lock_yaw_to_initial=False,
        )
        try:
            result = run_mission(
                self._markers_on_x_axis(),
                yaw_drift_per_s=0.10,
                max_seconds=30.0,
                max_records=99,
            )
        finally:
            sm._BEHAVIORS[sm.StateName.LINE_FOLLOW] = original

        # Drift integrates uncontested -> yaw should be far off zero.
        assert abs(result.drone.yaw) > 0.5, (
            f"with the lock disabled, yaw should compound past 0.5 rad; "
            f"saw {result.drone.yaw:+.3f}"
        )


def replace_behavior(b, **overrides):
    """Helper: produce a new frozen Behavior with the given fields
    overridden. Used by the no-lock sanity test to monkey-patch the
    LINE_FOLLOW behavior without rewriting the whole dict."""
    from dataclasses import replace
    return replace(b, **overrides)


class TestLookaheadMission:
    """Candidate-directed missions end to end: the seed-42-like layout
    (markers on the first and last interior rows) is where the row-skip
    + candidate design pays — the two middle rows never get flown."""

    def _corner_markers(self) -> dict[int, tuple[float, float]]:
        # Mirrors the seed-42 ground truth: two markers on row 4, two on
        # row 16, nothing on rows 8/12.
        return {2: (4.0, 4.0), 0: (24.0, 4.0), 3: (24.0, 16.0), 1: (4.0, 16.0)}

    def test_lookahead_records_all_four_at_true_cells(self):
        result = run_mission(
            self._corner_markers(),
            max_seconds=500.0,
            use_lookahead=True,
            sweep_row_step=2,
        )
        assert set(result.records) == {0, 1, 2, 3}
        for mid, gt in self._corner_markers().items():
            rec = result.records[mid]
            assert math.hypot(rec[0] - gt[0], rec[1] - gt[1]) < 0.01, (
                f"id {mid}: recorded {rec} vs gt {gt} — snap must be exact"
            )
        assert result.final_state is StateName.LAND
        # Row-16 markers must have arrived via candidates, not the sweep.
        assert StateName.GOTO_CANDIDATE in result.state_sequence

    def test_lookahead_beats_full_sweep_on_time(self):
        markers = self._corner_markers()
        base = run_mission(markers, max_seconds=600.0)
        fast = run_mission(markers, max_seconds=600.0,
                           use_lookahead=True, sweep_row_step=2)
        assert base.final_state is StateName.LAND
        assert fast.final_state is StateName.LAND
        assert set(base.records) == set(fast.records) == {0, 1, 2, 3}
        # The baseline must not contain the new state; the lookahead run
        # must be meaningfully faster (it skips rows 8 and 12 entirely
        # and short-circuits the rest of row 12's return leg).
        assert StateName.GOTO_CANDIDATE not in base.state_sequence
        assert fast.elapsed_s < base.elapsed_s - 20.0, (
            f"lookahead {fast.elapsed_s:.0f}s vs baseline "
            f"{base.elapsed_s:.0f}s — expected >20 s saving"
        )

    def test_side_blind_marker_recovered_by_fallback_sweep(self):
        """A marker the side camera never sees (missed detection) sits
        on a skipped row: the one-shot fallback sweep must still fly
        that row and record it with the downward camera."""
        markers = {0: (8.0, 4.0), 9: (12.0, 8.0)}
        result = run_mission(
            markers,
            max_seconds=500.0,
            use_lookahead=True,
            sweep_row_step=2,
            side_blind_ids=frozenset({9}),
        )
        assert set(result.records) == {0, 9}
        assert result.records[9] == (12.0, 8.0)
        assert result.fsm.context.sweep_fallback_done
        assert result.final_state is StateName.LAND
