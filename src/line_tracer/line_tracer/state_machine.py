"""Mission FSM for line_tracer (pure-Python, no rclpy).

Holds the high-level state, exposes the per-state ``Behavior``, and runs an
automaton that walks the 7-step competition mission:

  1. TAKEOFF       — climb open-loop to target altitude.
  2. LINE_FOLLOW   — perception-driven grid-line tracking + ArUco capture.
  3. WAYPOINT_VISIT — brief hover above a freshly-seen marker; record its
                     XY by snapping the dead-reckoned position to the
                     nearest grid intersection (markers are known to sit
                     on intersections, so a sighting is an absolute fix).
  4. ARRANGE_BY_ID — once all 4 markers are captured, plan a BFS path
                     visiting them in ascending ID order, then walk it
                     node-by-node.
  5. RETURN_PATH   — final leg of that plan, heading back to the start
                     coordinates captured at takeoff.
  6. LAND          — descend in place.

The 1-7 split is rules-aligned (시작 -> 라인트레이싱 -> 식별 -> 구조 경로 (역순)
-> 자동 착륙). The proposal's particle-filter / vertiport / KF-z elements
are deliberately *not* implemented here — see the M-A plan note in
docs/progress/line_tracer.md.

The ``tick()`` method is the heart of the automaton; ``set_state()`` is
kept as an external override hook (test handle + /line_tracer/set_state
service) but ``tick()`` will overwrite anything ``set_state`` did on the
next call.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from math import cos, hypot, sin
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from .dead_reckoning import State, snap_to_intersection

if TYPE_CHECKING:
    from .grid import Grid, Node
    from .perception import PerceptionResult


class StateName(Enum):
    TAKEOFF = "TAKEOFF"
    LINE_FOLLOW = "LINE_FOLLOW"
    WAYPOINT_VISIT = "WAYPOINT_VISIT"
    ARRANGE_BY_ID = "ARRANGE_BY_ID"
    RETURN_PATH = "RETURN_PATH"
    LAND = "LAND"

    @classmethod
    def parse(cls, raw: str) -> "StateName":
        """Case-insensitive parse. Raises ``ValueError`` on unknown name."""
        if raw is None:
            raise ValueError("state name is None")
        key = raw.strip().upper()
        try:
            return cls[key]
        except KeyError as exc:
            raise ValueError(f"unknown state name: {raw!r}") from exc


@dataclass(frozen=True)
class Behavior:
    """How a given state shapes the control loop."""

    target_altitude: float           # meters; dead_reckoning vz drives toward this
    use_lateral_error: bool          # honor du from perception
    use_heading_error: bool          # honor psi_err from perception
    use_forward_error: bool          # honor dv from perception
    cruise_vx: float = 0.0           # body +x demand when no forward error in use
    # When True the node drives yaw back to MissionContext.start_yaw if
    # perception is not providing a fresh psi_err. Without this the
    # firmware's residual mixer/quat-sign asymmetry steadily yaws the
    # drone during cruise (see r15: ~20° drift -> cruise_vx in body +X
    # becomes (+X, -Y) in world). use_heading_error from perception
    # takes precedence when active.
    lock_yaw_to_initial: bool = False


_DEFAULT_TARGET_ALT = 2.0


_BEHAVIORS: Dict[StateName, Behavior] = {
    # Climb to altitude before engaging any line-tracking corrections.
    # lock_yaw_to_initial intentionally OFF during TAKEOFF: while the
    # drone is in ground contact, a saturated yawrate_sp from the lock
    # interacts with the sphere body_collision and the firmware mixer
    # in ways that prevent vertical liftoff (r19 spun in place on the
    # ground at thrust_burst=0.85 with wz=-1.0). LINE_FOLLOW engages
    # the lock once airborne.
    StateName.TAKEOFF: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=False,
        use_heading_error=False,
        use_forward_error=False,
        cruise_vx=0.0,
        lock_yaw_to_initial=False,
    ),
    # Active line tracing: lateral (du) + heading correction keep the drone
    # on the closest vertical line; forward error (dv) is intentionally OFF
    # because grid lines are crossings to fly THROUGH, not align with — the
    # drone advances at cruise_vx, not by snapping to the next horizontal.
    # cruise_vx is small (0.2) because line_tracer has no body-velocity
    # feedback — pitch_sp = +cruise_vx / g becomes a constant tilt with
    # no drag in the sim, so the drone accelerates without bound. 0.2
    # gives ~0.02 rad pitch and the drone reaches ~2 m/s over ~10 s,
    # slow enough to catch each marker on the +X line before yaw drift
    # curls the track off-axis.
    StateName.LINE_FOLLOW: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=False,
        cruise_vx=0.2,
        lock_yaw_to_initial=True,
    ),
    # Hover above a marker for the snap recording; no forward cruise so the
    # drone doesn't drift off the intersection while we read it.
    StateName.WAYPOINT_VISIT: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=False,
        cruise_vx=0.0,
        lock_yaw_to_initial=True,
    ),
    # Walk a precomputed grid path. Perception is ignored — the node
    # consumes the FSM's target_xy_world instead.
    StateName.ARRANGE_BY_ID: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=False,
        use_heading_error=False,
        use_forward_error=False,
        cruise_vx=0.0,
        lock_yaw_to_initial=True,
    ),
    # Final leg back to the start coordinates; same handling as ARRANGE.
    StateName.RETURN_PATH: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=False,
        use_heading_error=False,
        use_forward_error=False,
        cruise_vx=0.0,
        lock_yaw_to_initial=True,
    ),
    # Descend; ignore perception (don't chase a line on the way down).
    StateName.LAND: Behavior(
        target_altitude=0.0,
        use_lateral_error=False,
        use_heading_error=False,
        use_forward_error=False,
        cruise_vx=0.0,
    ),
}


@dataclass
class MissionContext:
    """Persistent state across tick() calls — what the automaton remembers.

    All thresholds are tunable knobs the node may override at construction.
    Counters / records mutate in place as the mission progresses.
    """
    # Static configuration / dependencies
    grid: Optional["Grid"] = None
    max_records: int = 4
    takeoff_alt_threshold: float = 1.8     # m, altitude that counts as airborne
    takeoff_streak_required: int = 10      # consecutive ticks above threshold
    # 3 s: long enough to visually confirm the detection overlay on
    # /line_tracer/debug_image while parked over the marker (and for a
    # multi-frame ID vote later, M-E); the velocity loop brakes the
    # 0.5 m/s cruise in ~1 s, so the drone truly stands still for the
    # remainder.
    waypoint_hover_seconds: float = 3.0
    waypoint_arrival_dist: float = 0.5     # m, distance to the current target node
    return_arrival_dist: float = 0.3
    snap_max_err: float = 2.0              # m, beyond this snap is refused

    # Mission progress
    records: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    retrieval_path: List["Node"] = field(default_factory=list)
    retrieval_idx: int = 0
    start_xy: Optional[Tuple[float, float]] = None
    start_yaw: Optional[float] = None     # captured on first tick for yaw-lock
    return_path_start_t: Optional[float] = None
    return_path_timeout: float = 30.0     # force LAND if return doesn't arrive in time
    # ARRANGE_BY_ID stall guard: node advancement requires a tick to
    # sample the drone inside waypoint_arrival_dist of the current
    # node; an overshoot can orbit a node forever with no counterpart
    # to return_path_timeout. On timeout, fall through to RETURN_PATH
    # (still try to get home) rather than LAND-in-place.
    arrange_start_t: Optional[float] = None
    arrange_timeout: float = 60.0
    # LINE_FOLLOW without a grid falls back to a MOVING lookahead
    # target (drone_x + lookahead, start_y). The lookahead sets how
    # hard the velocity vector leans into cross-track error: with a
    # fixed far target the direction was ~unit +X, so a 1 m takeoff
    # drift decayed over tens of metres and the drone cruised past the
    # marker row outside the camera footprint (r42). 2.0 m makes a 1 m
    # cross error steer ~27 degrees toward the line while advancing.
    line_follow_lookahead: float = 2.0
    # Serpentine search plan over the grid's interior rows (generated
    # by _plan_sweep on the first LINE_FOLLOW tick with a grid).
    # sweep_arrival_dist is looser than waypoint_arrival_dist: sweep
    # corners are route shaping, not scored positions.
    sweep_path: List[Tuple[float, float]] = field(default_factory=list)
    sweep_idx: int = 0
    sweep_arrival_dist: float = 1.5
    sweep_margin: float = 2.0     # [m] inset from the arena border

    # Transition counters
    takeoff_alt_streak: int = 0
    waypoint_visit_id: Optional[int] = None
    waypoint_visit_start_t: Optional[float] = None


@dataclass(frozen=True)
class TickResult:
    """Per-tick output the node consumes after calling :meth:`tick`."""
    state: StateName
    behavior: Behavior
    target_xy_world: Optional[Tuple[float, float]] = None
    state_changed: bool = False
    snapped_record: Optional[Tuple[int, float, float]] = None


class StateMachine:
    """Owns the current ``StateName`` + ``MissionContext`` and runs ``tick``.

    Transitions are guarded by ``tick(now, dr_state, perception, altitude)``;
    ``set_state(raw)`` remains as an external override (test handle +
    /line_tracer/set_state service) but the next ``tick`` may overwrite it.
    """

    def __init__(
        self,
        initial: StateName = StateName.TAKEOFF,
        target_altitude: Optional[float] = None,
        context: Optional[MissionContext] = None,
    ) -> None:
        self._state = initial
        self._behaviors = dict(_BEHAVIORS)
        if target_altitude is not None:
            for s, b in self._behaviors.items():
                if s is StateName.LAND:
                    continue
                self._behaviors[s] = replace(b, target_altitude=target_altitude)
        self._context = context if context is not None else MissionContext()

    @property
    def state(self) -> StateName:
        return self._state

    @property
    def context(self) -> MissionContext:
        return self._context

    def behavior(self) -> Behavior:
        return self._behaviors[self._state]

    def set_state(self, raw: str) -> StateName:
        """Resolve and apply a transition. Returns the new state."""
        new_state = StateName.parse(raw)
        self._state = new_state
        return new_state

    # ------------------------------------------------------------------
    # Automaton tick
    # ------------------------------------------------------------------

    def tick(
        self,
        now: float,
        dr_state: State,
        perception: Optional["PerceptionResult"],
        altitude: float,
    ) -> TickResult:
        ctx = self._context
        prev_state = self._state
        snapped_record: Optional[Tuple[int, float, float]] = None

        if ctx.start_xy is None:
            ctx.start_xy = (dr_state.x, dr_state.y)
        if ctx.start_yaw is None:
            ctx.start_yaw = dr_state.yaw

        if self._state is StateName.TAKEOFF:
            self._tick_takeoff(altitude)

        elif self._state is StateName.LINE_FOLLOW:
            self._tick_line_follow(now, perception, dr_state)

        elif self._state is StateName.WAYPOINT_VISIT:
            snapped_record = self._tick_waypoint_visit(now, dr_state)

        elif self._state is StateName.ARRANGE_BY_ID:
            self._tick_arrange(now, dr_state)

        elif self._state is StateName.RETURN_PATH:
            self._tick_return(now, dr_state)

        # LAND is terminal; no transition logic.

        target_xy = self._current_target_xy(dr_state, altitude)

        return TickResult(
            state=self._state,
            behavior=self.behavior(),
            target_xy_world=target_xy,
            state_changed=self._state is not prev_state,
            snapped_record=snapped_record,
        )

    # ------------------------------------------------------------------
    # Per-state logic — kept small so each is one transition rule.
    # ------------------------------------------------------------------

    def _tick_takeoff(self, altitude: float) -> None:
        ctx = self._context
        if altitude >= ctx.takeoff_alt_threshold:
            ctx.takeoff_alt_streak += 1
        else:
            ctx.takeoff_alt_streak = 0
        if ctx.takeoff_alt_streak >= ctx.takeoff_streak_required:
            self._state = StateName.LINE_FOLLOW

    def _tick_line_follow(
        self, now: float, perception: Optional["PerceptionResult"], dr_state: State
    ) -> None:
        ctx = self._context

        # An unrecorded ArUco in view triggers a hover-and-record cycle.
        if perception is not None:
            for det in perception.aruco:
                if det.id not in ctx.records:
                    ctx.waypoint_visit_id = det.id
                    ctx.waypoint_visit_start_t = now
                    self._state = StateName.WAYPOINT_VISIT
                    return

        # All markers captured -> plan retrieval and switch to ARRANGE.
        if len(ctx.records) >= ctx.max_records and ctx.grid is not None:
            self._plan_retrieval(dr_state)
            return

        # Serpentine search over the grid rows. Markers sit on grid
        # intersections, so cruising each interior row scans every
        # intersection on it; the +X-only cruise of r38..r55 could only
        # ever find markers on the start row.
        if ctx.grid is not None and not ctx.sweep_path:
            self._plan_sweep()
        if ctx.sweep_path and ctx.sweep_idx < len(ctx.sweep_path):
            tx, ty = ctx.sweep_path[ctx.sweep_idx]
            if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.sweep_arrival_dist:
                ctx.sweep_idx += 1
                if ctx.sweep_idx >= len(ctx.sweep_path):
                    # Sweep exhausted without filling the records:
                    # retrieve what we have rather than cruising off
                    # into the wall or hovering forever.
                    self._plan_retrieval(dr_state)

    def _tick_waypoint_visit(
        self, now: float, dr_state: State
    ) -> Optional[Tuple[int, float, float]]:
        ctx = self._context
        snapped_record: Optional[Tuple[int, float, float]] = None

        wp_id = ctx.waypoint_visit_id
        if wp_id is not None and wp_id not in ctx.records:
            snapped = (
                snap_to_intersection(dr_state, ctx.grid, ctx.snap_max_err)
                if ctx.grid is not None
                else dr_state
            )
            ctx.records[wp_id] = (snapped.x, snapped.y)
            snapped_record = (wp_id, snapped.x, snapped.y)

        start = now if ctx.waypoint_visit_start_t is None else ctx.waypoint_visit_start_t
        if (now - start) >= ctx.waypoint_hover_seconds:
            ctx.waypoint_visit_id = None
            ctx.waypoint_visit_start_t = None
            self._state = StateName.LINE_FOLLOW
        return snapped_record

    def _tick_arrange(self, now: float, dr_state: State) -> None:
        ctx = self._context
        if not ctx.retrieval_path or ctx.grid is None:
            # Nothing was planned (mis-config) — no path to follow and
            # no basis for a return leg either.
            self._state = StateName.LAND
            return
        if ctx.arrange_start_t is None:
            ctx.arrange_start_t = now
        if (now - ctx.arrange_start_t) >= ctx.arrange_timeout:
            self._state = StateName.RETURN_PATH
            return
        # Path exhausted -> RETURN_PATH, never straight to LAND. The
        # return leg homes on the exact start_xy independently of the
        # node list, so this also covers the degenerate length-1 path:
        # when marker node == start node == current node, dedup in
        # visit_in_order collapses the plan to one node and the old
        # `if idx >= len: LAND` pre-empted the `elif idx == len-1:
        # RETURN_PATH` branch — r52 skipped the whole return leg and
        # landed on the marker.
        if ctx.retrieval_idx >= len(ctx.retrieval_path):
            self._state = StateName.RETURN_PATH
            return

        target_node = ctx.retrieval_path[ctx.retrieval_idx]
        tx, ty = ctx.grid.world(target_node)
        if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.waypoint_arrival_dist:
            ctx.retrieval_idx += 1
            if ctx.retrieval_idx >= len(ctx.retrieval_path):
                self._state = StateName.RETURN_PATH
            elif ctx.retrieval_idx == len(ctx.retrieval_path) - 1:
                # Last node is the start coordinate -> mark final leg.
                self._state = StateName.RETURN_PATH

    def _tick_return(self, now: float, dr_state: State) -> None:
        ctx = self._context
        if ctx.return_path_start_t is None:
            ctx.return_path_start_t = now
        # Timeout fallback: with no body-velocity feedback the drone
        # accumulates inertia and may never enter the return_arrival_dist
        # window. After return_path_timeout seconds, LAND regardless of
        # position so the FSM completes the demo flow. Real flight will
        # need body-velocity PD to retire this.
        if (now - ctx.return_path_start_t) >= ctx.return_path_timeout:
            self._state = StateName.LAND
            return
        if ctx.start_xy is None:
            # Nothing to home on (never captured) — land in place.
            self._state = StateName.LAND
            return

        # The return target is the EXACT start position, not its nearest
        # grid node — spawn can sit up to half a cell (2 m) from any
        # intersection, and the mission is scored on landing where the
        # drone took off (r41 landed on the (0,0) node, 4 m from spawn).
        # No retrieval_path/idx guards here: RETURN_PATH is reachable
        # with idx == len(path) (ARRANGE exhaustion) and the node list
        # is irrelevant to the homing leg.
        tx, ty = ctx.start_xy
        if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.return_arrival_dist:
            self._state = StateName.LAND

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _plan_sweep(self) -> None:
        """Serpentine (lawnmower) waypoints over the grid's interior
        rows, starting from the row nearest start_y and moving away
        from it. Rows are the grid's horizontal lines (markers only
        sit on intersections, so scanning each row with the downward
        camera sees every candidate on it). X endpoints are inset by
        sweep_margin from the arena border."""
        ctx = self._context
        if ctx.grid is None:
            return
        x_min = ctx.sweep_margin
        x_max = ctx.grid.width - ctx.sweep_margin
        rows = [y for y in ctx.grid.ys if 0.0 < y < ctx.grid.depth]
        if not rows:
            return
        start_y = ctx.start_xy[1] if ctx.start_xy is not None else rows[0]
        rows.sort(key=lambda y: abs(y - start_y))
        path: List[Tuple[float, float]] = []
        going_right = True
        for y in rows:
            if path:
                # Climb to the next row at the current x endpoint.
                path.append((path[-1][0], y))
            path.append((x_max if going_right else x_min, y))
            going_right = not going_right
        ctx.sweep_path = path
        ctx.sweep_idx = 0

    def _plan_retrieval(self, dr_state: State) -> None:
        """Build the ARRANGE_BY_ID node sequence: current node -> markers
        in ascending ID order -> start node, deduping intersections."""
        from .planner import visit_in_order   # local to avoid cycles

        ctx = self._context
        if ctx.grid is None or ctx.start_xy is None:
            return
        sorted_ids = sorted(ctx.records.keys())
        waypoints: List["Node"] = []
        for mid in sorted_ids:
            mx, my = ctx.records[mid]
            waypoints.append(ctx.grid.nearest_node(mx, my))
        waypoints.append(ctx.grid.nearest_node(*ctx.start_xy))
        cur = ctx.grid.nearest_node(dr_state.x, dr_state.y)
        ctx.retrieval_path = visit_in_order(ctx.grid, cur, waypoints)
        ctx.retrieval_idx = 0
        # Fresh mission-phase timers: without these resets a re-plan
        # (set_state or a second mission) inherits a stale
        # return_path_start_t and the 30 s timeout fires instantly.
        ctx.arrange_start_t = None
        ctx.return_path_start_t = None
        self._state = StateName.ARRANGE_BY_ID

    def _current_target_xy(
        self, dr_state: State, altitude: float
    ) -> Optional[Tuple[float, float]]:
        ctx = self._context
        if self._state is StateName.TAKEOFF:
            # Position hold over the start point while climbing —
            # but only once airborne. Commanding lateral attitude
            # during the ground-contact burst phase shoves the sphere
            # collision and flings the drone (r43: 4 m/s lateral, yaw
            # spin, 60 m runaway). Below the gate the drone stays
            # level; above it the hold brakes the takeoff drift that
            # walked r42 a metre off the marker row.
            if altitude < 0.5:
                return None
            return ctx.start_xy
        if self._state is StateName.LINE_FOLLOW:
            # Serpentine sweep waypoint when a plan exists; otherwise
            # the gridless fallback: a moving lookahead along the
            # world +X row the mission started on (markers sit on
            # +X-aligned lines; start_yaw mis-aimed r31 because spawn
            # yaw is captured a fraction off zero).
            if ctx.sweep_path and ctx.sweep_idx < len(ctx.sweep_path):
                return ctx.sweep_path[ctx.sweep_idx]
            if ctx.start_xy is None:
                return None
            sx, sy = ctx.start_xy
            return (dr_state.x + ctx.line_follow_lookahead, sy)
        if self._state not in (StateName.ARRANGE_BY_ID, StateName.RETURN_PATH):
            return None
        if self._state is StateName.RETURN_PATH and ctx.start_xy is not None:
            # Final leg homes on the exact start position (see
            # _tick_return); grid nodes are only used to route the
            # ARRANGE_BY_ID traversal.
            return ctx.start_xy
        if not ctx.retrieval_path or ctx.grid is None:
            return None
        if ctx.retrieval_idx >= len(ctx.retrieval_path):
            return None
        return ctx.grid.world(ctx.retrieval_path[ctx.retrieval_idx])
