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
from math import hypot
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
    StateName.LINE_FOLLOW: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=False,
        cruise_vx=0.5,
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
    waypoint_hover_seconds: float = 1.5
    waypoint_arrival_dist: float = 0.5     # m, distance to the current target node
    return_arrival_dist: float = 0.3
    snap_max_err: float = 2.0              # m, beyond this snap is refused

    # Mission progress
    records: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    retrieval_path: List["Node"] = field(default_factory=list)
    retrieval_idx: int = 0
    start_xy: Optional[Tuple[float, float]] = None
    start_yaw: Optional[float] = None     # captured on first tick for yaw-lock

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
            self._tick_arrange(dr_state)

        elif self._state is StateName.RETURN_PATH:
            self._tick_return(dr_state)

        # LAND is terminal; no transition logic.

        target_xy = self._current_target_xy()

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

    def _tick_arrange(self, dr_state: State) -> None:
        ctx = self._context
        if not ctx.retrieval_path or ctx.grid is None:
            self._state = StateName.LAND
            return
        if ctx.retrieval_idx >= len(ctx.retrieval_path):
            self._state = StateName.LAND
            return

        target_node = ctx.retrieval_path[ctx.retrieval_idx]
        tx, ty = ctx.grid.world(target_node)
        if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.waypoint_arrival_dist:
            ctx.retrieval_idx += 1
            if ctx.retrieval_idx >= len(ctx.retrieval_path):
                self._state = StateName.LAND
            elif ctx.retrieval_idx == len(ctx.retrieval_path) - 1:
                # Last node is the start coordinate -> mark final leg.
                self._state = StateName.RETURN_PATH

    def _tick_return(self, dr_state: State) -> None:
        ctx = self._context
        if not ctx.retrieval_path or ctx.grid is None:
            self._state = StateName.LAND
            return
        if ctx.retrieval_idx >= len(ctx.retrieval_path):
            self._state = StateName.LAND
            return

        target_node = ctx.retrieval_path[ctx.retrieval_idx]
        tx, ty = ctx.grid.world(target_node)
        if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.return_arrival_dist:
            self._state = StateName.LAND

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

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
        self._state = StateName.ARRANGE_BY_ID

    def _current_target_xy(self) -> Optional[Tuple[float, float]]:
        ctx = self._context
        if self._state not in (StateName.ARRANGE_BY_ID, StateName.RETURN_PATH):
            return None
        if not ctx.retrieval_path or ctx.grid is None:
            return None
        if ctx.retrieval_idx >= len(ctx.retrieval_path):
            return None
        return ctx.grid.world(ctx.retrieval_path[ctx.retrieval_idx])
