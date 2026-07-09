"""Mission FSM for line_tracer (pure-Python, no rclpy).

Holds the high-level state, exposes the per-state ``Behavior``, and runs an
automaton that walks the 7-step competition mission:

  1. TAKEOFF       — climb open-loop to target altitude.
  2. LINE_FOLLOW   — perception-driven grid-line tracking + ArUco capture.
  3. GOTO_CANDIDATE — fly to an intersection the sideways lookahead
                     camera voted a marker onto. Never records anything
                     itself: on arrival the downward camera sees the
                     marker and the normal WAYPOINT_VISIT machinery
                     takes over; if it doesn't within
                     candidate_wait_seconds, the candidate is dropped
                     and the sweep resumes where it left off.
  4. WAYPOINT_VISIT — brief hover above a freshly-seen marker; record its
                     XY by snapping the dead-reckoned position to the
                     nearest grid intersection (markers are known to sit
                     on intersections, so a sighting is an absolute fix).
  5. ARRANGE_BY_ID — once all 4 markers are captured, plan a BFS path
                     visiting them in ascending ID order, then walk it
                     node-by-node.
  6. RETURN_PATH   — final leg of that plan, heading back to the start
                     coordinates captured at takeoff.
  7. LAND          — descend in place.

Candidates (fed per tick via ``tick(candidates=...)`` from the node's
``CandidateTracker``) also reshape the search itself: with
``sweep_row_step=2`` the serpentine only flies every other interior row
— the side camera observes the skipped row from 3 m away — and the
sweep short-circuits into candidate visits as soon as
records + candidates account for every marker. A one-shot fallback
sweep over the skipped rows covers missed side detections.

WHERE a candidate gets collected is a scheduling decision, not a
reflex. The sweep alternates row traversals (legs) with row-to-row
transits, and a tour can only be spliced into a transit — splicing into
a leg would cut that leg's downward coverage short. So the transits are
the insertion points, and ``_candidates_to_tour_now`` picks among them:
a candidate the sweep will fly over anyway is never toured, and the
others are collected at the transit whose detour is smallest. See the
``MissionContext`` visit-policy knobs.

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
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Set, Tuple

from .dead_reckoning import State, snap_to_intersection

if TYPE_CHECKING:
    from .grid import Grid, Node
    from .perception import PerceptionResult
    from .side_camera import Candidate


class StateName(Enum):
    TAKEOFF = "TAKEOFF"
    LINE_FOLLOW = "LINE_FOLLOW"
    GOTO_CANDIDATE = "GOTO_CANDIDATE"
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
    # Fly to a lookahead candidate's intersection. Same world-target
    # navigation shape as ARRANGE_BY_ID; the downward camera stays
    # live via the LINE_FOLLOW-equivalent detection check in
    # _tick_goto_candidate, not via perception-driven steering.
    StateName.GOTO_CANDIDATE: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=False,
        use_heading_error=False,
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


# ---------------------------------------------------------------------------
# Route geometry — the cost model behind the candidate visit policy.
# ---------------------------------------------------------------------------

_XY = Tuple[float, float]


def _point_to_segment(p: _XY, a: _XY, b: _XY) -> float:
    """Shortest distance from ``p`` to the segment ``a``-``b``."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return hypot(px - (ax + t * dx), py - (ay + t * dy))


def _nn_chain_length(start: _XY, points: Sequence[_XY], end: _XY) -> float:
    """Length of the greedy nearest-neighbor chain start -> points -> end.

    Greedy is within pennies of optimal at the <=4 nodes a mission ever
    carries, and it is the order ``_enqueue_pending_candidates`` actually
    flies — so the cost the scheduler compares is the cost the drone pays.
    """
    total = 0.0
    px, py = start
    remaining = list(points)
    while remaining:
        nxt = min(remaining, key=lambda q: hypot(q[0] - px, q[1] - py))
        total += hypot(nxt[0] - px, nxt[1] - py)
        px, py = nxt
        remaining.remove(nxt)
    return total + hypot(end[0] - px, end[1] - py)


def _detour_cost(start: _XY, points: Sequence[_XY], end: _XY) -> float:
    """Extra distance a start -> points -> end tour costs over start -> end."""
    if not points:
        return 0.0
    return _nn_chain_length(start, points, end) - hypot(
        end[0] - start[0], end[1] - start[1]
    )


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
    # Row-skip: with the sideways lookahead camera observing the row
    # 4 m to +Y, the serpentine only needs every row_step-th interior
    # row. Requires the distance-sorted rows to ASCEND in y (camera
    # faces +Y = the unexplored side only for an ascending sweep);
    # _plan_sweep falls back to step 1 otherwise. The complement rows
    # are kept for the one-shot exhaustion fallback.
    sweep_row_step: int = 1
    sweep_skipped_rows: List[float] = field(default_factory=list)
    sweep_fallback_done: bool = False

    # Lookahead candidates: id -> Candidate (side_camera.Candidate duck
    # type: .node, .xy, .votes, .last_seen, .best_range). Refreshed each
    # tick from the tracker snapshot, minus recorded and dropped ids —
    # ALL dedup filtering lives here because the FSM owns records and
    # the drop decisions; the tracker only accumulates votes.
    candidates: Dict[int, "Candidate"] = field(default_factory=dict)
    dropped_candidate_ids: Set[int] = field(default_factory=set)
    # Visit order for GOTO_CANDIDATE; queue[0] is the active target and
    # stays queued until resolved (recorded -> purged, or dropped), so
    # a WAYPOINT_VISIT interrupt en route resumes the same target.
    candidate_queue: List[int] = field(default_factory=list)
    goto_start_t: Optional[float] = None    # per-attempt stall guard
    goto_arrive_t: Optional[float] = None   # arrival dwell before drop
    candidate_wait_seconds: float = 4.0
    goto_timeout: float = 60.0

    # Visit policy — which row-end flushes are worth interrupting the
    # sweep for. Zero radius + defer=False reproduces the r70 behavior
    # (flush every row end, every candidate) for A/B runs.
    #
    # Coverage: a candidate inside the downward camera's corridor of a
    # leg the drone has NOT flown yet is recorded for free when that leg
    # runs, so touring it is pure detour. r70 flew 37 m to visit id17 at
    # (21, 15) and then swept the row-15 leg straight over it. The radius
    # is half the downward footprint (~2.7 m at 2 m altitude) shrunk for
    # DR error; the next row is 3 m out, so the value is not delicate.
    candidate_coverage_radius: float = 1.0
    # Cheapest insertion: a tour may only be spliced into a transit (a
    # leg spliced mid-way loses the rest of its coverage), and the row-end
    # flush points ARE the transits — so the only choice is which one.
    # Flush here iff no later transit collects the same set for less.
    # r70 paid a 42.7 m detour flushing the row-6 pair at the row-3 east
    # end; the row-9 -> row-15 transit would have cost 10.0 m. Deferring
    # never delays the short-circuit (candidates count toward
    # max_records exactly as records do) and a wrong candidate stays
    # time-bounded either way — GOTO drops it after candidate_wait.
    defer_flush_to_cheapest: bool = True

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
        candidates: Optional[Dict[int, "Candidate"]] = None,
    ) -> TickResult:
        ctx = self._context
        prev_state = self._state
        snapped_record: Optional[Tuple[int, float, float]] = None

        if ctx.start_xy is None:
            ctx.start_xy = (dr_state.x, dr_state.y)
        if ctx.start_yaw is None:
            ctx.start_yaw = dr_state.yaw

        # Candidate dedup, the FSM-owned half: a tracker snapshot always
        # re-includes every id it ever voted (it has no mission state),
        # so recorded ids (already visited) and dropped ids (given up
        # at the node) are filtered here every tick. candidates=None
        # (callers without a lookahead camera / legacy tests) keeps the
        # existing dict but still purges newly resolved ids.
        fresh = candidates if candidates is not None else ctx.candidates
        ctx.candidates = {
            i: c
            for i, c in fresh.items()
            if i not in ctx.records and i not in ctx.dropped_candidate_ids
        }
        ctx.candidate_queue = [
            i for i in ctx.candidate_queue if i in ctx.candidates
        ]

        if self._state is StateName.TAKEOFF:
            self._tick_takeoff(altitude)

        elif self._state is StateName.LINE_FOLLOW:
            self._tick_line_follow(now, perception, dr_state)

        elif self._state is StateName.GOTO_CANDIDATE:
            self._tick_goto_candidate(now, perception, dr_state)

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

        # Short-circuit: every marker is accounted for (recorded or
        # believed at a voted intersection) — stop paying for blind
        # sweep legs and go collect the candidates. A wrong candidate
        # is time-bounded, not fatal: GOTO drops it after the arrival
        # wait and the sweep resumes at the same sweep_idx.
        if (
            ctx.grid is not None
            and ctx.candidates
            and len(ctx.records) + len(ctx.candidates) >= ctx.max_records
        ):
            self._enqueue_pending_candidates(dr_state)
            if ctx.candidate_queue:
                ctx.goto_start_t = None
                ctx.goto_arrive_t = None
                self._state = StateName.GOTO_CANDIDATE
                return

        # Queue drain: candidates enqueued earlier (row-finish flush /
        # short-circuit) are visited before any further sweeping; this
        # is also how the chain resumes after each WAYPOINT_VISIT
        # bounces back to LINE_FOLLOW.
        if ctx.candidate_queue:
            ctx.goto_start_t = None
            ctx.goto_arrive_t = None
            self._state = StateName.GOTO_CANDIDATE
            return

        # Serpentine search over the grid rows. Markers sit on grid
        # intersections, so cruising each interior row scans every
        # intersection on it; the +X-only cruise of r38..r55 could only
        # ever find markers on the start row.
        if ctx.grid is not None and not ctx.sweep_path:
            self._plan_sweep()
        if ctx.sweep_path:
            if ctx.sweep_idx >= len(ctx.sweep_path):
                # Reachable on the tick after GOTO_CANDIDATE returns
                # with the sweep already exhausted; without this the
                # gridless moving-lookahead fallback below would cruise
                # the drone into the wall.
                self._on_sweep_exhausted(dr_state)
                return
            tx, ty = ctx.sweep_path[ctx.sweep_idx]
            if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.sweep_arrival_dist:
                row_y = ty
                ctx.sweep_idx += 1
                if ctx.sweep_idx >= len(ctx.sweep_path):
                    # Sweep exhausted without filling the records:
                    # retrieve what we have rather than cruising off
                    # into the wall or hovering forever.
                    self._on_sweep_exhausted(dr_state)
                    return
                # Row-finish flush: the next waypoint starts a new row, so
                # the drone stands at a transit — the one place a
                # candidate tour splices in without truncating a leg. Take
                # it only if the visit policy says this transit is the
                # cheapest one left and the sweep will not fly over the
                # candidates on its own.
                if ctx.sweep_path[ctx.sweep_idx][1] != row_y and ctx.candidates:
                    tour = self._candidates_to_tour_now(dr_state)
                    if tour:
                        self._enqueue_pending_candidates(dr_state, ids=tour)
                        if ctx.candidate_queue:
                            ctx.goto_start_t = None
                            ctx.goto_arrive_t = None
                            self._state = StateName.GOTO_CANDIDATE

    def _tick_goto_candidate(
        self, now: float, perception: Optional["PerceptionResult"], dr_state: State
    ) -> None:
        ctx = self._context

        # The downward camera stays authoritative: ANY unrecorded marker
        # in view (the target or one crossed en route) goes through the
        # normal hover-and-snap recording. The queue is untouched — if
        # what got recorded was not queue[0], the same target resumes
        # after WAYPOINT_VISIT returns to LINE_FOLLOW.
        if perception is not None:
            for det in perception.aruco:
                if det.id not in ctx.records:
                    ctx.waypoint_visit_id = det.id
                    ctx.waypoint_visit_start_t = now
                    ctx.goto_start_t = None
                    ctx.goto_arrive_t = None
                    self._state = StateName.WAYPOINT_VISIT
                    return

        # Target resolved (recorded en route) or invalidated (vote moved
        # and the id fell out of the snapshot) — back to LINE_FOLLOW,
        # which drains the rest of the queue or resumes the sweep.
        if not ctx.candidate_queue:
            ctx.goto_start_t = None
            ctx.goto_arrive_t = None
            self._state = StateName.LINE_FOLLOW
            return

        cid = ctx.candidate_queue[0]
        cand = ctx.candidates[cid]

        if ctx.goto_start_t is None:
            ctx.goto_start_t = now
        if (now - ctx.goto_start_t) >= ctx.goto_timeout:
            # Stall guard, counterpart of arrange_timeout: never orbit a
            # candidate forever.
            self._drop_candidate(cid)
            self._state = StateName.LINE_FOLLOW
            return

        tx, ty = cand.xy
        if hypot(tx - dr_state.x, ty - dr_state.y) < ctx.waypoint_arrival_dist:
            if ctx.goto_arrive_t is None:
                ctx.goto_arrive_t = now
            elif (now - ctx.goto_arrive_t) >= ctx.candidate_wait_seconds:
                # Hovered over the voted intersection and the downward
                # camera never fired: the candidate was wrong. Drop it
                # (never re-promoted) and resume the search; a skipped-
                # row fallback sweep still covers the real marker.
                self._drop_candidate(cid)
                self._state = StateName.LINE_FOLLOW

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
            # Recorded beats candidate immediately (don't wait for the
            # next tick-top filter): the id leaves the candidate pool
            # and the visit queue in the same tick the record lands.
            ctx.candidates.pop(wp_id, None)
            ctx.candidate_queue = [
                i for i in ctx.candidate_queue if i != wp_id
            ]

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

    def _plan_sweep(
        self,
        rows_override: Optional[List[float]] = None,
        from_x: Optional[float] = None,
    ) -> None:
        """Serpentine (lawnmower) waypoints over the grid's interior
        rows, starting from the row nearest start_y and moving away
        from it. Rows are the grid's horizontal lines (markers only
        sit on intersections, so scanning each row with the downward
        camera sees every candidate on it). X endpoints are inset by
        sweep_margin from the arena border.

        With sweep_row_step > 1 only every step-th row is flown — the
        sideways lookahead camera observes each skipped row from the
        flown one 4 m away. The skip is only valid when the sorted rows
        ASCEND in y: the camera faces body +Y (world +Y under the yaw
        lock), which is the unexplored side of an ascending sweep only.
        A descending or oscillating order (spawn between rows) reverts
        to step 1 rather than sweeping with an eye on visited ground.

        rows_override bypasses row selection entirely — used by the
        exhaustion fallback to fly the previously skipped rows. That
        path also passes from_x (the drone's current x) so the plan
        routes to the NEAR endpoint of the first row before traversing:
        the original plan skips this because the spawn sits at the
        x_min inset by construction, but a fallback starts wherever the
        skip sweep ended — diving diagonally at the far endpoint would
        leave the first row's near half outside the downward camera."""
        ctx = self._context
        if ctx.grid is None:
            return
        x_min = ctx.sweep_margin
        x_max = ctx.grid.width - ctx.sweep_margin
        if rows_override is not None:
            rows = list(rows_override)
            ctx.sweep_skipped_rows = []
        else:
            rows = [y for y in ctx.grid.ys if 0.0 < y < ctx.grid.depth]
            if not rows:
                return
            start_y = ctx.start_xy[1] if ctx.start_xy is not None else rows[0]
            rows.sort(key=lambda y: abs(y - start_y))
            ctx.sweep_skipped_rows = []
            if ctx.sweep_row_step > 1:
                ascending = all(
                    rows[i] < rows[i + 1] for i in range(len(rows) - 1)
                )
                if ascending:
                    kept = rows[:: ctx.sweep_row_step]
                    ctx.sweep_skipped_rows = [y for y in rows if y not in kept]
                    rows = kept
        if not rows:
            return
        path: List[Tuple[float, float]] = []
        going_right = True
        if from_x is not None:
            # Route to the first row's near endpoint before traversing.
            going_right = abs(from_x - x_min) <= abs(from_x - x_max)
            path.append((x_min if going_right else x_max, rows[0]))
        for y in rows:
            if path and path[-1][1] != y:
                # Climb to the next row at the current x endpoint.
                path.append((path[-1][0], y))
            path.append((x_max if going_right else x_min, y))
            going_right = not going_right
        ctx.sweep_path = path
        ctx.sweep_idx = 0

    def _on_sweep_exhausted(self, dr_state: State) -> None:
        """Sweep ran out before all records landed. Priority: collect
        believed candidates; then a ONE-SHOT sweep over the rows the
        row-skip left out (a missed side detection is invisible to the
        candidate path — this is its safety net); finally retrieve
        whatever was recorded rather than cruising off the arena."""
        ctx = self._context
        if ctx.candidates:
            self._enqueue_pending_candidates(dr_state)
            if ctx.candidate_queue:
                ctx.goto_start_t = None
                ctx.goto_arrive_t = None
                self._state = StateName.GOTO_CANDIDATE
                return
        if (
            len(ctx.records) < ctx.max_records
            and ctx.sweep_skipped_rows
            and not ctx.sweep_fallback_done
        ):
            ctx.sweep_fallback_done = True
            self._plan_sweep(
                rows_override=list(ctx.sweep_skipped_rows),
                from_x=dr_state.x,
            )
            return
        self._plan_retrieval(dr_state)

    def _future_legs(self) -> List[Tuple[_XY, _XY]]:
        """Sweep segments still ahead that actually SCAN ground.

        ``_plan_sweep`` alternates row traversals (constant y — the legs,
        where the downward camera sees every intersection it passes) with
        transits (constant x — the climb to the next row, which scans
        nothing new). Only legs count as coverage.
        """
        ctx = self._context
        path = ctx.sweep_path
        return [
            (path[j], path[j + 1])
            for j in range(ctx.sweep_idx, len(path) - 1)
            if path[j][1] == path[j + 1][1] and path[j][0] != path[j + 1][0]
        ]

    def _future_flush_points(self) -> List[Tuple[_XY, _XY]]:
        """(tour start, rejoin) for every row-end transit still ahead.

        Callers reach this with ``sweep_idx`` already advanced past the
        transit the drone is standing on, so the CURRENT flush point is
        excluded by construction. The last flush point therefore sees an
        empty list and always fires — nothing can strand a candidate.
        """
        ctx = self._context
        path = ctx.sweep_path
        return [
            (path[j], path[j + 1])
            for j in range(ctx.sweep_idx, len(path) - 1)
            if path[j][1] != path[j + 1][1]
        ]

    def _covered_by_future_leg(self, xy: _XY) -> bool:
        ctx = self._context
        if ctx.candidate_coverage_radius <= 0.0:
            return False
        return any(
            _point_to_segment(xy, a, b) <= ctx.candidate_coverage_radius
            for a, b in self._future_legs()
        )

    def _candidates_to_tour_now(self, dr_state: State) -> List[int]:
        """Candidate ids worth interrupting the sweep for at this row end.

        Empty means keep sweeping: either the remaining legs already fly
        over every candidate, or a later transit collects the same set
        for a smaller detour. A candidate deferred here is re-considered
        at the next flush point, and ``_on_sweep_exhausted`` collects
        whatever is left unconditionally.
        """
        ctx = self._context
        pending = [
            i
            for i in ctx.candidates
            if i not in ctx.candidate_queue
            and not self._covered_by_future_leg(ctx.candidates[i].xy)
        ]
        if not pending or not ctx.defer_flush_to_cheapest:
            return pending
        pts = [ctx.candidates[i].xy for i in pending]
        cost_now = _detour_cost(
            (dr_state.x, dr_state.y), pts, ctx.sweep_path[ctx.sweep_idx]
        )
        if any(
            _detour_cost(start, pts, end) < cost_now
            for start, end in self._future_flush_points()
        ):
            return []
        return pending

    def _enqueue_pending_candidates(
        self, dr_state: State, ids: Optional[Sequence[int]] = None
    ) -> None:
        """Append every un-queued candidate in nearest-neighbor chain
        order from the current position (<=4 nodes; greedy chaining is
        within pennies of optimal at that size).

        ``ids`` restricts the set to what the row-end visit policy chose.
        Short-circuit and sweep exhaustion pass None — the sweep is over,
        so every candidate has to be flown to regardless.
        """
        ctx = self._context
        allowed = None if ids is None else set(ids)
        pending = [
            i
            for i in ctx.candidates
            if i not in ctx.candidate_queue and (allowed is None or i in allowed)
        ]
        px, py = dr_state.x, dr_state.y
        while pending:
            nxt = min(
                pending,
                key=lambda i: hypot(
                    ctx.candidates[i].xy[0] - px, ctx.candidates[i].xy[1] - py
                ),
            )
            ctx.candidate_queue.append(nxt)
            px, py = ctx.candidates[nxt].xy
            pending.remove(nxt)

    def _drop_candidate(self, cid: int) -> None:
        """Give up on a candidate id: never re-promoted (the tracker
        would keep re-snapshotting it otherwise) and cleared from the
        queue + goto timers."""
        ctx = self._context
        ctx.dropped_candidate_ids.add(cid)
        ctx.candidates.pop(cid, None)
        ctx.candidate_queue = [i for i in ctx.candidate_queue if i != cid]
        ctx.goto_start_t = None
        ctx.goto_arrive_t = None

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
        if self._state is StateName.GOTO_CANDIDATE:
            # Live lookup (not a copy captured at transition): if votes
            # move the winning node while en route, the target follows.
            # Hover-in-place during the arrival wait falls out of the
            # target staying pinned to the node.
            if ctx.candidate_queue:
                cand = ctx.candidates.get(ctx.candidate_queue[0])
                if cand is not None:
                    return cand.xy
            return None
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
