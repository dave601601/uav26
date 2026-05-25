"""Unit tests for line_tracer.state_machine."""
from dataclasses import dataclass, field
from typing import List

import pytest

from line_tracer.dead_reckoning import State
from line_tracer.grid import Grid
from line_tracer.perception import ArucoDetection, PerceptionResult
from line_tracer.state_machine import (
    Behavior,
    MissionContext,
    StateMachine,
    StateName,
    TickResult,
)


class TestStateNameParse:
    def test_canonical_uppercase(self):
        assert StateName.parse("TAKEOFF") is StateName.TAKEOFF

    def test_lowercase_works(self):
        assert StateName.parse("line_follow") is StateName.LINE_FOLLOW

    def test_mixed_case_works(self):
        assert StateName.parse("  Land  ") is StateName.LAND

    def test_unknown_raises(self):
        with pytest.raises(ValueError):
            StateName.parse("HOVER")

    def test_none_raises(self):
        with pytest.raises(ValueError):
            StateName.parse(None)


class TestBehaviorMap:
    def test_takeoff_climbs_without_corrections(self):
        sm = StateMachine(initial=StateName.TAKEOFF)
        b = sm.behavior()
        assert b.target_altitude > 0
        assert not b.use_lateral_error
        assert not b.use_heading_error
        assert not b.use_forward_error

    def test_line_follow_uses_all_corrections(self):
        sm = StateMachine(initial=StateName.LINE_FOLLOW)
        b = sm.behavior()
        assert b.target_altitude > 0
        assert b.use_lateral_error and b.use_heading_error and b.use_forward_error
        assert b.cruise_vx > 0

    def test_land_target_is_ground(self):
        sm = StateMachine(initial=StateName.LAND)
        b = sm.behavior()
        assert b.target_altitude == 0.0
        assert not b.use_lateral_error

    def test_waypoint_visit_hovers_without_forward_cruise(self):
        # WAYPOINT_VISIT must not push forward — drone stays over the marker
        # while the snap-record runs. Lateral + heading corrections stay on
        # so perception can keep the marker centered.
        b = StateMachine(initial=StateName.WAYPOINT_VISIT).behavior()
        assert b.use_lateral_error and b.use_heading_error
        assert not b.use_forward_error
        assert b.cruise_vx == 0.0

    @pytest.mark.parametrize(
        "phase", [StateName.ARRANGE_BY_ID, StateName.RETURN_PATH]
    )
    def test_retrieval_phases_ignore_perception(self, phase):
        # ARRANGE_BY_ID and RETURN_PATH navigate via the FSM's
        # target_xy_world output; perception-driven corrections are off.
        b = StateMachine(initial=phase).behavior()
        assert not b.use_lateral_error
        assert not b.use_heading_error
        assert not b.use_forward_error
        assert b.cruise_vx == 0.0


class TestTransitions:
    def test_default_initial_is_takeoff(self):
        assert StateMachine().state is StateName.TAKEOFF

    def test_set_state_transitions(self):
        sm = StateMachine()
        sm.set_state("LINE_FOLLOW")
        assert sm.state is StateName.LINE_FOLLOW
        sm.set_state("LAND")
        assert sm.state is StateName.LAND

    def test_set_state_is_case_insensitive(self):
        sm = StateMachine()
        sm.set_state("line_follow")
        assert sm.state is StateName.LINE_FOLLOW

    def test_set_state_returns_resolved_state(self):
        sm = StateMachine()
        out = sm.set_state("LAND")
        assert out is StateName.LAND

    def test_set_state_unknown_raises_and_does_not_transition(self):
        sm = StateMachine(initial=StateName.LINE_FOLLOW)
        with pytest.raises(ValueError):
            sm.set_state("FLY_AROUND")
        assert sm.state is StateName.LINE_FOLLOW


class TestParameterOverrides:
    def test_target_altitude_override_applies_to_all_but_land(self):
        sm = StateMachine(target_altitude=3.5)
        sm.set_state("TAKEOFF")
        assert sm.behavior().target_altitude == 3.5
        sm.set_state("LINE_FOLLOW")
        assert sm.behavior().target_altitude == 3.5
        sm.set_state("LAND")
        assert sm.behavior().target_altitude == 0.0


# ---------------------------------------------------------------------------
# Mission tick automaton — TAKEOFF → ... → LAND
# ---------------------------------------------------------------------------


def _empty_perception() -> PerceptionResult:
    return PerceptionResult()


def _seen(marker_id: int) -> PerceptionResult:
    """A single ArUco detection at image center (centering is irrelevant
    for the FSM; only id matters for the transition trigger)."""
    return PerceptionResult(
        aruco=[ArucoDetection(id=marker_id, center_uv=(320.0, 240.0),
                              corners_uv=tuple([(0.0, 0.0)] * 4))]
    )


class TestMissionTick:
    """Walk the full mission with synthetic perception + DR inputs.

    Markers are placed on the 4 m grid (so snap_to_intersection rounds them
    to themselves). The drone is teleported via the dr_state argument; the
    FSM's job is only to advance the state machine and emit target_xy_world
    on retrieval phases.
    """

    def _grid(self) -> Grid:
        # 30 x 20 m world, 4 m cells. Markers at four interior intersections.
        return Grid.from_extents(
            width=30.0, depth=20.0, cell=4.0,
            marker_xy={0: (8.0, 4.0), 1: (12.0, 8.0),
                       2: (20.0, 8.0), 3: (24.0, 16.0)},
        )

    def _ctx(self) -> MissionContext:
        # Shorten the takeoff streak + hover seconds so the test runs quickly.
        return MissionContext(
            grid=self._grid(),
            takeoff_streak_required=3,
            waypoint_hover_seconds=0.1,
            waypoint_arrival_dist=0.5,
        )

    def test_full_sequence_reaches_land(self):
        sm = StateMachine(initial=StateName.TAKEOFF, context=self._ctx())
        now = 0.0

        # 1) TAKEOFF: altitude climbs from 0 to 2 m. Three ticks above
        # threshold flip to LINE_FOLLOW.
        for alt in (0.5, 1.0, 1.5):
            r = sm.tick(now, State(x=2.0, y=4.0, z=alt), _empty_perception(), alt)
            assert r.state is StateName.TAKEOFF
            now += 0.05
        for alt in (1.85, 1.85, 1.85, 1.85):
            r = sm.tick(now, State(x=2.0, y=4.0, z=alt), _empty_perception(), alt)
            now += 0.05
        assert r.state is StateName.LINE_FOLLOW

        # 2) Visit each marker. Drone is teleported above the marker,
        # FSM sees a new id → WAYPOINT_VISIT, hovers briefly, records,
        # back to LINE_FOLLOW.
        for mid in (0, 1, 2, 3):
            mx, my = sm.context.grid.marker_xy[mid]
            r = sm.tick(now, State(x=mx + 0.1, y=my - 0.1, z=2.0),
                        _seen(mid), 2.0)
            assert r.state is StateName.WAYPOINT_VISIT
            # one or two more ticks elapses the hover window
            now += sm.context.waypoint_hover_seconds + 0.05
            r = sm.tick(now, State(x=mx, y=my, z=2.0),
                        _empty_perception(), 2.0)
            assert r.state is StateName.LINE_FOLLOW
            assert mid in sm.context.records

        # All four recorded -> next LINE_FOLLOW tick should plan retrieval
        # and switch to ARRANGE_BY_ID.
        r = sm.tick(now, State(x=2.0, y=4.0, z=2.0), _empty_perception(), 2.0)
        assert r.state is StateName.ARRANGE_BY_ID
        assert sm.context.retrieval_path
        # Path: start with current node, end with start_xy = (2, 4) node.
        assert sm.context.retrieval_path[-1] == sm.context.grid.nearest_node(2.0, 4.0)

        # 3) Walk the retrieval path: pretend the drone is exactly on each
        # node and let the FSM advance. Final transition is to LAND (via
        # an intermediate RETURN_PATH when the next-to-last node arrives).
        seen_states = [r.state]
        max_ticks = 60
        for _ in range(max_ticks):
            now += 0.05
            if sm.state in (StateName.ARRANGE_BY_ID, StateName.RETURN_PATH):
                tx, ty = r.target_xy_world  # type: ignore[misc]
                dr = State(x=tx, y=ty, z=2.0)
                r = sm.tick(now, dr, _empty_perception(), 2.0)
                seen_states.append(r.state)
                if r.state is StateName.LAND:
                    break
            else:
                break
        assert sm.state is StateName.LAND
        assert StateName.ARRANGE_BY_ID in seen_states
        assert StateName.RETURN_PATH in seen_states

    def test_records_snap_to_grid_intersection(self):
        sm = StateMachine(initial=StateName.LINE_FOLLOW, context=self._ctx())
        # Drone slightly off (8.0, 4.0) — within snap_max_err of intersection.
        r = sm.tick(0.0, State(x=8.3, y=3.8, z=2.0), _seen(0), 2.0)
        assert r.state is StateName.WAYPOINT_VISIT
        r = sm.tick(0.05, State(x=8.3, y=3.8, z=2.0), _empty_perception(), 2.0)
        # Marker recorded at snapped intersection, not the noisy DR pose.
        assert sm.context.records[0] == (8.0, 4.0)
        assert r.snapped_record is None or r.snapped_record[0] == 0

    def test_same_marker_seen_twice_does_not_double_record(self):
        sm = StateMachine(initial=StateName.LINE_FOLLOW, context=self._ctx())
        # First sighting at marker 0 -> WAYPOINT_VISIT, record (8,4).
        sm.tick(0.0, State(x=8.0, y=4.0, z=2.0), _seen(0), 2.0)
        sm.tick(sm.context.waypoint_hover_seconds + 0.01,
                State(x=8.0, y=4.0, z=2.0), _empty_perception(), 2.0)
        assert sm.state is StateName.LINE_FOLLOW
        assert 0 in sm.context.records
        # Seeing the same id again must NOT enter WAYPOINT_VISIT again.
        r = sm.tick(1.0, State(x=8.0, y=4.0, z=2.0), _seen(0), 2.0)
        assert r.state is StateName.LINE_FOLLOW

    def test_target_xy_world_emitted_during_retrieval(self):
        sm = StateMachine(initial=StateName.LINE_FOLLOW, context=self._ctx())
        # Pre-seed records so the next tick plans retrieval.
        sm.context.records.update({
            0: (8.0, 4.0), 1: (12.0, 8.0), 2: (20.0, 8.0), 3: (24.0, 16.0),
        })
        sm.context.start_xy = (2.0, 4.0)
        r = sm.tick(0.0, State(x=2.0, y=4.0, z=2.0), _empty_perception(), 2.0)
        assert r.state is StateName.ARRANGE_BY_ID
        assert r.target_xy_world is not None
        # First retrieval target is one of the planned grid intersections
        # (current node, marker nodes, or start node — all on the 4-m grid).
        tx, ty = r.target_xy_world
        assert (tx, ty) in {
            (0.0, 4.0), (4.0, 4.0), (8.0, 4.0), (12.0, 8.0),
            (20.0, 8.0), (24.0, 16.0),
        }
