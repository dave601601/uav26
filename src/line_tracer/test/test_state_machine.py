"""Unit tests for line_tracer.state_machine."""
import pytest

from line_tracer.state_machine import Behavior, StateMachine, StateName


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

    @pytest.mark.parametrize(
        "stub", [StateName.WAYPOINT_VISIT, StateName.ARRANGE_BY_ID, StateName.RETURN_PATH]
    )
    def test_stub_states_behave_like_line_follow(self, stub):
        sm_stub = StateMachine(initial=stub)
        sm_lf = StateMachine(initial=StateName.LINE_FOLLOW)
        # only behavior fields should match (target alt, flags, cruise);
        # state itself is obviously different
        assert sm_stub.behavior() == sm_lf.behavior()


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
