"""Mission FSM for line_tracer (pure-Python, no rclpy).

Holds the current high-level state and exposes a per-state ``Behavior``
which the node consults each control tick.

Active behaviors  : TAKEOFF, LINE_FOLLOW, LAND.
Stub behaviors    : WAYPOINT_VISIT, ARRANGE_BY_ID, RETURN_PATH (treated
                    identically to LINE_FOLLOW for now — the perception+DR
                    loop runs, but no waypoint scheduling logic exists yet).
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Dict, Optional


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


_DEFAULT_TARGET_ALT = 2.0


_BEHAVIORS: Dict[StateName, Behavior] = {
    # Climb to altitude before engaging any line-tracking corrections.
    StateName.TAKEOFF: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=False,
        use_heading_error=False,
        use_forward_error=False,
        cruise_vx=0.0,
    ),
    # Active line tracing: lateral + heading corrections + a small cruise.
    StateName.LINE_FOLLOW: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=True,
        cruise_vx=0.4,
    ),
    # Stubs that behave like LINE_FOLLOW until their planners are written.
    StateName.WAYPOINT_VISIT: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=True,
        cruise_vx=0.4,
    ),
    StateName.ARRANGE_BY_ID: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=True,
        cruise_vx=0.4,
    ),
    StateName.RETURN_PATH: Behavior(
        target_altitude=_DEFAULT_TARGET_ALT,
        use_lateral_error=True,
        use_heading_error=True,
        use_forward_error=True,
        cruise_vx=0.4,
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


class StateMachine:
    """Holds the current ``StateName`` and dispatches its ``Behavior``.

    Transitions are unrestricted by design: the FSM only validates that the
    requested name is known. Operational guards (e.g. forbid LAND→TAKEOFF
    while in flight) belong to the node layer once we have real telemetry.
    """

    def __init__(
        self,
        initial: StateName = StateName.TAKEOFF,
        target_altitude: Optional[float] = None,
    ) -> None:
        self._state = initial
        self._behaviors = dict(_BEHAVIORS)
        if target_altitude is not None:
            for s, b in self._behaviors.items():
                if s is StateName.LAND:
                    continue
                self._behaviors[s] = replace(b, target_altitude=target_altitude)

    @property
    def state(self) -> StateName:
        return self._state

    def behavior(self) -> Behavior:
        return self._behaviors[self._state]

    def set_state(self, raw: str) -> StateName:
        """Resolve and apply a transition. Returns the new state."""
        new_state = StateName.parse(raw)
        self._state = new_state
        return new_state
