"""Unit tests for line_tracer.mission (pure Python, no ROS)."""
from __future__ import annotations

import pytest

from line_tracer.mission import (
    ArucoDetection,
    ControlMode,
    ExplorationPlanner,
    GridMap,
    IntersectionDetection,
    LineDetection,
    McuCommand,
    MissionManager,
    MissionState,
    MoveDirection,
    Node,
    PathPlanner,
    PerceptionData,
    SensorData,
    move_to_next_node,
)


def _null(*_args) -> None:
    """Silent logger so tests do not spam stdout."""


def _perception(
    line_visible: bool = False,   # convenience: sets both presence flags
    has_vertical=None,
    has_horizontal=None,
    dx: float = 0.0,
    dy: float = 0.0,
    angle_error: float = 0.0,
    line_conf: float = 0.0,
    intersection: bool = False,
    fwd: bool = False,
    left: bool = False,
    right: bool = False,
    back: bool = False,
    aruco_id=None,
    cx: float = 0.0,
    cy: float = 0.0,
    yaw: float = 0.0,
    aruco_conf: float = 0.0,
) -> PerceptionData:
    has_v = line_visible if has_vertical is None else has_vertical
    has_h = line_visible if has_horizontal is None else has_horizontal
    return PerceptionData(
        line=LineDetection(has_v, has_h, dx, dy, angle_error, line_conf),
        intersection=IntersectionDetection(intersection, fwd, left, right, back),
        aruco=ArucoDetection(aruco_id is not None, aruco_id, cx, cy, yaw, aruco_conf),
    )


def _sensors(
    altitude: float = 2.0,
    battery: float = 15.5,
    imu_ok: bool = True,
    lidar_ok: bool = True,
    rc: bool = True,
    dr_x=None,
    dr_y=None,
    vx=None,
    vy=None,
) -> SensorData:
    return SensorData(altitude, battery, imu_ok, lidar_ok, rc, dr_x, dr_y, vx, vy)


# ---------------------------------------------------------------------------
# enums / grid geometry
# ---------------------------------------------------------------------------

def test_enum_fixed_values():
    assert int(MissionState.INIT) == 0
    assert int(MissionState.FAILSAFE) == 11
    assert int(ControlMode.HOLD) == 0
    assert int(ControlMode.EMERGENCY_LAND) == 7
    dirs = (MoveDirection.X_POS, MoveDirection.X_NEG, MoveDirection.Y_POS, MoveDirection.Y_NEG)
    assert [int(d) for d in dirs] == [0, 1, 2, 3]


def test_grid_node_world_and_nearest():
    g = GridMap(11, 8, cell_size_m=3.0, logger=_null)
    assert g.node_world(Node(0, 0)) == (0.0, 0.0)
    assert g.node_world(Node(10, 7)) == (30.0, 21.0)
    assert g.nearest_node(9.0, 0.4) == Node(3, 0)
    assert g.nearest_node(-5.0, 100.0) == Node(0, 7)   # clamped into bounds


# ---------------------------------------------------------------------------
# serpentine exploration
# ---------------------------------------------------------------------------

def test_serpentine_covers_full_11x8_grid():
    """Following the planner from (0,0) visits every node and never
    steps out of the grid."""
    grid = GridMap(11, 8, logger=_null)
    planner = ExplorationPlanner()
    node = Node(0, 0)
    direction = MoveDirection.X_POS
    visited = {node}
    total = grid.node_count_x * grid.node_count_y   # 88

    for _ in range(1000):
        nxt = move_to_next_node(node, direction)
        assert grid.in_bounds(nxt)     # the interior traversal never leaves
        node = nxt
        visited.add(node)
        if len(visited) == total:
            break
        direction = planner.choose_direction(grid, node, direction)

    assert len(visited) == total


def test_serpentine_never_leaves_grid_after_exhaustion():
    """Once the grid is swept, the planner keeps returning in-bounds
    moves (restart-and-loop until markers are found)."""
    grid = GridMap(11, 8, logger=_null)
    planner = ExplorationPlanner()
    node = Node(0, 0)
    direction = MoveDirection.X_POS
    for _ in range(600):
        nxt = move_to_next_node(node, direction)
        assert grid.in_bounds(nxt)
        node = nxt
        direction = planner.choose_direction(grid, node, direction)


# ---------------------------------------------------------------------------
# happy path INIT -> ... -> EXPLORE
# ---------------------------------------------------------------------------

def test_happy_path_init_to_explore():
    m = MissionManager(logger=_null)

    cmd = m.step(0.0, _sensors(altitude=0.0), _perception())
    assert m.state == MissionState.TAKEOFF
    assert cmd.mode == int(ControlMode.HOLD)

    cmd = m.step(0.0, _sensors(altitude=2.0), _perception())
    assert m.state == MissionState.LOCALIZE
    assert cmd.mode == int(ControlMode.ALIGN_MARKER)

    cmd = m.step(0.0, _sensors(), _perception())
    assert m.state == MissionState.ENTER_GRID
    assert cmd.mode == int(ControlMode.MOVE_TO_LANDMARK)

    cmd = m.step(0.0, _sensors(dr_x=0.0, dr_y=0.0), _perception(line_visible=True))
    assert m.state == MissionState.EXPLORE
    assert cmd.mode == int(ControlMode.SEARCH_LINE)
    assert m.current_node == Node(0, 0)
    assert m.home_node == Node(0, 0)


def test_enter_grid_snaps_home_from_dr():
    m = MissionManager(logger=_null)
    m.state = MissionState.ENTER_GRID
    m.step(0.0, _sensors(dr_x=3.1, dr_y=0.0), _perception(line_visible=True))
    assert m.state == MissionState.EXPLORE
    assert m.current_node == Node(1, 0)     # x=3 line already crossed at 3.1
    assert m.home_node == Node(1, 0)


def test_enter_grid_snap_takes_node_behind_travel():
    """r77 off-by-one: DR (2.0, 3.0) traveling X_POS had NOT crossed x=3
    yet, so the entry snap must pick (0, 1) — the line behind — and the
    first intersection pulse then lands on (1, 1), the line ahead."""
    m = MissionManager(logger=_null)
    m.state = MissionState.ENTER_GRID
    m.move_direction = MoveDirection.X_POS
    m.step(0.0, _sensors(dr_x=2.0, dr_y=3.0), _perception(line_visible=True))
    assert m.state == MissionState.EXPLORE
    assert m.current_node == Node(0, 1)
    assert m.home_node == Node(0, 1)

    m.step(1.0, _sensors(dr_x=3.0, dr_y=3.0), _perception(intersection=True))
    assert m.current_node == Node(1, 1)


def test_entry_node_direction_semantics():
    g = GridMap(11, 8, cell_size_m=3.0, logger=_null)
    # Travel axis: floor for positive travel, ceil for negative.
    assert g.entry_node(2.0, 3.0, MoveDirection.X_POS) == Node(0, 1)
    assert g.entry_node(2.0, 3.0, MoveDirection.X_NEG) == Node(1, 1)
    assert g.entry_node(3.0, 4.0, MoveDirection.Y_POS) == Node(1, 1)
    assert g.entry_node(3.0, 4.0, MoveDirection.Y_NEG) == Node(1, 2)
    # Perpendicular axis keeps nearest; results clamp into bounds.
    assert g.entry_node(2.9, 1.6, MoveDirection.X_POS) == Node(0, 1)
    assert g.entry_node(-1.0, -1.0, MoveDirection.X_POS) == Node(0, 0)


def test_explore_intersection_advances_node_and_turns_at_edge():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    cmd = m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(10, 0)
    assert m.move_direction == MoveDirection.Y_POS     # turned at the x edge
    assert cmd.mode == int(ControlMode.HOLD)           # settling the turn

    # the turn settles once the DR speed bleeds off; the next pulse then steps up
    m.step(0.5, _sensors(vx=0.0, vy=0.0), _perception())
    m.step(1.0, _sensors(vx=0.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(10, 1)               # stepped up, stayed in grid


# ---------------------------------------------------------------------------
# turn settle (brake before an axis-changing / reversing leg)
# ---------------------------------------------------------------------------

def test_explore_axis_change_enters_settle_hold():
    """An axis-changing turn (X -> Y) commands HOLD, not FOLLOW_LINE, while
    it still carries transit momentum."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS     # at the x edge -> planner turns Y

    cmd = m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(10, 0)
    assert m.move_direction == MoveDirection.Y_POS    # axis changed X -> Y
    assert cmd.mode == int(ControlMode.HOLD)          # settling, not cruising


def test_settle_exits_on_speed_drop():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    cmd = m.step(0.5, _sensors(vx=0.9, vy=0.0), _perception())
    assert cmd.mode == int(ControlMode.HOLD)          # still moving -> hold

    cmd = m.step(1.0, _sensors(vx=0.2, vy=0.1), _perception())
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)   # speed < 0.25 -> resume


def test_settle_exits_on_timeout():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    cmd = m.step(3.9, _sensors(vx=1.0, vy=0.0), _perception())
    assert cmd.mode == int(ControlMode.HOLD)          # never slowed, 3.9 s < 4.0

    cmd = m.step(4.0, _sensors(vx=1.0, vy=0.0), _perception())
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)   # timeout fired


def test_settle_falls_back_to_min_time_without_velocity():
    """No velocity estimate: settle holds for settle_min_s, then resumes."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    m.step(0.0, _sensors(), _perception(intersection=True))
    cmd = m.step(1.0, _sensors(), _perception())
    assert cmd.mode == int(ControlMode.HOLD)          # 1.0 s < 1.5 s min hold

    cmd = m.step(1.5, _sensors(), _perception())
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)   # min hold elapsed


def test_pulses_during_settle_do_not_advance_node():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    node_at_settle = m.current_node          # (10, 0), the counted node
    dir_at_settle = m.move_direction         # Y_POS

    # pump pulses while still settling: navigation must not move
    for t in (0.5, 1.0, 1.5):
        cmd = m.step(t, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
        assert cmd.mode == int(ControlMode.HOLD)
        assert m.current_node == node_at_settle
        assert m.move_direction == dir_at_settle


def test_same_axis_reversal_settles():
    """The degenerate single-row fallback flips X_POS -> X_NEG with no Y move;
    that reversal settles too."""
    grid = GridMap(node_count_x=3, node_count_y=1, cell_size_m=3.0, logger=_null)
    m = MissionManager(grid_map=grid, logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(1, 0)
    m.move_direction = MoveDirection.X_POS

    cmd = m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(2, 0)               # advanced to the x edge
    assert m.move_direction == MoveDirection.X_NEG    # reversed on the same axis
    assert cmd.mode == int(ControlMode.HOLD)          # reversal settles


def test_straight_advance_does_not_settle():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(2, 0)
    m.move_direction = MoveDirection.X_POS

    cmd = m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(3, 0)
    assert m.move_direction == MoveDirection.X_POS    # kept cruising the row
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)   # no settle
    assert m._settle_until_mode is None


def test_follow_rescue_path_turn_settles():
    """An L-turn in the rescue path (X_POS -> Y_POS) settles at the corner."""
    m = MissionManager(logger=_null)
    m.state = MissionState.FOLLOW_RESCUE_PATH
    m.current_node = Node(0, 0)
    m.rescue_path = [Node(1, 0), Node(1, 1)]
    m.path_index = 0

    cmd = m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception())
    assert m.move_direction == MoveDirection.X_POS
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)

    # reaching the corner peeks the turn and settles
    cmd = m.step(1.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(1, 0)
    assert m.move_direction == MoveDirection.Y_POS
    assert cmd.mode == int(ControlMode.HOLD)

    # a pulse while settling is ignored; resume once the speed drops
    cmd = m.step(1.5, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert cmd.mode == int(ControlMode.HOLD)
    assert m.current_node == Node(1, 0)
    cmd = m.step(2.0, _sensors(vx=0.0, vy=0.0), _perception())
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)
    assert m.move_direction == MoveDirection.Y_POS


# ---------------------------------------------------------------------------
# marker confirmation (3 s majority vote + DR snap)
# ---------------------------------------------------------------------------

def test_marker_confirm_majority_vote():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(2, 0)

    cmd = m.step(10.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=3, aruco_conf=0.9))
    assert m.state == MissionState.MARKER_CONFIRM
    assert cmd.mode == int(ControlMode.ALIGN_MARKER)

    # accumulate votes 3, 3, 2 inside the 3 s window
    m.step(10.5, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=3))
    m.step(11.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=3))
    m.step(12.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=2))
    assert m.state == MissionState.MARKER_CONFIRM   # window not elapsed yet

    m.step(13.0, _sensors(dr_x=6.0, dr_y=0.0), _perception())
    assert m.state == MissionState.EXPLORE
    assert m.grid_map.contains_marker(3)
    assert not m.grid_map.contains_marker(2)
    assert m.grid_map.node_of_marker(3) == Node(2, 0)


def test_marker_confirm_snaps_to_dr_node_not_stale_count():
    """Marker spotted mid-edge: it is recorded at the DR-nearest node and
    the node index is re-zeroed there, not left at the stale counted node."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(2, 0)     # stale count; drone actually drifted on

    m.step(0.0, _sensors(dr_x=9.0, dr_y=0.2), _perception(aruco_id=7))
    m.step(1.0, _sensors(dr_x=9.0, dr_y=0.2), _perception(aruco_id=7))
    m.step(3.0, _sensors(dr_x=9.0, dr_y=0.2), _perception())

    assert m.state == MissionState.EXPLORE
    assert m.grid_map.node_of_marker(7) == Node(3, 0)   # DR (9.0, 0.2) -> (3, 0)
    assert m.current_node == Node(3, 0)                 # re-zeroed, not (2, 0)


def test_marker_confirm_dr_none_falls_back_to_current_node():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(4, 1)

    m.step(0.0, _sensors(), _perception(aruco_id=5))
    m.step(1.0, _sensors(), _perception(aruco_id=5))
    m.step(3.0, _sensors(), _perception())

    assert m.grid_map.node_of_marker(5) == Node(4, 1)
    assert m.current_node == Node(4, 1)


def test_marker_confirm_records_marker_node_not_overshot_drone_node():
    """r84 overshoot: the drone brakes a full cell past the marker while the
    hover ticks see the marker's center errors pointing back at its true node.
    The record must land on the marker node; current_node on the drone node."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(2, 0)     # stale count
    m.move_direction = MoveDirection.X_NEG

    # marker sits at world (3, 0) = node (1, 0); center errors = marker - drone
    m.step(0.0, _sensors(dr_x=4.0, dr_y=0.1), _perception(aruco_id=14, cx=-1.0, cy=-0.1))
    m.step(1.0, _sensors(dr_x=2.0, dr_y=0.05), _perception(aruco_id=14, cx=1.0, cy=-0.05))
    m.step(2.0, _sensors(dr_x=0.5, dr_y=0.0), _perception(aruco_id=14, cx=2.5, cy=0.0))
    # drone ends at world (0, 0) = node (0, 0), one cell past the marker
    m.step(3.0, _sensors(dr_x=0.0, dr_y=0.0), _perception())

    assert m.state == MissionState.EXPLORE
    assert m.grid_map.node_of_marker(14) == Node(1, 0)   # marker's own node
    assert m.current_node == Node(0, 0)                  # drone's actual node
    assert m.grid_map.node_of_marker(14) != m.current_node


def test_marker_confirm_id_and_node_majority_combined():
    """The winning id is the id majority; the recorded node is that id's node
    majority. A minority id and a minority node both lose."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 0)

    # marker at world (6, 0) = node (2, 0)
    m.step(0.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=8))
    m.step(0.5, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=8))
    m.step(1.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=8))
    # a single id-8 sighting whose projection lands on node (3, 0): minority node
    m.step(1.5, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=8, cx=3.0))
    # a minority id (2) projecting onto (2, 0): loses the id vote
    m.step(2.0, _sensors(dr_x=6.0, dr_y=0.0), _perception(aruco_id=2))
    m.step(3.0, _sensors(dr_x=6.0, dr_y=0.0), _perception())

    assert m.grid_map.contains_marker(8)
    assert not m.grid_map.contains_marker(2)
    assert m.grid_map.node_of_marker(8) == Node(2, 0)


def test_marker_confirm_rejects_when_projection_out_of_tolerance():
    """Every hover projection sits farther than snap_max_err from any node, so
    the id has zero node votes and the confirm is rejected (nothing recorded).
    current_node still re-zeros to the drone's DR node."""
    m = MissionManager(logger=_null, snap_max_err=0.5)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 0)

    m.step(0.0, _sensors(dr_x=9.0, dr_y=0.0), _perception(aruco_id=7))
    # projected 1.2 m off node (3, 0) each tick > snap_max_err 0.5 -> no vote
    m.step(1.0, _sensors(dr_x=9.0, dr_y=0.0), _perception(aruco_id=7, cx=1.2))
    m.step(2.0, _sensors(dr_x=9.0, dr_y=0.0), _perception(aruco_id=7, cx=1.2))
    m.step(3.0, _sensors(dr_x=9.0, dr_y=0.0), _perception())

    assert m.state == MissionState.EXPLORE
    assert not m.grid_map.contains_marker(7)
    assert m.current_node == Node(3, 0)


def test_marker_confirm_rechooses_stale_outward_direction():
    """After the re-zero parks the drone on an edge node, a stale outward
    move_direction is re-chosen so the next pulse advances INTO the grid
    instead of stepping off it (r84 add_edge out-of-bounds crash)."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(2, 2)
    m.move_direction = MoveDirection.X_NEG    # points off-grid once re-zeroed

    # marker at world (3, 6) = node (1, 2); the drone brakes a cell past it
    m.step(0.0, _sensors(dr_x=3.5, dr_y=6.0), _perception(aruco_id=20, cx=-0.5))
    m.step(1.0, _sensors(dr_x=2.0, dr_y=6.0), _perception(aruco_id=20, cx=1.0))
    m.step(2.0, _sensors(dr_x=0.5, dr_y=6.0), _perception(aruco_id=20, cx=2.5))
    m.step(3.0, _sensors(dr_x=0.0, dr_y=6.0), _perception())

    assert m.state == MissionState.EXPLORE
    assert m.current_node == Node(0, 2)                 # drone node (x edge)
    assert m.grid_map.node_of_marker(20) == Node(1, 2)  # marker's own node
    assert m.move_direction != MoveDirection.X_NEG      # stale outward cleared
    assert m.move_direction == MoveDirection.Y_POS      # re-chosen into the grid

    # the next pulse advances into the grid; no exception is raised
    m.step(4.0, _sensors(dr_x=0.0, dr_y=9.0), _perception(intersection=True))
    assert m.current_node == Node(0, 3)


def test_explore_off_grid_pulse_dropped_and_direction_recovered():
    """Defensive net: a pulse whose advance would leave the grid is dropped
    (no add_edge, no raise); the direction is re-chosen so the following pulse
    advances in."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(0, 3)
    m.move_direction = MoveDirection.X_NEG    # (-1, 3) is off-grid

    cmd = m.step(0.0, _sensors(vx=0.0, vy=0.0), _perception(intersection=True))
    assert m.current_node == Node(0, 3)               # not advanced
    assert m.move_direction != MoveDirection.X_NEG    # re-chosen
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)

    m.step(1.0, _sensors(vx=0.0, vy=0.0), _perception(intersection=True))
    assert m.grid_map.in_bounds(m.current_node)
    assert m.current_node != Node(0, 3)               # advanced into the grid


def test_follow_rescue_off_grid_pulse_dropped_without_advance():
    """Defensive net on the rescue path: a malformed off-grid waypoint would
    step the advance off the grid; the pulse is dropped instead of raising."""
    m = MissionManager(logger=_null)
    m.state = MissionState.FOLLOW_RESCUE_PATH
    m.current_node = Node(0, 0)
    m.rescue_path = [Node(-1, 0)]     # intentionally off-grid to force the guard
    m.path_index = 0

    cmd = m.step(0.0, _sensors(vx=0.0, vy=0.0), _perception(intersection=True))
    assert m.state == MissionState.FOLLOW_RESCUE_PATH
    assert m.current_node == Node(0, 0)               # not advanced, no raise
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)


# ---------------------------------------------------------------------------
# rescue path (BFS in ascending ID order) -> LAND -> FINISHED
# ---------------------------------------------------------------------------

def test_rescue_path_ascending_id_then_land_finished():
    m = MissionManager(logger=_null)
    # place four markers with IDs out of order, all along row 0
    m.grid_map.save_marker(3, Node(6, 0))
    m.grid_map.save_marker(1, Node(2, 0))
    m.grid_map.save_marker(4, Node(8, 0))
    m.grid_map.save_marker(2, Node(4, 0))
    m.state = MissionState.EXPLORE
    m.current_node = Node(0, 0)
    m.home_node = Node(0, 0)

    cmd = m.step(0.0, _sensors(), _perception())
    assert m.state == MissionState.PLAN_RESCUE_PATH
    assert cmd.mode == int(ControlMode.HOLD)

    cmd = m.step(0.0, _sensors(), _perception())
    assert m.state == MissionState.FOLLOW_RESCUE_PATH

    # marker nodes appear in the path in ascending-ID order
    marker_nodes = [Node(2, 0), Node(4, 0), Node(6, 0), Node(8, 0)]
    idxs = [m.rescue_path.index(n) for n in marker_nodes]
    assert idxs == sorted(idxs)

    # walk the whole path via intersection pulses; vx/vy=0 lets the row-end
    # reversal (X_POS -> X_NEG) settle immediately instead of stalling.
    for _ in range(200):
        if m.state != MissionState.FOLLOW_RESCUE_PATH:
            break
        m.step(0.0, _sensors(vx=0.0, vy=0.0), _perception(intersection=True))
    assert m.state == MissionState.LAND
    assert m.current_node == Node(0, 0)     # returned home

    cmd = m.step(0.0, _sensors(altitude=0.1), _perception())
    assert m.state == MissionState.FINISHED
    assert cmd.mode == int(ControlMode.LAND_ON_MARKER)

    cmd = m.step(0.0, _sensors(altitude=0.1), _perception())
    assert cmd.mode == int(ControlMode.STOP)


def test_plan_rescue_path_empty_goes_failsafe():
    # a marker unreachable relative to home cannot happen on a full grid,
    # so force the empty-path branch with a stubbed planner.
    m = MissionManager(logger=_null)
    m.grid_map.save_marker(1, Node(1, 0))
    m.grid_map.save_marker(2, Node(2, 0))
    m.grid_map.save_marker(3, Node(3, 0))
    m.grid_map.save_marker(4, Node(4, 0))
    m.state = MissionState.EXPLORE

    class _EmptyPlanner(PathPlanner):
        def build_rescue_path(self, grid_map, current_node, home_node):
            return []

    m.path_planner = _EmptyPlanner(logger=_null)
    m.step(0.0, _sensors(), _perception())          # -> PLAN_RESCUE_PATH
    m.step(0.0, _sensors(), _perception())          # build empty -> FAILSAFE
    assert m.state == MissionState.FAILSAFE


# ---------------------------------------------------------------------------
# failsafe
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "kwargs",
    [
        {"imu_ok": False},
        {"lidar_ok": False},
        {"rc": False},
        {"battery": 13.5},
    ],
)
def test_failsafe_on_each_sensor_flag(kwargs):
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    cmd = m.step(0.0, _sensors(**kwargs), _perception(line_visible=True))
    assert m.state == MissionState.FAILSAFE
    assert cmd.mode == int(ControlMode.EMERGENCY_LAND)
    assert cmd.emergency is True


def test_battery_at_threshold_is_ok():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.step(0.0, _sensors(battery=14.0), _perception(line_visible=True))
    assert m.state == MissionState.EXPLORE     # 14.0 is not below the 14.0 V threshold


# ---------------------------------------------------------------------------
# McuCommand field passthrough
# ---------------------------------------------------------------------------

def test_mcu_command_field_passthrough_and_flags():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)
    m.move_direction = MoveDirection.Y_POS
    m.grid_map.save_marker(9, Node(3, 2))      # already seen -> no MARKER_CONFIRM

    perc = _perception(
        has_vertical=True, has_horizontal=True, dx=0.12, dy=-0.07,
        angle_error=-0.05, line_conf=0.8,
        fwd=True, left=False, right=True, back=False,
        aruco_id=9, cx=0.03, cy=-0.04, yaw=0.01, aruco_conf=0.7,
    )
    cmd = m.step(0.0, _sensors(vx=0.5, vy=-0.2), perc)

    assert m.state == MissionState.EXPLORE
    assert cmd.mission_state == int(MissionState.EXPLORE)
    assert cmd.seq == 1
    assert (cmd.node_x, cmd.node_y) == (3, 2)
    assert cmd.move_direction == int(MoveDirection.Y_POS)

    assert cmd.vertical_line is True
    assert cmd.horizontal_line is True
    assert cmd.line_dx == 0.12
    assert cmd.line_dy == -0.07
    assert cmd.line_angle_error == -0.05
    assert cmd.line_confidence == 0.8

    assert cmd.intersection_forward is True and cmd.intersection_right is True
    assert cmd.intersection_left is False and cmd.intersection_backward is False

    assert cmd.marker_detected is True and cmd.marker_id == 9
    assert cmd.marker_error_x == 0.03 and cmd.marker_error_y == -0.04
    assert cmd.marker_yaw_error == 0.01

    assert cmd.vx_est == 0.5 and cmd.vy_est == -0.2
    assert cmd.vel_est_valid is True


def test_mcu_command_marker_none_and_invalid_velocity():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.marker_id == -1
    assert cmd.marker_detected is False
    assert cmd.vel_est_valid is False
    assert cmd.vx_est == 0.0 and cmd.vy_est == 0.0


def test_send_command_hook_is_overridable():
    captured = []
    m = MissionManager(logger=_null)
    m.send_command_to_mcu = lambda cmd: captured.append(cmd)
    m.state = MissionState.EXPLORE
    out = m.step(0.0, _sensors(), _perception(line_visible=True))
    assert captured == [out]


# ---------------------------------------------------------------------------
# speed scheduling (MISSION_INTERFACE 7a)
# ---------------------------------------------------------------------------

def test_mcu_command_default_speed_scale_is_full():
    assert McuCommand().speed_scale == 100


def test_speed_scale_straight_leg_is_full():
    """A mid-row X straight, no hint, not a first leg: full cruise."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)
    m.move_direction = MoveDirection.X_POS
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 100


def test_speed_scale_transit_leg_is_slow():
    """Y travel (moving between rows) is a transit leg -> scale_transit."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(5, 2)
    m.move_direction = MoveDirection.Y_POS
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 40


def test_speed_scale_final_leg_before_boundary_is_slow():
    """The leg whose next node is the last in-bounds one -> scale_final_leg."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 2)     # next (10,2) is the x-edge; (11,2) is OOB
    m.move_direction = MoveDirection.X_POS
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 50


def test_speed_scale_first_leg_after_settle_until_next_advance():
    """A settle exit starts the next leg slow (scale_transit) even on an X
    leg that is otherwise full speed, until the next node advance clears it."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    # row-end turn X_POS -> Y_POS settles
    m.step(0.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    # settle exits onto the Y transit leg (transit AND first-leg -> slow)
    cmd = m.step(0.5, _sensors(vx=0.0, vy=0.0), _perception())
    assert cmd.speed_scale == 40

    # step up a row; planner reverses to X_NEG (a turn) -> settles again
    m.step(1.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    # settle exits onto an X leg: slow ONLY because it is the first leg
    cmd = m.step(1.5, _sensors(vx=0.0, vy=0.0), _perception())
    assert m.move_direction == MoveDirection.X_NEG
    assert cmd.speed_scale == 40

    # next advance clears the first-leg flag; mid-row X straight is full speed
    cmd = m.step(2.0, _sensors(vx=1.0, vy=0.0), _perception(intersection=True))
    assert m.move_direction == MoveDirection.X_NEG
    assert cmd.speed_scale == 100


def test_speed_scale_front_hint_within_range_slows_outside_does_not():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)     # mid-row X straight -> baseline 100
    m.move_direction = MoveDirection.X_POS

    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 100

    # a hint within range slows to scale_hint
    m.set_front_hint(5, Node(6, 2), 3.0)
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 50

    # a hint beyond range does not slow
    m.set_front_hint(5, Node(6, 2), 6.0)
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 100

    # a None id clears the hint back to full speed
    m.set_front_hint(5, Node(6, 2), 3.0)
    m.set_front_hint(None, None, None)
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 100


def test_speed_scale_hint_for_recorded_marker_is_ignored():
    """A hint for a marker already recorded is dropped, so it never slows."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)
    m.move_direction = MoveDirection.X_POS
    m.grid_map.save_marker(5, Node(6, 2))

    m.set_front_hint(5, Node(6, 2), 2.0)     # ignored: id already recorded
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 100


def test_speed_scale_lowest_wins_when_rules_combine():
    """Transit (40), final-leg (50) and hint (50) all apply at once; the
    lowest wins."""
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(5, 6)     # Y_POS: next (5,7) last in-bounds row
    m.move_direction = MoveDirection.Y_POS
    m.set_front_hint(9, Node(5, 7), 2.0)
    cmd = m.step(0.0, _sensors(), _perception())
    assert cmd.speed_scale == 40


# ---------------------------------------------------------------------------
# lost-line recovery (MISSION_INTERFACE 3)
# ---------------------------------------------------------------------------

def _drive_into_recovery(m, node, direction, dr_x, dr_y):
    """Put the manager on a cruising leg with the needed line absent and
    step past lost_line_timeout_s so recovery engages. Returns the command
    from the tick recovery entered."""
    m.state = MissionState.EXPLORE
    m.current_node = node
    m.move_direction = direction
    s = _sensors(dr_x=dr_x, dr_y=dr_y)
    m.step(0.0, s, _perception())          # first absent tick starts the clock
    return m.step(2.0, s, _perception())   # timeout reached -> enter recovery


def test_recovery_enters_only_after_timeout():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)
    m.move_direction = MoveDirection.X_POS
    s = _sensors(dr_x=9.0, dr_y=6.0)

    m.step(0.0, s, _perception())          # needed vertical line absent
    assert not m._recovery_active
    cmd = m.step(1.9, s, _perception())    # still under the 2.0 s timeout
    assert not m._recovery_active
    assert cmd.speed_scale != 0
    cmd = m.step(2.0, s, _perception())    # timeout reached
    assert m._recovery_active
    assert cmd.speed_scale == 0


def test_recovery_line_dx_pulls_toward_nominal_from_both_sides():
    # nominal row for node (3, 2) is world y = 6.0
    m = MissionManager(logger=_null)
    cmd = _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=5.0)
    assert m._recovery_active
    assert cmd.vertical_line is True
    assert cmd.line_dx == pytest.approx(1.0)     # below the row -> pull +y

    m2 = MissionManager(logger=_null)
    cmd2 = _drive_into_recovery(m2, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=7.0)
    assert cmd2.line_dx == pytest.approx(-1.0)   # above the row -> pull -y


def test_recovery_clamps_synthesized_offset_to_wire_range():
    m = MissionManager(logger=_null)
    # nominal y = 6.0, dr_y = 2.0 -> raw 4.0 clamps to +2.0
    cmd = _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=2.0)
    assert cmd.line_dx == pytest.approx(2.0)


def test_recovery_y_travel_uses_line_dy():
    m = MissionManager(logger=_null)
    # Y_POS -> needed horizontal line; nominal column for node (3, 2) is x = 9.0
    cmd = _drive_into_recovery(m, Node(3, 2), MoveDirection.Y_POS, dr_x=8.0, dr_y=6.0)
    assert m._recovery_active
    assert cmd.horizontal_line is True
    assert cmd.line_dy == pytest.approx(1.0)     # left of the column -> pull +x
    assert cmd.vertical_line is False
    assert cmd.line_dx == 0.0


def test_recovery_forces_speed_scale_zero_and_follow_line():
    m = MissionManager(logger=_null)
    cmd = _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=6.0)
    assert cmd.speed_scale == 0
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)


def test_recovery_reacquire_needs_consecutive_ticks_flicker_resets():
    m = MissionManager(logger=_null)
    _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=6.0)
    assert m._recovery_active
    s = _sensors(dr_x=9.0, dr_y=6.0)
    present = _perception(has_vertical=True)
    absent = _perception()

    m.step(2.1, s, present)                  # count 1
    m.step(2.2, s, present)                  # count 2 (not yet 3)
    assert m._recovery_active
    m.step(2.3, s, absent)                   # a flicker resets the count
    assert m._recovery_active
    assert m._recovery_reacquire_count == 0
    m.step(2.4, s, present)                  # count 1 again
    m.step(2.5, s, present)                  # count 2
    m.step(2.6, s, present)                  # count 3 -> exit
    assert not m._recovery_active


def test_recovery_exit_resumes_cruise_scale():
    m = MissionManager(logger=_null)
    _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=6.0)
    s = _sensors(dr_x=9.0, dr_y=6.0)
    present = _perception(has_vertical=True)
    m.step(2.1, s, present)                  # count 1
    m.step(2.2, s, present)                  # count 2
    cmd = m.step(2.3, s, present)            # count 3 -> exit, normal cruise
    assert not m._recovery_active
    assert cmd.speed_scale == 100            # mid-row X straight, full cruise
    assert cmd.vertical_line is True         # the real line, passed through
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)


def test_recovery_timeout_goes_failsafe():
    m = MissionManager(logger=_null)
    _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=6.0)
    assert m._recovery_active                # entered at now = 2.0
    s = _sensors(dr_x=9.0, dr_y=6.0)
    cmd = m.step(21.9, s, _perception())     # 19.9 s in recovery, still short of 20
    assert m.state == MissionState.EXPLORE
    assert m._recovery_active
    cmd = m.step(22.0, s, _perception())     # 20.0 s without reacquisition
    assert m.state == MissionState.FAILSAFE
    assert cmd.mode == int(ControlMode.EMERGENCY_LAND)


def test_recovery_does_not_enter_without_dr():
    logs = []
    m = MissionManager(logger=logs.append)
    m.state = MissionState.EXPLORE
    m.current_node = Node(3, 2)
    m.move_direction = MoveDirection.X_POS
    s = _sensors()                           # DR None

    m.step(0.0, s, _perception())
    cmd = m.step(2.0, s, _perception())      # timeout, but DR unavailable
    assert not m._recovery_active
    assert cmd.speed_scale != 0              # stays with the HOLD degradation
    assert cmd.mode == int(ControlMode.FOLLOW_LINE)
    m.step(3.0, s, _perception())            # still absent, no second log
    unavailable = [line for line in logs if "[RECOVER] unavailable" in line]
    assert len(unavailable) == 1
    assert "dr=none" in unavailable[0]


def test_recovery_ignores_intersection_pulses():
    m = MissionManager(logger=_null)
    _drive_into_recovery(m, Node(3, 2), MoveDirection.X_POS, dr_x=9.0, dr_y=6.0)
    assert m._recovery_active
    node_before = m.current_node
    s = _sensors(dr_x=9.0, dr_y=6.0)
    # a pulse while off the line is untrustworthy -> ignored, node stays put
    cmd = m.step(2.1, s, _perception(intersection=True))
    assert m.current_node == node_before
    assert m._recovery_active
    assert cmd.speed_scale == 0
