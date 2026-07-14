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
    assert m.current_node == Node(1, 0)     # DR (3.1, 0) -> node (1, 0)
    assert m.home_node == Node(1, 0)


def test_explore_intersection_advances_node_and_turns_at_edge():
    m = MissionManager(logger=_null)
    m.state = MissionState.EXPLORE
    m.current_node = Node(9, 0)
    m.move_direction = MoveDirection.X_POS

    m.step(0.0, _sensors(), _perception(intersection=True))
    assert m.current_node == Node(10, 0)
    assert m.move_direction == MoveDirection.Y_POS     # turned at the x edge

    m.step(0.0, _sensors(), _perception(intersection=True))
    assert m.current_node == Node(10, 1)               # stepped up, stayed in grid


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

    # walk the whole path via intersection pulses
    for _ in range(200):
        if m.state != MissionState.FOLLOW_RESCUE_PATH:
            break
        m.step(0.0, _sensors(), _perception(intersection=True))
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
