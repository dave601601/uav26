"""Jetson-side mission layer for the node-based navigation algorithm.

This module is the sim-side source of truth for the mission state
machine that runs on the Jetson. It decides WHAT to do (mission state,
control mode, travel direction, and the raw vision errors to forward);
the MCU decides HOW to fly it (error -> velocity -> attitude -> mixer).
The whole file is pure Python (stdlib only) so it can be unit-tested
without ROS and later ported to C++/STM32.

Frames and units (metric at this layer):
  - Body frame is REP-103 FLU: +x forward, +y left. Grid axes are
    aligned with body axes because yaw is locked to the initial heading.
  - Grid node (i, j) maps to world (i * cell, j * cell) meters, arena
    SW corner at (0, 0). Default arena 30 x 21 m, 3 m cell -> 11 x 8
    intersection nodes.
  - LineDetection carries both grid-line offsets in body meters: dx is the
    signed +y position of the nearest vertical line, dy the signed +x
    position of the nearest horizontal line (the [dx, dy, flag] contract,
    MISSION_INTERFACE section 6). angle_error is radians, FLU +CCW.
    ArucoDetection center errors are body meters.
  - The ROS node converts pixels to meters (altitude / f) before it
    builds PerceptionData; this layer never sees pixels.

Navigation is node-based: position is an integer node plus a
MoveDirection, advanced one node per intersection pulse. World meters
are logged alongside but never drive control, except two snap points:
grid entry and each marker confirmation both re-zero the node index
against the companion dead-reckoning (DR) estimate.

No binary packing lives here. McuCommand mirrors the wire fields as
plain Python values; the C protocol layer owns the byte layout.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from enum import IntEnum
from math import ceil, floor, hypot
from typing import Callable, Dict, List, Optional, Tuple


# ============================================================
# 1. Enums (fixed values, shared Jetson/MCU)
# ============================================================


class MissionState(IntEnum):
    """Mission state, fixed values so a serial log reading state=4 is
    immediately meaningful on both the Jetson and the MCU."""

    INIT = 0
    TAKEOFF = 1
    LOCALIZE = 2
    ENTER_GRID = 3
    EXPLORE = 4
    MARKER_CONFIRM = 5
    PLAN_RESCUE_PATH = 6
    FOLLOW_RESCUE_PATH = 7
    RETURN_HOME = 8
    LAND = 9
    FINISHED = 10
    FAILSAFE = 11


class ControlMode(IntEnum):
    """Control mode requested from the MCU outer loop. Replaces the
    skeleton's mode strings: a serial link cannot carry strings, so the
    byte value travels and .name is logged for readability."""

    HOLD = 0
    FOLLOW_LINE = 1
    ALIGN_MARKER = 2
    SEARCH_LINE = 3
    MOVE_TO_LANDMARK = 4
    LAND_ON_MARKER = 5
    STOP = 6
    EMERGENCY_LAND = 7


class MoveDirection(IntEnum):
    """Fixed-yaw body-frame travel direction (FLU: +x forward, +y left).
    Grid axes are aligned with body axes under yaw lock."""

    X_POS = 0
    X_NEG = 1
    Y_POS = 2
    Y_NEG = 3


@dataclass(frozen=True)
class Node:
    """Grid intersection index. Node(1, 0) is one cell along +x, i.e.
    one cell width (cell_size_m meters) from the origin."""

    x: int
    y: int


# ============================================================
# 2. Perception / sensor / command dataclasses
# ============================================================


@dataclass
class LineDetection:
    """Grid-line estimate for both axes (the [dx, dy, flag] contract).

    dx is the signed body +y position of the nearest vertical grid line,
    dy the signed body +x position of the nearest horizontal grid line
    (meters, 0.0 when that line is absent — see has_vertical / has_horizontal).
    angle_error is the followed line's heading vs the travel axis in radians
    FLU (+CCW); confidence 0..1. The MCU selects dx or dy by travel axis; the
    Jetson selects only angle_error."""

    has_vertical: bool = False
    has_horizontal: bool = False
    dx: float = 0.0
    dy: float = 0.0
    angle_error: float = 0.0
    confidence: float = 0.0


@dataclass
class IntersectionDetection:
    """Grid-crossing detector output. detected is a PULSE: true for
    exactly one result per physical crossing (hysteresis lives in
    perception). The four booleans report which branches extend from
    the crossing, relative to the current travel axis."""

    detected: bool = False
    forward: bool = False
    left: bool = False
    right: bool = False
    backward: bool = False


@dataclass
class ArucoDetection:
    """ArUco marker estimate. center_error_x/y are body-frame meters
    (+x forward, +y left); yaw_error is radians; confidence 0..1.
    marker_id is None when nothing is detected."""

    detected: bool = False
    marker_id: Optional[int] = None
    center_error_x: float = 0.0
    center_error_y: float = 0.0
    yaw_error: float = 0.0
    confidence: float = 0.0


@dataclass
class PerceptionData:
    """One perception frame handed to MissionManager (all metric)."""

    line: LineDetection
    intersection: IntersectionDetection
    aruco: ArucoDetection


@dataclass
class SensorData:
    """Companion / flight-controller telemetry for one loop.

    dr_x, dr_y are the dead-reckoning world position estimate in meters,
    used ONLY for the two snap points (grid entry, marker confirm) and
    for logging; None when no estimate is available. vx_est, vy_est are
    the DR body-frame velocity in m/s that pass through to McuCommand
    with vel_est_valid; None when the estimate is not valid.
    """

    altitude: float
    battery_voltage: float
    imu_ok: bool
    lidar_ok: bool
    rc_connected: bool
    dr_x: Optional[float] = None
    dr_y: Optional[float] = None
    vx_est: Optional[float] = None
    vy_est: Optional[float] = None


@dataclass
class McuCommand:
    """High-level command sent to the MCU, one field per wire field
    (MISSION_INTERFACE section 6). Values are plain Python: metric
    meters/radians, ControlMode/MissionState/MoveDirection as ints,
    marker_id -1 when none. The C protocol layer owns the byte layout
    (Q14 scaling, u8 confidences, the flags/flags2 bytes); the
    vertical_line/horizontal_line, intersection and marker booleans pack
    into flags, vel_est_valid and emergency into flags2 bits 0/1.
    """

    mode: int = int(ControlMode.HOLD)
    mission_state: int = int(MissionState.INIT)
    seq: int = 0

    node_x: int = 0
    node_y: int = 0
    move_direction: int = int(MoveDirection.X_POS)

    target_altitude: float = 2.0

    vertical_line: bool = False
    horizontal_line: bool = False
    line_dx: float = 0.0
    line_dy: float = 0.0
    line_angle_error: float = 0.0
    line_confidence: float = 0.0

    intersection_detected: bool = False
    intersection_forward: bool = False
    intersection_left: bool = False
    intersection_right: bool = False
    intersection_backward: bool = False

    marker_detected: bool = False
    marker_id: int = -1
    marker_error_x: float = 0.0
    marker_error_y: float = 0.0
    marker_yaw_error: float = 0.0
    marker_confidence: float = 0.0

    vx_est: float = 0.0
    vy_est: float = 0.0
    vel_est_valid: bool = False

    emergency: bool = False

    # Cruise scaling percent 0..100 (100 = full cruise); the MCU applies
    # effective_cruise = cruise * speed_scale / 100. Values >100 clamp on decode.
    speed_scale: int = 100


# ============================================================
# 3. Direction helpers
# ============================================================


def move_direction_vector(direction: MoveDirection) -> Tuple[int, int]:
    """MoveDirection -> unit (dx, dy) in body/grid indices. Body x/y and
    grid x/y are aligned; change only this function if that changes."""

    if direction == MoveDirection.X_POS:
        return 1, 0
    if direction == MoveDirection.X_NEG:
        return -1, 0
    if direction == MoveDirection.Y_POS:
        return 0, 1
    if direction == MoveDirection.Y_NEG:
        return 0, -1
    raise ValueError(f"Unknown move direction: {direction}")


def move_to_next_node(node: Node, direction: MoveDirection) -> Node:
    """Advance the node index by one crossing in the travel direction."""

    dx, dy = move_direction_vector(direction)
    return Node(node.x + dx, node.y + dy)


def direction_to_adjacent_node(current: Node, target: Node) -> MoveDirection:
    """Travel direction from current to an orthogonally adjacent target."""

    dx = target.x - current.x
    dy = target.y - current.y

    if dx == 1 and dy == 0:
        return MoveDirection.X_POS
    if dx == -1 and dy == 0:
        return MoveDirection.X_NEG
    if dx == 0 and dy == 1:
        return MoveDirection.Y_POS
    if dx == 0 and dy == -1:
        return MoveDirection.Y_NEG
    raise ValueError(
        f"Target node {target} is not adjacent to current node {current}"
    )


def _flip_x(direction: MoveDirection) -> MoveDirection:
    return MoveDirection.X_NEG if direction == MoveDirection.X_POS else MoveDirection.X_POS


def _flip_y(direction: MoveDirection) -> MoveDirection:
    return MoveDirection.Y_NEG if direction == MoveDirection.Y_POS else MoveDirection.Y_POS


# ============================================================
# 4. Grid map (preallocated)
# ============================================================


@dataclass
class GridNodeInfo:
    """Per-intersection state the GridMap tracks: visit and marker flags."""

    node: Node
    visited: bool = False
    has_marker: bool = False
    marker_id: Optional[int] = None
    marker_confirmed: bool = False
    marker_order: Optional[int] = None


class GridMap:
    """Full-scale preallocated grid map.

    All nodes and adjacency are created up front; only visit/marker flags
    change afterwards. Default is the confirmed 11 x 8 intersection grid
    of the 30 x 21 m arena at a 3 m cell (MISSION_INTERFACE section 2).
    """

    def __init__(
        self,
        node_count_x: int = 11,
        node_count_y: int = 8,
        cell_size_m: float = 3.0,
        logger: Callable[[str], None] = print,
    ):
        self.node_count_x = node_count_x
        self.node_count_y = node_count_y
        self.cell_size_m = cell_size_m
        self._log = logger

        # nodes[y][x]: every intersection is allocated once.
        self.nodes = [
            [GridNodeInfo(node=Node(x, y)) for x in range(node_count_x)]
            for y in range(node_count_y)
        ]

        self.edge_visited_flags: Dict[Tuple[Tuple[int, int], Tuple[int, int]], bool] = {}
        self.marker_id_to_node: Dict[int, Node] = {}
        self.next_marker_order = 0

        for y in range(node_count_y):
            for x in range(node_count_x):
                node = Node(x, y)
                for neighbor in self.neighbors(node):
                    key = self.edge_key(node, neighbor)
                    if key not in self.edge_visited_flags:
                        self.edge_visited_flags[key] = False

    # -------- geometry ----------------------------------------------------

    def in_bounds(self, node: Node) -> bool:
        return 0 <= node.x < self.node_count_x and 0 <= node.y < self.node_count_y

    def validate_node(self, node: Node) -> None:
        if not self.in_bounds(node):
            raise ValueError(f"Node out of grid bounds: {node}")

    def node_world(self, node: Node) -> Tuple[float, float]:
        """Grid index -> nominal world position in meters."""

        return (node.x * self.cell_size_m, node.y * self.cell_size_m)

    def nearest_node(self, x: float, y: float) -> Node:
        """World meters -> nearest in-bounds grid node (used for DR snaps)."""

        i = round(x / self.cell_size_m)
        j = round(y / self.cell_size_m)
        i = max(0, min(self.node_count_x - 1, i))
        j = max(0, min(self.node_count_y - 1, j))
        return Node(i, j)

    def entry_node(self, x: float, y: float, direction: MoveDirection) -> Node:
        """World meters -> the grid-entry node, given the travel direction.

        On the travel axis take the last grid line already crossed (floor
        for positive travel, ceil for negative); on the perpendicular axis
        take the nearest line. Snapping to the NEAREST node on the travel
        axis put the index one cell ahead whenever the drone had not yet
        crossed the nearest line, so the first intersection pulse landed a
        full cell in front of the physical position (r77)."""

        ti = x / self.cell_size_m
        tj = y / self.cell_size_m
        if direction == MoveDirection.X_POS:
            i, j = floor(ti), round(tj)
        elif direction == MoveDirection.X_NEG:
            i, j = ceil(ti), round(tj)
        elif direction == MoveDirection.Y_POS:
            i, j = round(ti), floor(tj)
        else:
            i, j = round(ti), ceil(tj)
        i = max(0, min(self.node_count_x - 1, i))
        j = max(0, min(self.node_count_y - 1, j))
        return Node(i, j)

    # -------- adjacency ---------------------------------------------------

    def are_adjacent(self, a: Node, b: Node) -> bool:
        return abs(a.x - b.x) + abs(a.y - b.y) == 1

    def neighbors(self, node: Node) -> List[Node]:
        self.validate_node(node)
        result: List[Node] = []
        for direction in MoveDirection:
            dx, dy = move_direction_vector(direction)
            neighbor = Node(node.x + dx, node.y + dy)
            if self.in_bounds(neighbor):
                result.append(neighbor)
        return result

    def edge_key(self, a: Node, b: Node) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """Direction-independent edge key: sort the two endpoints."""

        self.validate_node(a)
        self.validate_node(b)
        return tuple(sorted([(a.x, a.y), (b.x, b.y)]))

    # -------- compatibility shims (preallocated grid never grows) ---------

    def add_node(self, node: Node) -> None:
        """Kept for skeleton compatibility; validates instead of adding."""

        self.validate_node(node)

    def add_edge(self, a: Node, b: Node) -> None:
        """Kept for skeleton compatibility; validates adjacency and bounds."""

        self.validate_node(a)
        self.validate_node(b)
        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")

    # -------- visit / marker bookkeeping ----------------------------------

    def node_info(self, node: Node) -> GridNodeInfo:
        self.validate_node(node)
        return self.nodes[node.y][node.x]

    def mark_node_visited(self, node: Node) -> None:
        self.node_info(node).visited = True

    def mark_edge_visited(self, a: Node, b: Node) -> None:
        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")
        self.edge_visited_flags[self.edge_key(a, b)] = True
        self.mark_node_visited(a)
        self.mark_node_visited(b)

    def edge_visited(self, a: Node, b: Node) -> bool:
        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")
        return self.edge_visited_flags[self.edge_key(a, b)]

    def save_marker(self, marker_id: int, node: Node) -> None:
        """Record a confirmed marker at a node (first ID wins)."""

        self.validate_node(node)
        if marker_id in self.marker_id_to_node:
            self._log(f"[MARKER] ID {marker_id} already saved")
            return

        info = self.node_info(node)
        info.has_marker = True
        info.marker_id = marker_id
        info.marker_confirmed = True
        info.marker_order = self.next_marker_order

        self.marker_id_to_node[marker_id] = node
        self.next_marker_order += 1
        self._log(f"[MARKER] ID {marker_id} saved at {node}, order={info.marker_order}")

    def contains_marker(self, marker_id: int) -> bool:
        return marker_id in self.marker_id_to_node

    def marker_count(self) -> int:
        return len(self.marker_id_to_node)

    def sorted_marker_ids(self) -> List[int]:
        return sorted(self.marker_id_to_node.keys())

    def node_of_marker(self, marker_id: int) -> Node:
        return self.marker_id_to_node[marker_id]


# ============================================================
# 5. Path planner (BFS, self-contained for the C++ port)
# ============================================================


class PathPlanner:
    """Shortest-path and rescue-route planning on the grid graph.

    Self-contained on purpose: this whole layer is destined for a C++
    port, so it does not import the ROS-side planner.py. All edges are
    unit length, so BFS is optimal.
    """

    def __init__(self, logger: Callable[[str], None] = print):
        self._log = logger

    def shortest_path(self, grid_map: GridMap, start: Node, goal: Node) -> List[Node]:
        queue: deque = deque([start])
        parent: Dict[Node, Optional[Node]] = {start: None}

        while queue:
            current = queue.popleft()
            if current == goal:
                break
            for nxt in grid_map.neighbors(current):
                if nxt not in parent:
                    parent[nxt] = current
                    queue.append(nxt)

        if goal not in parent:
            self._log("[PATH] No path found")
            return []

        path: List[Node] = []
        cur: Optional[Node] = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()
        return path

    def build_rescue_path(
        self, grid_map: GridMap, current_node: Node, home_node: Node
    ) -> List[Node]:
        """Route current -> markers in ascending ID order -> home. The
        returned path excludes current_node (first entry is the first
        node to move to)."""

        full_path: List[Node] = []
        pos = current_node

        for marker_id in grid_map.sorted_marker_ids():
            target = grid_map.node_of_marker(marker_id)
            segment = self.shortest_path(grid_map, pos, target)
            if not segment:
                return []
            full_path.extend(segment[1:])  # drop the shared seam node
            pos = target

        return_segment = self.shortest_path(grid_map, pos, home_node)
        if not return_segment:
            return []
        full_path.extend(return_segment[1:])
        return full_path


# ============================================================
# 6. Exploration planner (serpentine / boustrophedon)
# ============================================================


class ExplorationPlanner:
    """Chooses the next travel direction during EXPLORE.

    Boustrophedon (lawnmower) sweep by direction rules, no world path:
    sweep a row along x until the edge, step once in y, reverse x for
    the next row. It never returns a direction that would leave the grid
    (the skeleton's keep-direction placeholder walked off the edge and
    add_edge raised). When the grid is exhausted before enough markers
    are found, it reverses the vertical direction and sweeps back, so it
    keeps re-scanning until the mission-complete condition is met.
    """

    def __init__(self):
        self.x_dir = MoveDirection.X_POS
        self.y_dir = MoveDirection.Y_POS

    def choose_direction(
        self,
        grid_map: GridMap,
        current_node: Node,
        current_direction: MoveDirection,
    ) -> MoveDirection:
        # Sync internal sweep state with the direction just traveled so
        # the planner cannot drift from the mission manager.
        if current_direction in (MoveDirection.X_POS, MoveDirection.X_NEG):
            self.x_dir = current_direction
        else:
            self.y_dir = current_direction

        # Keep sweeping along x while the next node stays in bounds.
        if grid_map.in_bounds(move_to_next_node(current_node, self.x_dir)):
            return self.x_dir

        # Row end: step one row in y and reverse x for the next row.
        if grid_map.in_bounds(move_to_next_node(current_node, self.y_dir)):
            self.x_dir = _flip_x(self.x_dir)
            return self.y_dir

        # Grid exhausted in y: restart the sweep by reversing the vertical
        # direction (bounce off the top/bottom row).
        self.y_dir = _flip_y(self.y_dir)
        if grid_map.in_bounds(move_to_next_node(current_node, self.y_dir)):
            self.x_dir = _flip_x(self.x_dir)
            return self.y_dir

        # Degenerate single-row grid: just sweep back along x.
        self.x_dir = _flip_x(self.x_dir)
        return self.x_dir


# ============================================================
# 7. Mission manager (state machine)
# ============================================================


class MissionManager:
    """Node-based mission state machine (INIT..FAILSAFE).

    It never touches OpenCV or motors: it advances the state, picks a
    ControlMode, and builds the McuCommand. Time (now) and the logger are
    injected for sim time and testability.
    """

    def __init__(
        self,
        grid_map: Optional[GridMap] = None,
        logger: Callable[[str], None] = print,
        required_marker_count: int = 4,
        snap_max_err: float = 2.0,
        settle_speed_mps: float = 0.25,
        settle_timeout_s: float = 4.0,
        settle_min_s: float = 1.5,
        scale_transit: int = 40,
        scale_final_leg: int = 50,
        scale_hint: int = 50,
        hint_slow_range_m: float = 4.0,
        lost_line_timeout_s: float = 2.0,
        recovery_reacquire_ticks: int = 3,
        recovery_timeout_s: float = 20.0,
    ):
        self._log = logger
        self.state = MissionState.INIT
        self.target_altitude = 2.0

        # Speed scheduling (MISSION_INTERFACE 7a): slow the legs that end in a
        # stop or turn so braking authority is there when it is needed. The
        # scale (percent of cruise) is recomputed each cruising tick; the MCU
        # applies effective_cruise = cruise * scale / 100.
        self.scale_transit = scale_transit
        self.scale_final_leg = scale_final_leg
        self.scale_hint = scale_hint
        self.hint_slow_range_m = hint_slow_range_m
        # True from a settle exit until the next node advance (the first leg
        # after any settle runs slow).
        self._first_leg_after_settle = False
        # Latest front-camera marker hint (id, node, ground distance ahead).
        self._front_hint_id: Optional[int] = None
        self._front_hint_node: Optional[Node] = None
        self._front_hint_distance: Optional[float] = None

        # Turn-settle: after a node advance that changes the travel axis (or
        # reverses it), hold on the node until the DR speed bleeds off so
        # transit momentum cannot slide the drone onto the next row's line.
        self.settle_speed_mps = settle_speed_mps
        self.settle_timeout_s = settle_timeout_s
        self.settle_min_s = settle_min_s
        # None means no settle is active; otherwise the mode to resume once it
        # completes (the state's normal cruise mode).
        self._settle_until_mode: Optional[ControlMode] = None
        self._settle_started_t: Optional[float] = None

        # Lost-line recovery: when a cruising leg loses the followed line's
        # presence bit for lost_line_timeout_s, synthesize a virtual line from
        # DR toward the believed row and creep (speed_scale 0) until the real
        # line returns for recovery_reacquire_ticks consecutive frames.
        # recovery_timeout_s without reacquisition -> FAILSAFE.
        self.lost_line_timeout_s = lost_line_timeout_s
        self.recovery_reacquire_ticks = recovery_reacquire_ticks
        self.recovery_timeout_s = recovery_timeout_s
        # When the needed line first went absent (None = present or reset).
        self._lost_line_since: Optional[float] = None
        self._recovery_active = False
        self._recovery_started_t: Optional[float] = None
        self._recovery_reacquire_count = 0
        # One [RECOVER] unavailable log per continuous-absence episode.
        self._recovery_dr_none_logged = False

        self.grid_map = grid_map if grid_map is not None else GridMap(
            node_count_x=11, node_count_y=8, cell_size_m=3.0, logger=logger
        )
        self.path_planner = PathPlanner(logger=logger)
        self.exploration_planner = ExplorationPlanner()

        self.current_node = Node(0, 0)
        self.home_node = Node(0, 0)
        self.move_direction = MoveDirection.X_POS
        self.grid_map.mark_node_visited(self.current_node)

        self.rescue_path: List[Node] = []
        self.path_index = 0

        # 3 s marker confirmation (majority vote over the hover window).
        # detected_ids_during_hover votes the id; marker_node_votes votes,
        # per id, the marker's own projected grid node. Recording a
        # projection farther than snap_max_err from any node is refused
        # (markers sit on intersections, so a far projection is untrusted).
        self.marker_confirm_start_time: Optional[float] = None
        self.detected_ids_during_hover: List[int] = []
        self.marker_node_votes: Dict[int, List[Node]] = {}
        self.required_marker_count = required_marker_count
        self.snap_max_err = snap_max_err

        self._seq = 0
        self._last_dr: Tuple[Optional[float], Optional[float]] = (None, None)

    # -------- front-camera hint (speed scheduling only) -------------------

    def set_front_hint(
        self,
        marker_id: Optional[int],
        node: Optional[Node],
        distance_m: Optional[float],
    ) -> None:
        """Store the latest front-camera marker hint (id, grid node, ground
        distance ahead) used only for speed scheduling. A None id clears the
        hint; a hint for an already recorded marker is ignored."""

        if marker_id is None:
            self._front_hint_id = None
            self._front_hint_node = None
            self._front_hint_distance = None
            return
        if self.grid_map.contains_marker(marker_id):
            return
        self._front_hint_id = marker_id
        self._front_hint_node = node
        self._front_hint_distance = distance_m

    # -------- logging helpers ---------------------------------------------

    def _context_str(self) -> str:
        nx, ny = self.grid_map.node_world(self.current_node)
        dx, dy = self._last_dr
        dr = f"({dx:.2f}, {dy:.2f})m" if dx is not None and dy is not None else "(none)"
        return (
            f"node=({self.current_node.x}, {self.current_node.y}) "
            f"nominal=({nx:.2f}, {ny:.2f})m dr={dr}"
        )

    def change_state(self, new_state: MissionState) -> None:
        self._log(
            f"[STATE] {self.state.name}({int(self.state)}) -> "
            f"{new_state.name}({int(new_state)}) {self._context_str()}"
        )
        self.state = new_state

    def _snap_node_from_dr(self) -> Node:
        """Node nearest the DR estimate; current_node when DR is None."""

        dx, dy = self._last_dr
        if dx is None or dy is None:
            return self.current_node
        return self.grid_map.nearest_node(dx, dy)

    def _marker_node_from_projection(self, aruco: ArucoDetection) -> Optional[Node]:
        """Grid node nearest the marker's own projected world position, or
        None when DR is unavailable or the projection sits farther than
        snap_max_err from any node (the trackers' snap tolerance)."""

        dr_x, dr_y = self._last_dr
        if dr_x is None or dr_y is None:
            return None
        # center errors are body-frame meters; yaw is locked to the grid
        # axes, so body axes == world axes and these add directly.
        mx = dr_x + aruco.center_error_x
        my = dr_y + aruco.center_error_y
        node = self.grid_map.nearest_node(mx, my)
        nx, ny = self.grid_map.node_world(node)
        if hypot(nx - mx, ny - my) > self.snap_max_err:
            return None
        return node

    def _finish_marker_confirm(self) -> None:
        """Close the confirm window. Re-zero current_node to the drone's DR
        node (independent of the marker — the drone can sit a cell away after
        braking), then record the majority id at the marker's own
        majority-voted node. Reject when no id was seen, or, with DR, when the
        winning id has no in-tolerance node vote. Without DR, fall back to
        recording at the drone's current node (degraded but defined)."""

        self.current_node = self._snap_node_from_dr()
        # A re-zero can leave move_direction pointing off-grid (braking may
        # have parked the drone on an edge node). Re-choose it from the new
        # node so the next pulse advances into the grid, as ENTER_GRID does.
        self.move_direction = self.exploration_planner.choose_direction(
            self.grid_map, self.current_node, self.move_direction
        )

        if not self.detected_ids_during_hover:
            self._log("[MARKER] confirmation failed: no id seen")
            return
        confirmed_id = Counter(self.detected_ids_during_hover).most_common(1)[0][0]

        if self._last_dr[0] is None or self._last_dr[1] is None:
            self.grid_map.save_marker(confirmed_id, self.current_node)
            return

        node_votes = self.marker_node_votes.get(confirmed_id)
        if not node_votes:
            self._log(f"[MARKER] confirmation failed: id {confirmed_id} no node vote")
            return
        marker_node = Counter(node_votes).most_common(1)[0][0]
        self.grid_map.save_marker(confirmed_id, marker_node)

    def _entry_node_from_dr(self) -> Node:
        """Grid-entry node from DR: behind the drone along move_direction
        on the travel axis, nearest on the perpendicular axis. current_node
        when DR is None."""

        dx, dy = self._last_dr
        if dx is None or dy is None:
            return self.current_node
        return self.grid_map.entry_node(dx, dy, self.move_direction)

    # -------- turn settle -------------------------------------------------

    def _begin_settle(self, now: float, resume_mode: ControlMode) -> None:
        """Park on the just-counted node and brake before the new leg. Records
        the start time; leaves current_node and move_direction untouched."""

        self._settle_until_mode = resume_mode
        self._settle_started_t = now
        # A settle breaks lost-line continuity: the pulse that triggered it
        # means the line was just seen.
        self._reset_recovery()
        self._log(f"[SETTLE] enter dir={self.move_direction.name} {self._context_str()}")

    def _settle_complete(self, now: float, sensors: SensorData) -> bool:
        """True once braked enough to cruise: DR body speed below
        settle_speed_mps, or the timeout fired. When no velocity estimate is
        available, fall back to a fixed minimum hold."""

        elapsed = now - self._settle_started_t
        if elapsed >= self.settle_timeout_s:
            return True
        if sensors.vx_est is None or sensors.vy_est is None:
            return elapsed >= self.settle_min_s
        return hypot(sensors.vx_est, sensors.vy_est) < self.settle_speed_mps

    def _settling_mode(self, now: float, sensors: SensorData) -> Optional[ControlMode]:
        """Drive an active settle: HOLD while braking, the resume mode on the
        tick it completes, None when no settle is active. Intersection pulses
        are ignored while this returns non-None (the caller returns early)."""

        if self._settle_until_mode is None:
            return None
        if not self._settle_complete(now, sensors):
            return ControlMode.HOLD
        resume = self._settle_until_mode
        self._settle_until_mode = None
        self._settle_started_t = None
        # The leg that starts here runs slow until the next node advance.
        self._first_leg_after_settle = True
        self._log(f"[SETTLE] exit dir={self.move_direction.name} {self._context_str()}")
        return resume

    # -------- lost-line recovery ------------------------------------------

    def _needed_line_present(self, perception: PerceptionData) -> bool:
        """Presence bit for the line the current leg follows: the vertical
        line for +/-x travel, the horizontal line for +/-y travel."""

        if self.move_direction in (MoveDirection.X_POS, MoveDirection.X_NEG):
            return perception.line.has_vertical
        return perception.line.has_horizontal

    def _reset_recovery(self) -> None:
        """Clear all lost-line tracking and any active recovery."""

        self._lost_line_since = None
        self._recovery_active = False
        self._recovery_started_t = None
        self._recovery_reacquire_count = 0
        self._recovery_dr_none_logged = False

    def _recovery_mode(
        self, now: float, perception: PerceptionData
    ) -> Optional[ControlMode]:
        """Drive lost-line recovery for the current cruising leg. Returns
        FOLLOW_LINE while recovering (make_mcu_command then synthesizes the
        line from DR and forces speed_scale 0), EMERGENCY_LAND on the recovery
        timeout, or None when normal processing should continue."""

        line_present = self._needed_line_present(perception)

        if self._recovery_active:
            # Reacquire needs recovery_reacquire_ticks CONSECUTIVE real frames;
            # a single absent frame resets the count.
            if line_present:
                self._recovery_reacquire_count += 1
                if self._recovery_reacquire_count >= self.recovery_reacquire_ticks:
                    self._log(f"[RECOVER] exit {self._context_str()}")
                    self._reset_recovery()
                    return None
            else:
                self._recovery_reacquire_count = 0
            if now - self._recovery_started_t >= self.recovery_timeout_s:
                self._log(f"[RECOVER] timeout -> FAILSAFE {self._context_str()}")
                self._reset_recovery()
                self.change_state(MissionState.FAILSAFE)
                return ControlMode.EMERGENCY_LAND
            return ControlMode.FOLLOW_LINE

        # Not recovering: track how long the needed line has been absent.
        if line_present:
            self._lost_line_since = None
            self._recovery_dr_none_logged = False
            return None
        if self._lost_line_since is None:
            self._lost_line_since = now
            return None
        if now - self._lost_line_since < self.lost_line_timeout_s:
            return None

        # Absent past the timeout: enter recovery unless DR is unavailable.
        if self._last_dr[0] is None or self._last_dr[1] is None:
            if not self._recovery_dr_none_logged:
                self._log(f"[RECOVER] unavailable (dr=none) {self._context_str()}")
                self._recovery_dr_none_logged = True
            return None
        self._recovery_active = True
        self._recovery_started_t = now
        self._recovery_reacquire_count = 0
        self._log(f"[RECOVER] enter {self._context_str()}")
        return ControlMode.FOLLOW_LINE

    # -------- main tick ---------------------------------------------------

    def update(
        self, now: float, sensors: SensorData, perception: PerceptionData
    ) -> ControlMode:
        """One mission tick. Returns the ControlMode to command."""

        self._last_dr = (sensors.dr_x, sensors.dr_y)

        # Failsafe checks run first, every tick.
        if not sensors.imu_ok or not sensors.lidar_ok:
            self.change_state(MissionState.FAILSAFE)
        if not sensors.rc_connected:
            self.change_state(MissionState.FAILSAFE)
        if sensors.battery_voltage < 14.0:
            self.change_state(MissionState.FAILSAFE)

        if self.state == MissionState.INIT:
            self.change_state(MissionState.TAKEOFF)
            return ControlMode.HOLD

        if self.state == MissionState.TAKEOFF:
            # Climb centered on the vertiport marker until at altitude.
            if sensors.altitude > 1.8:
                self.change_state(MissionState.LOCALIZE)
            return ControlMode.ALIGN_MARKER

        if self.state == MissionState.LOCALIZE:
            localization_converged = True  # placeholder for the filter check
            if localization_converged:
                self.change_state(MissionState.ENTER_GRID)
            return ControlMode.MOVE_TO_LANDMARK

        if self.state == MissionState.ENTER_GRID:
            # First grid line found: snap the node index from DR (one-time
            # init), pick a safe first direction, then start exploring.
            # The entry snap takes the node BEHIND the drone on the travel
            # axis so the first pulse advances onto the line actually ahead.
            if perception.line.has_vertical or perception.line.has_horizontal:
                start = self._entry_node_from_dr() if self._last_dr[0] is not None else Node(0, 0)
                self.current_node = start
                self.home_node = start
                self.grid_map.mark_node_visited(start)
                self.move_direction = self.exploration_planner.choose_direction(
                    self.grid_map, start, self.move_direction
                )
                self.change_state(MissionState.EXPLORE)
            return ControlMode.SEARCH_LINE

        if self.state == MissionState.EXPLORE:
            # 0. Settle the previous turn before doing anything else, so pulses
            #    that arrive while parked on the counted node are ignored.
            settle_mode = self._settling_mode(now, sensors)
            if settle_mode is not None:
                return settle_mode

            # 0b. Lost-line recovery: creep toward the believed row when the
            #     followed line's presence bit has been absent too long.
            recovery_mode = self._recovery_mode(now, perception)
            if recovery_mode is not None:
                return recovery_mode

            # 1. A new, unseen marker starts the 3 s confirmation.
            if (
                perception.aruco.detected
                and perception.aruco.marker_id is not None
                and not self.grid_map.contains_marker(perception.aruco.marker_id)
            ):
                self._reset_recovery()  # leaving the cruise leg for the confirm
                self.marker_confirm_start_time = now
                self.detected_ids_during_hover = []
                self.marker_node_votes = {}
                self.change_state(MissionState.MARKER_CONFIRM)
                return ControlMode.ALIGN_MARKER

            # 2. An intersection pulse advances one node and picks the next
            #    direction (guaranteed in-bounds by the serpentine planner).
            if perception.intersection.detected:
                prev_node = self.current_node
                next_node = move_to_next_node(prev_node, self.move_direction)
                # Defensive: a stale outward direction must never step off the
                # grid. Drop the pulse, re-choose a valid direction, and wait
                # for the next crossing.
                if not self.grid_map.in_bounds(next_node):
                    self._log(
                        f"[GRID] off-grid pulse dropped dir={self.move_direction.name} "
                        f"{self._context_str()}"
                    )
                    self.move_direction = self.exploration_planner.choose_direction(
                        self.grid_map, prev_node, self.move_direction
                    )
                    return ControlMode.FOLLOW_LINE
                self.current_node = next_node
                self._first_leg_after_settle = False  # advanced past the first leg
                self.grid_map.add_edge(prev_node, self.current_node)
                self.grid_map.mark_edge_visited(prev_node, self.current_node)
                prev_direction = self.move_direction
                self.move_direction = self.exploration_planner.choose_direction(
                    self.grid_map, self.current_node, self.move_direction
                )
                self._log(
                    f"[GRID] move_direction={self.move_direction.name}"
                    f"({int(self.move_direction)}) {self._context_str()}"
                )
                # A turn (axis change or same-axis reversal) settles first; a
                # straight-through advance keeps cruising the row.
                if self.move_direction != prev_direction:
                    self._begin_settle(now, ControlMode.FOLLOW_LINE)
                    return ControlMode.HOLD

            # 3. Enough markers found: plan the rescue route.
            if self.grid_map.marker_count() >= self.required_marker_count:
                self.change_state(MissionState.PLAN_RESCUE_PATH)
                return ControlMode.HOLD

            return ControlMode.FOLLOW_LINE

        if self.state == MissionState.MARKER_CONFIRM:
            # Vote the id (majority) and, per id, the marker's own projected
            # grid node over the hover window. The marker node comes from the
            # marker's projection, not the drone's DR: braking into the confirm
            # can leave the drone a cell past the marker.
            if perception.aruco.detected and perception.aruco.marker_id is not None:
                marker_id = perception.aruco.marker_id
                self.detected_ids_during_hover.append(marker_id)
                node = self._marker_node_from_projection(perception.aruco)
                if node is not None:
                    self.marker_node_votes.setdefault(marker_id, []).append(node)

            elapsed = now - self.marker_confirm_start_time
            if elapsed >= 3.0:
                self._finish_marker_confirm()
                self.change_state(MissionState.EXPLORE)
            return ControlMode.ALIGN_MARKER

        if self.state == MissionState.PLAN_RESCUE_PATH:
            self.rescue_path = self.path_planner.build_rescue_path(
                self.grid_map, self.current_node, self.home_node
            )
            self.path_index = 0
            self._log(f"[PATH] rescue path = {self.rescue_path}")
            if not self.rescue_path:
                self.change_state(MissionState.FAILSAFE)
            else:
                self.change_state(MissionState.FOLLOW_RESCUE_PATH)
            return ControlMode.HOLD

        if self.state == MissionState.FOLLOW_RESCUE_PATH:
            # Settle the previous turn first; pulses while parked are ignored.
            settle_mode = self._settling_mode(now, sensors)
            if settle_mode is not None:
                return settle_mode

            if self.path_index >= len(self.rescue_path):
                self.change_state(MissionState.LAND)
                return ControlMode.HOLD

            target_node = self.rescue_path[self.path_index]
            self.move_direction = direction_to_adjacent_node(self.current_node, target_node)

            # Lost-line recovery uses the current leg's travel axis, set above.
            recovery_mode = self._recovery_mode(now, perception)
            if recovery_mode is not None:
                return recovery_mode

            if perception.intersection.detected:
                # Defensive: never advance off-grid. move_direction already
                # points at the waypoint (line above); if that step would still
                # leave the grid, drop the pulse rather than raise.
                if not self.grid_map.in_bounds(
                    move_to_next_node(self.current_node, self.move_direction)
                ):
                    self._log(
                        f"[GRID] off-grid pulse dropped dir={self.move_direction.name} "
                        f"{self._context_str()}"
                    )
                    return ControlMode.FOLLOW_LINE
                prev_direction = self.move_direction
                self.current_node = move_to_next_node(self.current_node, self.move_direction)
                self._first_leg_after_settle = False  # advanced past the first leg
                self._log(
                    f"[FOLLOW] target=({target_node.x}, {target_node.y}) "
                    f"move_direction={self.move_direction.name}"
                    f"({int(self.move_direction)}) {self._context_str()}"
                )
                if self.current_node == target_node:
                    self.path_index += 1
                    # If the next leg turns off this axis (or reverses), brake
                    # on the corner before cruising it.
                    if self.path_index < len(self.rescue_path):
                        next_dir = direction_to_adjacent_node(
                            self.current_node, self.rescue_path[self.path_index]
                        )
                        if next_dir != prev_direction:
                            self.move_direction = next_dir
                            self._begin_settle(now, ControlMode.FOLLOW_LINE)
                            return ControlMode.HOLD
            return ControlMode.FOLLOW_LINE

        if self.state == MissionState.RETURN_HOME:
            # build_rescue_path already returns to home, so this is unused
            # in the current flow; kept for the full state enum.
            return ControlMode.FOLLOW_LINE

        if self.state == MissionState.LAND:
            if sensors.altitude < 0.2:
                self.change_state(MissionState.FINISHED)
            return ControlMode.LAND_ON_MARKER

        if self.state == MissionState.FINISHED:
            return ControlMode.STOP

        if self.state == MissionState.FAILSAFE:
            return ControlMode.EMERGENCY_LAND

        return ControlMode.HOLD

    # -------- speed scheduling --------------------------------------------

    def _compute_speed_scale(self) -> int:
        """Cruise speed scale (percent) for the current leg (MISSION_INTERFACE
        7a). 100 outside the cruising states; otherwise the lowest applicable
        slow-down so any leg that ends in a stop or turn is already slow."""

        if self.state not in (MissionState.EXPLORE, MissionState.FOLLOW_RESCUE_PATH):
            return 100

        scale = 100
        # Transit legs (Y travel between rows) and the first leg after a settle.
        if self.move_direction in (MoveDirection.Y_POS, MoveDirection.Y_NEG):
            scale = min(scale, self.scale_transit)
        if self._first_leg_after_settle:
            scale = min(scale, self.scale_transit)

        # Final leg before a row end: the next node is the last in-bounds one
        # in the travel direction (the node past it is out of bounds).
        next_node = move_to_next_node(self.current_node, self.move_direction)
        if self.grid_map.in_bounds(next_node) and not self.grid_map.in_bounds(
            move_to_next_node(next_node, self.move_direction)
        ):
            scale = min(scale, self.scale_final_leg)

        # Front-camera marker hint projected within range ahead on this row.
        if (
            self._front_hint_distance is not None
            and self._front_hint_distance <= self.hint_slow_range_m
        ):
            scale = min(scale, self.scale_hint)

        return scale

    # -------- command assembly / dispatch ---------------------------------

    def make_mcu_command(
        self, mode: ControlMode, perception: PerceptionData, sensors: SensorData
    ) -> McuCommand:
        """Bundle the mission state and vision errors into an McuCommand."""

        self._seq = (self._seq + 1) & 0xFF
        speed_scale = self._compute_speed_scale()

        # Lost-line recovery overrides the followed-line fields with a virtual
        # line synthesized from DR toward the believed row, and creeps at
        # speed_scale 0 (lateral correction only) until the real line returns.
        vertical_line = perception.line.has_vertical
        horizontal_line = perception.line.has_horizontal
        line_dx = perception.line.dx
        line_dy = perception.line.dy
        recovering = (
            self._recovery_active
            and self.state in (MissionState.EXPLORE, MissionState.FOLLOW_RESCUE_PATH)
            and self._last_dr[0] is not None
            and self._last_dr[1] is not None
        )
        if recovering:
            speed_scale = 0
            nom_x, nom_y = self.grid_map.node_world(self.current_node)
            dr_x, dr_y = self._last_dr
            if self.move_direction in (MoveDirection.X_POS, MoveDirection.X_NEG):
                line_dx = max(-2.0, min(2.0, nom_y - dr_y))  # clamp to the wire range
                vertical_line = True
            else:
                line_dy = max(-2.0, min(2.0, nom_x - dr_x))
                horizontal_line = True

        if speed_scale != 100:
            self._log(f"[{self.state.name}] scale={speed_scale} {self._context_str()}")
        vel_valid = sensors.vx_est is not None and sensors.vy_est is not None
        marker_id = perception.aruco.marker_id
        return McuCommand(
            mode=int(mode),
            mission_state=int(self.state),
            seq=self._seq,
            node_x=self.current_node.x,
            node_y=self.current_node.y,
            move_direction=int(self.move_direction),
            target_altitude=self.target_altitude,
            vertical_line=vertical_line,
            horizontal_line=horizontal_line,
            line_dx=line_dx,
            line_dy=line_dy,
            line_angle_error=perception.line.angle_error,
            line_confidence=perception.line.confidence,
            intersection_detected=perception.intersection.detected,
            intersection_forward=perception.intersection.forward,
            intersection_left=perception.intersection.left,
            intersection_right=perception.intersection.right,
            intersection_backward=perception.intersection.backward,
            marker_detected=perception.aruco.detected,
            marker_id=marker_id if marker_id is not None else -1,
            marker_error_x=perception.aruco.center_error_x,
            marker_error_y=perception.aruco.center_error_y,
            marker_yaw_error=perception.aruco.yaw_error,
            marker_confidence=perception.aruco.confidence,
            vx_est=sensors.vx_est if vel_valid else 0.0,
            vy_est=sensors.vy_est if vel_valid else 0.0,
            vel_est_valid=vel_valid,
            emergency=(mode == ControlMode.EMERGENCY_LAND),
            speed_scale=speed_scale,
        )

    def step(
        self, now: float, sensors: SensorData, perception: PerceptionData
    ) -> McuCommand:
        """Jetson main-loop entry: update state, build the command, send it."""

        mode = self.update(now=now, sensors=sensors, perception=perception)
        command = self.make_mcu_command(mode=mode, perception=perception, sensors=sensors)
        self.send_command_to_mcu(command)
        return command

    def send_command_to_mcu(self, command: McuCommand) -> None:
        """Overridable dispatch hook. The ROS node overrides this to publish
        the McuCommand; the default logs a one-line summary."""

        self._log(
            f"[MCU_CMD] mode={ControlMode(command.mode).name}({command.mode}) "
            f"state={MissionState(command.mission_state).name} seq={command.seq} "
            f"node=({command.node_x}, {command.node_y}) "
            f"dir={MoveDirection(command.move_direction).name} "
            f"line_dx={command.line_dx:.3f} line_dy={command.line_dy:.3f} "
            f"marker_id={command.marker_id} "
            f"vel_valid={command.vel_est_valid} emergency={command.emergency}"
        )
