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
from math import ceil, floor
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
    ):
        self._log = logger
        self.state = MissionState.INIT
        self.target_altitude = 2.0

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
        self.marker_confirm_start_time: Optional[float] = None
        self.detected_ids_during_hover: List[int] = []
        self.required_marker_count = required_marker_count

        self._seq = 0
        self._last_dr: Tuple[Optional[float], Optional[float]] = (None, None)

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

    def _entry_node_from_dr(self) -> Node:
        """Grid-entry node from DR: behind the drone along move_direction
        on the travel axis, nearest on the perpendicular axis. current_node
        when DR is None."""

        dx, dy = self._last_dr
        if dx is None or dy is None:
            return self.current_node
        return self.grid_map.entry_node(dx, dy, self.move_direction)

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
            # 1. A new, unseen marker starts the 3 s confirmation.
            if (
                perception.aruco.detected
                and perception.aruco.marker_id is not None
                and not self.grid_map.contains_marker(perception.aruco.marker_id)
            ):
                self.marker_confirm_start_time = now
                self.detected_ids_during_hover = []
                self.change_state(MissionState.MARKER_CONFIRM)
                return ControlMode.ALIGN_MARKER

            # 2. An intersection pulse advances one node and picks the next
            #    direction (guaranteed in-bounds by the serpentine planner).
            if perception.intersection.detected:
                prev_node = self.current_node
                self.current_node = move_to_next_node(prev_node, self.move_direction)
                self.grid_map.add_edge(prev_node, self.current_node)
                self.grid_map.mark_edge_visited(prev_node, self.current_node)
                self.move_direction = self.exploration_planner.choose_direction(
                    self.grid_map, self.current_node, self.move_direction
                )
                self._log(
                    f"[GRID] move_direction={self.move_direction.name}"
                    f"({int(self.move_direction)}) {self._context_str()}"
                )

            # 3. Enough markers found: plan the rescue route.
            if self.grid_map.marker_count() >= self.required_marker_count:
                self.change_state(MissionState.PLAN_RESCUE_PATH)
                return ControlMode.HOLD

            return ControlMode.FOLLOW_LINE

        if self.state == MissionState.MARKER_CONFIRM:
            # Accumulate IDs for the majority vote over the hover window.
            if perception.aruco.detected and perception.aruco.marker_id is not None:
                self.detected_ids_during_hover.append(perception.aruco.marker_id)

            elapsed = now - self.marker_confirm_start_time
            if elapsed >= 3.0:
                if self.detected_ids_during_hover:
                    confirmed_id = Counter(self.detected_ids_during_hover).most_common(1)[0][0]
                    # The marker sits ON an intersection, so re-zero the node
                    # index against DR and record the marker there.
                    snap_node = self._snap_node_from_dr()
                    self.current_node = snap_node
                    self.grid_map.save_marker(confirmed_id, snap_node)
                else:
                    self._log("[MARKER] confirmation failed")
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
            if self.path_index >= len(self.rescue_path):
                self.change_state(MissionState.LAND)
                return ControlMode.HOLD

            target_node = self.rescue_path[self.path_index]
            self.move_direction = direction_to_adjacent_node(self.current_node, target_node)

            if perception.intersection.detected:
                self.current_node = move_to_next_node(self.current_node, self.move_direction)
                self._log(
                    f"[FOLLOW] target=({target_node.x}, {target_node.y}) "
                    f"move_direction={self.move_direction.name}"
                    f"({int(self.move_direction)}) {self._context_str()}"
                )
                if self.current_node == target_node:
                    self.path_index += 1
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

    # -------- command assembly / dispatch ---------------------------------

    def make_mcu_command(
        self, mode: ControlMode, perception: PerceptionData, sensors: SensorData
    ) -> McuCommand:
        """Bundle the mission state and vision errors into an McuCommand."""

        self._seq = (self._seq + 1) & 0xFF
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
            vertical_line=perception.line.has_vertical,
            horizontal_line=perception.line.has_horizontal,
            line_dx=perception.line.dx,
            line_dy=perception.line.dy,
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
