"""
mission_skeleton.py

목표:
- 전체 미션 알고리즘의 틀을 직관적으로 보기 위한 Python skeleton.
- 실제 드론 제어 코드가 아니라, 나중에 C++/STM32 코드로 옮길 구조를 잡기 위한 코드.
"""

from dataclasses import dataclass
from enum import IntEnum
from collections import deque, Counter
import time


# ============================================================
# 1. 기본 자료형
# ============================================================


class MissionState(IntEnum):
    """
    Mission state values used in Jetson ↔ STM32 communication.

    숫자 값을 명시적으로 고정한다.
    통신 로그에서 state=4처럼 찍혀도 바로 의미를 알 수 있게 하기 위함.
    """

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


class MoveDirection(IntEnum):
    """
    Fixed-yaw body-frame movement direction.

    드론 자세는 고정되어 있고,
    body 좌표계 기준으로 어느 방향으로 이동할지를 나타낸다.
    """

    X_POS = 0   # body +x 방향 이동
    X_NEG = 1   # body -x 방향 이동
    Y_POS = 2   # body +y 방향 이동
    Y_NEG = 3   # body -y 방향 이동


@dataclass(frozen=True)
class Node:
    """격자 교차점 좌표. Node(1, 0)은 동쪽으로 한 칸, 즉 3m 이동한 위치."""
    x: int
    y: int


@dataclass
class LineDetection:
    visible: bool = False
    lateral_error: float = 0.0   # 선 중심이 화면 중앙에서 얼마나 벗어났는가
    angle_error: float = 0.0     # 선 방향과 드론 heading의 각도 오차
    confidence: float = 0.0


@dataclass
class IntersectionDetection:
    detected: bool = False
    forward: bool = False
    left: bool = False
    right: bool = False
    backward: bool = False


@dataclass
class ArucoDetection:
    detected: bool = False
    marker_id: int | None = None
    center_error_x: float = 0.0
    center_error_y: float = 0.0
    yaw_error: float = 0.0
    confidence: float = 0.0


@dataclass
class PerceptionData:
    line: LineDetection
    intersection: IntersectionDetection
    aruco: ArucoDetection


@dataclass
class SensorData:
    altitude: float
    battery_voltage: float
    imu_ok: bool
    lidar_ok: bool
    rc_connected: bool


@dataclass
class McuCommand:
    """
    Jetson이 MCU로 보내는 high-level command.

    Jetson은 제어루프를 돌려 vx, vy, vz, yaw_rate를 계산하지 않는다.
    대신 MCU가 내부 제어루프를 돌릴 수 있도록
    현재 미션 상태, 모드, 목표 고도, 라인 오차, 마커 중심 오차를 전달한다.
    """

    mode: str

    # 공통 목표값
    target_altitude: float = 2.0

    # mission / navigation 정보
    mission_state: int = 0
    current_node_x: int = 0
    current_node_y: int = 0

    # body-frame movement command
    move_direction: int = 0
    body_x_command: int = 0
    body_y_command: int = 0

    # line tracing용 정보
    line_visible: bool = False
    line_lateral_error: float = 0.0
    line_angle_error: float = 0.0
    line_confidence: float = 0.0

    # intersection 정보
    intersection_detected: bool = False
    intersection_forward: bool = False
    intersection_left: bool = False
    intersection_right: bool = False
    intersection_backward: bool = False

    # ArUco / marker alignment용 정보
    marker_detected: bool = False
    marker_id: int | None = None
    marker_error_x: float = 0.0
    marker_error_y: float = 0.0
    marker_yaw_error: float = 0.0
    marker_confidence: float = 0.0

    # 안전 관련
    emergency: bool = False


# ============================================================
# 2. 방향 관련 유틸 함수
# ============================================================

def move_direction_vector(direction: MoveDirection) -> tuple[int, int]:
    """
    MoveDirection을 body/grid 좌표계의 단위 이동 벡터로 바꾼다.

    반환값:
        (dx, dy)

    여기서는 body x/y축과 grid x/y축이 정렬되어 있다고 가정한다.
    나중에 축 정의가 바뀌면 이 함수만 수정하면 된다.
    """

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
    """
    현재 이동 방향으로 교차점 하나를 지났다고 보고
    grid node를 업데이트한다.
    """

    dx, dy = move_direction_vector(direction)
    return Node(node.x + dx, node.y + dy)


def direction_to_adjacent_node(current: Node, target: Node) -> MoveDirection:
    """
    current에서 인접한 target으로 가기 위한 이동 방향을 반환한다.
    target은 current의 상하좌우 이웃이어야 한다.
    """

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


# ============================================================
# 3. 격자 지도
# ============================================================

@dataclass
class GridNodeInfo:
    """
    Full-scale grid에서 각 교차점이 가지는 정보.

    GridMap이 node 방문 정보와 marker 정보를 함께 관리한다.
    """

    node: Node

    # 방문 여부
    visited: bool = False

    # ArUco marker 정보
    has_marker: bool = False
    marker_id: int | None = None
    marker_confirmed: bool = False

    # 마커를 몇 번째로 발견했는지 기록한다.
    marker_order: int | None = None


class GridMap:
    """
    Full-scale preallocated grid map.

    처음부터 전체 grid node와 edge를 만들어둔다.
    이후에는 node / edge / marker 관련 flag만 바꾼다.

    기본값:
        24m x 15m 미션장, 3m 간격 grid라고 하면
        cell 수는 8 x 5,
        교차점 node 수는 9 x 6.
    """

    def __init__(self, node_count_x: int = 9, node_count_y: int = 6):
        self.node_count_x = node_count_x
        self.node_count_y = node_count_y

        # nodes[y][x] 형태로 모든 node를 사전할당한다.
        self.nodes = [
            [
                GridNodeInfo(node=Node(x, y))
                for x in range(self.node_count_x)
            ]
            for y in range(self.node_count_y)
        ]

        # 모든 인접 edge의 visited flag를 사전할당한다.
        self.edge_visited_flags = {}

        # marker 정보도 GridMap 안에서 함께 관리한다.
        self.marker_id_to_node: dict[int, Node] = {}
        self.next_marker_order = 0

        for y in range(self.node_count_y):
            for x in range(self.node_count_x):
                node = Node(x, y)

                for neighbor in self.neighbors(node):
                    key = self.edge_key(node, neighbor)

                    if key not in self.edge_visited_flags:
                        self.edge_visited_flags[key] = False

    def in_bounds(self, node: Node) -> bool:
        """
        node가 grid 범위 안에 있는지 확인한다.
        """

        return (
            0 <= node.x < self.node_count_x
            and 0 <= node.y < self.node_count_y
        )

    def validate_node(self, node: Node):
        """
        범위 밖 node를 사용하면 바로 에러를 낸다.
        """

        if not self.in_bounds(node):
            raise ValueError(f"Node out of grid bounds: {node}")

    def node_info(self, node: Node) -> GridNodeInfo:
        """
        특정 node의 정보를 반환한다.
        """

        self.validate_node(node)
        return self.nodes[node.y][node.x]

    def mark_node_visited(self, node: Node):
        """
        특정 교차점을 방문했다고 표시한다.
        """

        self.node_info(node).visited = True

    def edge_key(self, a: Node, b: Node):
        """
        edge를 방향과 무관하게 저장하기 위한 key.

        a -> b와 b -> a는 같은 edge이므로
        좌표 tuple을 정렬해서 하나의 key로 만든다.
        """

        self.validate_node(a)
        self.validate_node(b)

        p1 = (a.x, a.y)
        p2 = (b.x, b.y)

        return tuple(sorted([p1, p2]))

    def are_adjacent(self, a: Node, b: Node) -> bool:
        """
        두 node가 상하좌우로 인접한지 확인한다.
        """

        dx = abs(a.x - b.x)
        dy = abs(a.y - b.y)

        return dx + dy == 1

    def neighbors(self, node: Node):
        """
        grid 범위 안에 있는 상하좌우 이웃 node를 반환한다.
        """

        self.validate_node(node)

        result = []

        for direction in MoveDirection:
            dx, dy = move_direction_vector(direction)
            neighbor = Node(node.x + dx, node.y + dy)

            if self.in_bounds(neighbor):
                result.append(neighbor)

        return result

    def add_node(self, node: Node):
        """
        기존 코드와의 호환성을 위해 남겨둔다.

        full-scale grid에서는 node를 새로 추가하지 않는다.
        범위 안 node인지 확인만 한다.
        """

        self.validate_node(node)

    def add_edge(self, a: Node, b: Node):
        """
        기존 코드와의 호환성을 위해 남겨둔다.

        full-scale grid에서는 edge를 새로 추가하지 않는다.
        두 node가 유효하고 인접한지만 확인한다.
        """

        self.validate_node(a)
        self.validate_node(b)

        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")

    def mark_edge_visited(self, a: Node, b: Node):
        """
        두 node 사이의 edge를 방문했다고 표시한다.
        """

        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")

        key = self.edge_key(a, b)
        self.edge_visited_flags[key] = True

        self.mark_node_visited(a)
        self.mark_node_visited(b)

    def edge_visited(self, a: Node, b: Node) -> bool:
        """
        두 node 사이 edge를 이미 방문했는지 확인한다.
        """

        if not self.are_adjacent(a, b):
            raise ValueError(f"Nodes are not adjacent: {a}, {b}")

        key = self.edge_key(a, b)
        return self.edge_visited_flags[key]

    def save_marker(self, marker_id: int, node: Node):
        """
        특정 node에 ArUco marker 정보를 저장한다.
        """

        self.validate_node(node)

        if marker_id in self.marker_id_to_node:
            print(f"[MARKER] ID {marker_id} already saved")
            return

        info = self.node_info(node)
        info.has_marker = True
        info.marker_id = marker_id
        info.marker_confirmed = True
        info.marker_order = self.next_marker_order

        self.marker_id_to_node[marker_id] = node
        self.next_marker_order += 1

        print(
            f"[MARKER] ID {marker_id} saved at {node}, "
            f"order={info.marker_order}"
        )

    def contains_marker(self, marker_id: int) -> bool:
        """
        해당 ArUco ID가 이미 저장되었는지 확인한다.
        """

        return marker_id in self.marker_id_to_node

    def marker_count(self) -> int:
        """
        현재까지 저장한 마커 개수.
        """

        return len(self.marker_id_to_node)

    def sorted_marker_ids(self):
        """
        구조 경로 생성을 위해 ArUco ID 순서대로 반환한다.
        """

        return sorted(self.marker_id_to_node.keys())

    def node_of_marker(self, marker_id: int) -> Node:
        """
        특정 ArUco ID가 있는 node를 반환한다.
        """

        return self.marker_id_to_node[marker_id]


# ============================================================
# 4. 경로 계획: BFS 최단경로
# ============================================================

class PathPlanner:
    def shortest_path(self, grid_map: GridMap, start: Node, goal: Node):
        """
        격자 그래프에서 BFS로 최단경로를 찾는다.
        모든 edge 길이가 같으므로 BFS면 충분하다.
        """

        queue = deque([start])
        parent = {start: None}

        while queue:
            current = queue.popleft()

            if current == goal:
                break

            for nxt in grid_map.neighbors(current):
                if nxt not in parent:
                    parent[nxt] = current
                    queue.append(nxt)

        if goal not in parent:
            print("[PATH] No path found")
            return []

        path = []
        cur = goal

        while cur is not None:
            path.append(cur)
            cur = parent[cur]

        path.reverse()
        return path

    def build_rescue_path(
        self,
        grid_map: GridMap,
        current_node: Node,
        home_node: Node,
    ):
        """
        구조 경로:
        현재 위치 → ID 작은 마커 → ID 큰 마커 → ... → 시작점

        마커 위치 정보는 GridMap 안의
        marker flag / marker_id_to_node에서 가져온다.
        """

        full_path = []
        pos = current_node

        for marker_id in grid_map.sorted_marker_ids():
            target = grid_map.node_of_marker(marker_id)
            segment = self.shortest_path(grid_map, pos, target)

            if not segment:
                return []

            # segment[0]은 현재 위치이므로 중복 방지
            full_path.extend(segment[1:])
            pos = target

        # 마지막 마커에서 home으로 복귀
        return_segment = self.shortest_path(grid_map, pos, home_node)

        if not return_segment:
            return []

        full_path.extend(return_segment[1:])
        return full_path


# ============================================================
# 6. 제어기 skeleton
# ============================================================

# ============================================================
# 7. 탐색 방향 선택
# ============================================================

class ExplorationPlanner:
    """
    탐색 단계에서 교차점에 도착했을 때
    다음 이동 방향을 결정한다.

    이번 revision에서는 TargetEstimator를 추가하지 않으므로,
    일단 현재 이동 방향을 유지한다.
    """

    def choose_direction(
        self,
        current_direction: MoveDirection,
        intersection: IntersectionDetection,
    ) -> MoveDirection:

        # TODO:
        # 나중에 TargetEstimator 또는 방문 여부 기반 탐색 로직으로 교체.
        # 지금은 가장 단순하게 기존 방향 유지.
        return current_direction


# ============================================================
# 8. 미션 매니저
# ============================================================

class MissionManager:
    """
    전체 상태머신.

    여기서는 직접 OpenCV나 모터를 건드리지 않는다.
    상태를 바꾸고, 어떤 제어 모드를 사용할지만 결정한다.
    """

    def __init__(self):
        self.state = MissionState.INIT
        self.target_altitude = 2.0

        # 전체 9 × 6 격자를 처음부터 생성
        self.grid_map = GridMap(
            node_count_x=9,
            node_count_y=6,
        )
        self.path_planner = PathPlanner()
        self.exploration_planner = ExplorationPlanner()

        self.current_node = Node(0, 0)
        self.home_node = Node(0, 0)
        self.move_direction = MoveDirection.X_POS

        # 시작 노드는 이미 방문한 것으로 표시
        self.grid_map.mark_node_visited(self.current_node)

        self.rescue_path: list[Node] = []
        self.path_index = 0

        # 3초 마커 확인용
        self.marker_confirm_start_time = None
        self.detected_ids_during_hover = []

        # 몇 개의 경로점 마커를 찾아야 하는가
        self.required_marker_count = 4


    def change_state(self, new_state: MissionState):
        print(
        f"[STATE] "
        f"{self.state.name}({int(self.state)}) "
        f"-> "
        f"{new_state.name}({int(new_state)})"
    )
        self.state = new_state

    def update(
        self,
        now: float,
        sensors: SensorData,
        perception: PerceptionData,
    ) -> str:
        """
        매 루프마다 호출된다.

        반환값은 지금 사용해야 할 제어 모드다.
        예:
            "TAKEOFF"
            "FOLLOW_LINE"
            "ALIGN_MARKER"
            "LAND"
        """

        # -----------------------------
        # Failsafe는 항상 먼저 확인
        # -----------------------------
        if not sensors.imu_ok or not sensors.lidar_ok:
            self.change_state(MissionState.FAILSAFE)

        if not sensors.rc_connected:
            self.change_state(MissionState.FAILSAFE)

        if sensors.battery_voltage < 14.0:
            self.change_state(MissionState.FAILSAFE)

        # -----------------------------
        # 상태별 로직
        # -----------------------------
        if self.state == MissionState.INIT:
            self.change_state(MissionState.TAKEOFF)
            return "HOLD"

        elif self.state == MissionState.TAKEOFF:
            # 실제로는 버티포트 ArUco 중심을 보면서 이륙
            if sensors.altitude > 1.8:
                self.change_state(MissionState.LOCALIZE)
            return "ALIGN_MARKER"

        elif self.state == MissionState.LOCALIZE:
            # 실제로는 landmark/particle filter 수렴 여부를 봄
            localization_converged = True

            if localization_converged:
                self.change_state(MissionState.ENTER_GRID)

            return "MOVE_TO_LANDMARK"

        elif self.state == MissionState.ENTER_GRID:
            # 실제로는 첫 격자선을 찾을 때까지 천천히 이동
            if perception.line.visible:
                self.change_state(MissionState.EXPLORE)

            return "SEARCH_LINE"

        elif self.state == MissionState.EXPLORE:
            # 1. 새로운 ArUco 마커를 발견하면 3초 확인 상태로 들어간다.
            if (
                perception.aruco.detected
                and perception.aruco.marker_id is not None
                and not self.grid_map.contains_marker(perception.aruco.marker_id)
            ):
                self.marker_confirm_start_time = now
                self.detected_ids_during_hover = []
                self.change_state(MissionState.MARKER_CONFIRM)
                return "ALIGN_MARKER"

            # 2. 교차점에 도착하면 현재 node 업데이트 및 방향 선택
            if perception.intersection.detected:
                prev_node = self.current_node
                self.current_node = move_to_next_node(
                    self.current_node,
                    self.move_direction,
                )

                self.grid_map.add_edge(prev_node, self.current_node)
                self.grid_map.mark_edge_visited(prev_node, self.current_node)

                next_direction = self.exploration_planner.choose_direction(
                    self.move_direction,
                    perception.intersection,
                )

                self.move_direction = next_direction

                print(
                    f"[GRID] current node = {self.current_node}, "
                    f"move_direction = {self.move_direction.name}({int(self.move_direction)})"
                )

            # 3. 마커 4개를 찾으면 구조 경로 계획
            if self.grid_map.marker_count() >= self.required_marker_count:
                self.change_state(MissionState.PLAN_RESCUE_PATH)
                return "HOLD"

            return "FOLLOW_LINE"

        elif self.state == MissionState.MARKER_CONFIRM:
            # 3초 동안 마커 ID를 계속 모은다.
            if perception.aruco.detected and perception.aruco.marker_id is not None:
                self.detected_ids_during_hover.append(perception.aruco.marker_id)

            elapsed = now - self.marker_confirm_start_time

            if elapsed >= 3.0:
                if len(self.detected_ids_during_hover) > 0:
                    confirmed_id = Counter(self.detected_ids_during_hover).most_common(1)[0][0]

                    # 탐색 단계에서는 어떤 ID든 저장하면 된다.
                    self.grid_map.save_marker(confirmed_id, self.current_node)
                else:
                    print("[MARKER] confirmation failed")

                self.change_state(MissionState.EXPLORE)

            return "ALIGN_MARKER"

        elif self.state == MissionState.PLAN_RESCUE_PATH:
            self.rescue_path = self.path_planner.build_rescue_path(
                self.grid_map,
                self.current_node,
                self.home_node,
            )

            self.path_index = 0

            print(f"[PATH] rescue path = {self.rescue_path}")

            if len(self.rescue_path) == 0:
                self.change_state(MissionState.FAILSAFE)
            else:
                self.change_state(MissionState.FOLLOW_RESCUE_PATH)

            return "HOLD"

        elif self.state == MissionState.FOLLOW_RESCUE_PATH:
            # 현재 path target
            if self.path_index >= len(self.rescue_path):
                self.change_state(MissionState.LAND)
                return "HOLD"

            target_node = self.rescue_path[self.path_index]

            self.move_direction = direction_to_adjacent_node(
                self.current_node,
                target_node,
            )

            # 교차점에 도착할 때마다 다음 노드로 진행했다고 판단
            if perception.intersection.detected:
                self.current_node = move_to_next_node(
                    self.current_node,
                    self.move_direction,
                )

                print(
                    f"[FOLLOW] current = {self.current_node}, "
                    f"target = {target_node}, "
                    f"move_direction = {self.move_direction.name}({int(self.move_direction)})"
                )

                if self.current_node == target_node:
                    self.path_index += 1

            return "FOLLOW_LINE"

        elif self.state == MissionState.RETURN_HOME:
            # 위 build_rescue_path에 home 복귀까지 포함했으므로
            # 지금 skeleton에서는 따로 사용하지 않아도 됨.
            return "FOLLOW_LINE"

        elif self.state == MissionState.LAND:
            if sensors.altitude < 0.2:
                self.change_state(MissionState.FINISHED)
            return "LAND_ON_MARKER"

        elif self.state == MissionState.FINISHED:
            return "STOP"

        elif self.state == MissionState.FAILSAFE:
            return "EMERGENCY_LAND"

        return "HOLD"


    def make_mcu_command(
        self,
        mode: str,
        perception: PerceptionData,
        sensors: SensorData,
    ) -> McuCommand:
        """
        현재 MissionManager 상태와 비전 결과를 묶어서
        MCU로 보낼 high-level command packet을 만든다.

        제어루프는 MCU 내부에서 돌고,
        Jetson/MissionManager는 mode, 목표 고도, 라인 오차,
        마커 중심 오차, body-frame 이동 방향만 전달한다.
        """

        body_x_command, body_y_command = move_direction_vector(
            self.move_direction
        )

        return McuCommand(
            mode=mode,
            target_altitude=self.target_altitude,

            mission_state=int(self.state),
            current_node_x=self.current_node.x,
            current_node_y=self.current_node.y,

            move_direction=int(self.move_direction),
            body_x_command=body_x_command,
            body_y_command=body_y_command,

            line_visible=perception.line.visible,
            line_lateral_error=perception.line.lateral_error,
            line_angle_error=perception.line.angle_error,
            line_confidence=perception.line.confidence,

            intersection_detected=perception.intersection.detected,
            intersection_forward=perception.intersection.forward,
            intersection_left=perception.intersection.left,
            intersection_right=perception.intersection.right,
            intersection_backward=perception.intersection.backward,

            marker_detected=perception.aruco.detected,
            marker_id=perception.aruco.marker_id,
            marker_error_x=perception.aruco.center_error_x,
            marker_error_y=perception.aruco.center_error_y,
            marker_yaw_error=perception.aruco.yaw_error,
            marker_confidence=perception.aruco.confidence,

            emergency=(mode == "EMERGENCY_LAND"),
        )

    def step(self, sensors: SensorData, perception: PerceptionData) -> McuCommand:
        """
        Jetson main loop에서 매번 호출되는 함수.

        MissionManager가 직접
        1. 현재 미션 상태를 업데이트하고,
        2. MCU로 보낼 command를 생성하고,
        3. 전송 함수로 넘긴다.

        따라서 별도의 FlightCore 클래스는 필요하지 않다.
        """

        now = time.time()

        mode = self.update(
            now=now,
            sensors=sensors,
            perception=perception,
        )

        command = self.make_mcu_command(
            mode=mode,
            perception=perception,
            sensors=sensors,
        )

        self.send_command_to_mcu(command)
        return command

    def send_command_to_mcu(self, command: McuCommand):
        """
        실제로는 여기서 UART / USB / CAN / MAVLink 등으로
        MCU에 command packet을 보낸다.

        지금은 skeleton이므로 print만 한다.
        """

        print(
            f"[MCU_CMD] "
            f"mode={command.mode}, "
            f"state={command.mission_state}, "
            f"move_direction={command.move_direction}, "
            f"body_cmd=({command.body_x_command}, {command.body_y_command}), "
            f"node=({command.current_node_x}, {command.current_node_y}), "
            f"target_alt={command.target_altitude:.2f}, "
            f"line_visible={command.line_visible}, "
            f"line_err={command.line_lateral_error:.3f}, "
            f"line_angle={command.line_angle_error:.3f}, "
            f"marker_detected={command.marker_detected}, "
            f"marker_id={command.marker_id}, "
            f"marker_err=({command.marker_error_x:.3f}, {command.marker_error_y:.3f}), "
            f"emergency={command.emergency}"
        )


# ============================================================
# 9. 전체 Flight Core
# ============================================================

# ============================================================
# 10. code test
# ============================================================

def fake_sensor(altitude=2.0):
    return SensorData(
        altitude=altitude,
        battery_voltage=15.5,
        imu_ok=True,
        lidar_ok=True,
        rc_connected=True,
    )


def fake_perception(
    line_visible=True,
    intersection=False,
    aruco_id=None,
):
    return PerceptionData(
        line=LineDetection(
            visible=line_visible,
            lateral_error=0.05,
            angle_error=-0.02,
            confidence=0.9,
        ),
        intersection=IntersectionDetection(
            detected=intersection,
            forward=True,
            left=True,
            right=True,
            backward=True,
        ),
        aruco=ArucoDetection(
            detected=(aruco_id is not None),
            marker_id=aruco_id,
            center_error_x=0.01,
            center_error_y=-0.02,
            yaw_error=0.03,
            confidence=0.95,
        ),
    )


if __name__ == "__main__":
    mission = MissionManager()

    # 실제 비행에서는 이 while문이 50Hz, 100Hz 등으로 계속 돈다.
    # 여기서는 직관적 테스트용으로 몇 step만 수동 실행한다.

    # 이륙
    mission.step(fake_sensor(altitude=0.5), fake_perception(line_visible=False))
    mission.step(fake_sensor(altitude=2.0), fake_perception(line_visible=False))

    # localization 후 grid 진입
    mission.step(fake_sensor(altitude=2.0), fake_perception(line_visible=False))
    mission.step(fake_sensor(altitude=2.0), fake_perception(line_visible=True))

    # 교차점 몇 개 통과
    mission.step(fake_sensor(), fake_perception(intersection=True))
    mission.step(fake_sensor(), fake_perception(intersection=True))

    # 마커 ID 3 발견
    mission.step(fake_sensor(), fake_perception(aruco_id=3))
    time.sleep(3.1)
    mission.step(fake_sensor(), fake_perception(aruco_id=3))

    # 마커 ID 1 발견
    mission.step(fake_sensor(), fake_perception(aruco_id=1))
    time.sleep(3.1)
    mission.step(fake_sensor(), fake_perception(aruco_id=1))

    # 마커 ID 4 발견
    mission.step(fake_sensor(), fake_perception(aruco_id=4))
    time.sleep(3.1)
    mission.step(fake_sensor(), fake_perception(aruco_id=4))

    # 마커 ID 2 발견
    mission.step(fake_sensor(), fake_perception(aruco_id=2))
    time.sleep(3.1)
    mission.step(fake_sensor(), fake_perception(aruco_id=2))

    # 이후 PLAN_RESCUE_PATH로 넘어감
    mission.step(fake_sensor(), fake_perception())
