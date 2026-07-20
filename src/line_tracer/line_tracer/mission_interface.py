"""Jetson<->MCU mission interface: the shared vocabulary and the wire.

This file is the Jetson<->MCU contract. A firmware engineer wiring a
Jetson to the STM32 reads this file first: it mirrors the fc_core C
headers `protocol.h` (byte layout, magic/version, Q14 and CRC16-CCITT
conventions) and `mission_ctrl.h` (how the MCU consumes the command),
expressed as plain Python. The enums, dataclasses, and direction
helpers here are the vocabulary both sides agree on; `pack_mcu_command`
serialises an `McuCommand` into the exact 34-byte mission frame the
STM32 decodes with `fc_proto_decode_mission`.

Two halves:
  1. Contract types. Enums have fixed integer values so a serial log
     reading state=4 / mode=1 is meaningful on both ends. The
     dataclasses carry plain metric Python values (meters, radians,
     0..1 confidences, marker_id -1 = none); no scaling lives on them.
  2. Wire codec (downlink only, companion -> FC). `pack_mcu_command`
     applies the fixed-point scaling and bit packing and appends the
     CRC. The layout is defined once in protocol.h and duplicated here
     only as encode logic — the byte offsets are NOT re-documented; read
     protocol.h for the offset table.

Fixed-point and framing (must match protocol.c byte for byte):
  - Q14 = float * 16384, clamped to the int16 rail, so the usable range
    is about -2.0 .. +1.99994 for the line/marker/velocity fields.
  - target_altitude travels as u16 centimetres, clamped 0..10 m.
  - Confidences 0..1 float -> u8 0..255 (round half up). This float->u8
    convention is defined here: the C struct stores the u8 verbatim, so
    the Jetson owns the scaling.
  - flags / flags2 bit layout mirrors the FC_PROTO_MFLAG_* macros.
  - CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) over bytes
    0..31, little-endian trailer at offset 32.

Uplink (FC -> companion telemetry) is future work: an `unpack_uplink`
mirroring `fc_proto_decode_up` would live here once the Jetson needs to
read FC state back. Only the downlink is implemented for now.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple


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
    (protocol.h mission frame). Values are plain Python: metric
    meters/radians, ControlMode/MissionState/MoveDirection as ints,
    marker_id -1 when none. pack_mcu_command owns the byte layout (Q14
    scaling, u8 confidences, the flags/flags2 bytes); the
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
# 3. Direction helpers (shared vocabulary, used by logic + tests)
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


# ============================================================
# 4. Wire constants (mirror protocol.h)
# ============================================================

FC_PROTO_MISSION_MAGIC = 0xA6
FC_PROTO_VERSION = 0x01
FC_PROTO_MISSION_LEN = 34

# Bit 7 of the mode byte requests arm; bits 0..6 carry ControlMode.
FC_PROTO_MODE_ARM_BIT = 0x80
FC_PROTO_MODE_MASK = 0x7F

# Mission flags byte (offset 29).
FC_PROTO_MFLAG_VERTICAL_LINE = 1 << 0
FC_PROTO_MFLAG_HORIZONTAL_LINE = 1 << 1
FC_PROTO_MFLAG_INTERSECTION = 1 << 2
FC_PROTO_MFLAG_FWD = 1 << 3
FC_PROTO_MFLAG_LEFT = 1 << 4
FC_PROTO_MFLAG_RIGHT = 1 << 5
FC_PROTO_MFLAG_BACK = 1 << 6
FC_PROTO_MFLAG_MARKER_DETECTED = 1 << 7

# Mission flags2 byte (offset 30).
FC_PROTO_MFLAG2_VEL_EST_VALID = 1 << 0
FC_PROTO_MFLAG2_EMERGENCY = 1 << 1

_Q14_SCALE = 16384.0


# ============================================================
# 5. Fixed-point + CRC helpers (mirror protocol.c)
# ============================================================


def fc_proto_crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT: poly 0x1021, init 0xFFFF, no input/output reflection.
    Byte-for-byte port of fc_proto_crc16_ccitt in protocol.c."""

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _f_to_q14(v: float) -> int:
    """Float -> Q14 int16, clamped to the int16 rail then truncated toward
    zero, matching the C (int16_t) cast."""

    s = float(v) * _Q14_SCALE
    if s > 32767.0:
        s = 32767.0
    if s < -32768.0:
        s = -32768.0
    return int(s)


def _m_to_cm(m: float) -> int:
    """Altitude metres -> u16 centimetres, clamped 0..10 m (round half up)."""

    cm = float(m) * 100.0
    if cm < 0.0:
        cm = 0.0
    if cm > 1000.0:
        cm = 1000.0
    return int(cm + 0.5)


def _conf_to_u8(c: float) -> int:
    """Confidence 0..1 float -> u8 0..255 (round half up)."""

    if c < 0.0:
        c = 0.0
    if c > 1.0:
        c = 1.0
    return int(c * 255.0 + 0.5)


def _clamp_i8(v: int) -> int:
    return max(-128, min(127, int(v)))


def _clamp_u8(v: int) -> int:
    return max(0, min(255, int(v)))


# Little-endian struct for mission-frame bytes 0..31 (the 32 payload bytes
# the CRC covers). Field order follows the protocol.h offset table exactly.
_MISSION_STRUCT = struct.Struct("<BBBBBbbBHhhhhhhhhbBBBBB")


# ============================================================
# 6. Downlink packer (companion -> FC)
# ============================================================


def pack_mcu_command(
    cmd: McuCommand, seq: Optional[int] = None, arm: bool = True
) -> bytes:
    """Serialise an McuCommand into the 34-byte mission frame.

    Produces exactly the layout protocol.h defines, so the bytes decode
    with fc_proto_decode_mission on the STM32. seq overrides cmd.seq when
    given (the serial link owns the running counter); arm sets bit 7 of
    the mode byte. Out-of-range values are clamped, not rejected: Q14 to
    the +/-2.0 rail, altitude to 0..10 m, confidences to 0..255,
    node/marker ids to int8, everything else to its byte width.
    """

    seq_val = cmd.seq if seq is None else seq
    mode_byte = (int(cmd.mode) & FC_PROTO_MODE_MASK) | (
        FC_PROTO_MODE_ARM_BIT if arm else 0
    )

    flags = 0
    if cmd.vertical_line:
        flags |= FC_PROTO_MFLAG_VERTICAL_LINE
    if cmd.horizontal_line:
        flags |= FC_PROTO_MFLAG_HORIZONTAL_LINE
    if cmd.intersection_detected:
        flags |= FC_PROTO_MFLAG_INTERSECTION
    if cmd.intersection_forward:
        flags |= FC_PROTO_MFLAG_FWD
    if cmd.intersection_left:
        flags |= FC_PROTO_MFLAG_LEFT
    if cmd.intersection_right:
        flags |= FC_PROTO_MFLAG_RIGHT
    if cmd.intersection_backward:
        flags |= FC_PROTO_MFLAG_BACK
    if cmd.marker_detected:
        flags |= FC_PROTO_MFLAG_MARKER_DETECTED

    flags2 = 0
    if cmd.vel_est_valid:
        flags2 |= FC_PROTO_MFLAG2_VEL_EST_VALID
    if cmd.emergency:
        flags2 |= FC_PROTO_MFLAG2_EMERGENCY

    buf = bytearray(FC_PROTO_MISSION_LEN)
    _MISSION_STRUCT.pack_into(
        buf,
        0,
        FC_PROTO_MISSION_MAGIC,
        FC_PROTO_VERSION,
        mode_byte & 0xFF,
        _clamp_u8(int(cmd.mission_state)),
        seq_val & 0xFF,
        _clamp_i8(cmd.node_x),
        _clamp_i8(cmd.node_y),
        _clamp_u8(int(cmd.move_direction)),
        _m_to_cm(cmd.target_altitude),
        _f_to_q14(cmd.line_dx),
        _f_to_q14(cmd.line_dy),
        _f_to_q14(cmd.line_angle_error),
        _f_to_q14(cmd.marker_error_x),
        _f_to_q14(cmd.marker_error_y),
        _f_to_q14(cmd.marker_yaw_error),
        _f_to_q14(cmd.vx_est),
        _f_to_q14(cmd.vy_est),
        _clamp_i8(cmd.marker_id),
        _conf_to_u8(cmd.line_confidence),
        _conf_to_u8(cmd.marker_confidence),
        flags,
        flags2,
        _clamp_u8(cmd.speed_scale),
    )

    crc = fc_proto_crc16_ccitt(bytes(buf[: FC_PROTO_MISSION_LEN - 2]))
    struct.pack_into("<H", buf, FC_PROTO_MISSION_LEN - 2, crc)
    return bytes(buf)
