"""Unit tests for line_tracer.mission_interface (the Jetson<->MCU wire).

The GOLDEN_FRAME below is a cross-language golden vector: the SAME 34
bytes are asserted in src/fc_core/test/test_protocol.cpp
(ProtocolMission.GoldenVectorFromJetson), where fc_proto_decode_mission
must accept them and recover the field values. Change the two together
or the Jetson->STM32 wire compatibility check is meaningless.
"""
from __future__ import annotations

import struct

import pytest

from line_tracer.mission_interface import (
    ControlMode,
    McuCommand,
    MissionState,
    MoveDirection,
    fc_proto_crc16_ccitt,
    pack_mcu_command,
)


# Golden vector shared with src/fc_core/test/test_protocol.cpp
# (ProtocolMission.GoldenVectorFromJetson) — change together.
GOLDEN_FRAME = bytes.fromhex(
    "a60181042a05fd02fa00002000d0660600f00008cdfc331367e607cc99ad03281eff"
)


def _golden_command() -> McuCommand:
    """The representative command that packs to GOLDEN_FRAME: non-trivial
    values in every field, both flags bytes non-zero, both i8 signs."""

    return McuCommand(
        mode=int(ControlMode.FOLLOW_LINE),        # 1
        mission_state=int(MissionState.EXPLORE),  # 4
        seq=42,
        node_x=5,
        node_y=-3,
        move_direction=int(MoveDirection.Y_POS),  # 2
        target_altitude=2.5,                      # -> 250 cm
        vertical_line=True,                       # flags bit0
        horizontal_line=False,
        line_dx=0.5,
        line_dy=-0.75,
        line_angle_error=0.1,
        line_confidence=0.8,                      # -> 204
        intersection_detected=True,               # flags bit2
        intersection_forward=True,                # flags bit3
        intersection_left=False,
        intersection_right=True,                  # flags bit5
        intersection_backward=False,
        marker_detected=True,                     # flags bit7
        marker_id=7,
        marker_error_x=-0.25,
        marker_error_y=0.125,
        marker_yaw_error=-0.05,
        marker_confidence=0.6,                    # -> 153
        vx_est=0.3,
        vy_est=-0.4,
        vel_est_valid=True,                       # flags2 bit0
        emergency=True,                           # flags2 bit1
        speed_scale=40,
    )


def test_pack_matches_golden_frame():
    """The packer reproduces the exact bytes shared with the C decoder."""
    frame = pack_mcu_command(_golden_command(), seq=42, arm=True)
    assert len(frame) == 34
    assert frame == GOLDEN_FRAME


def test_golden_frame_field_offsets():
    """Spot-check fields at their protocol.h byte offsets."""
    f = GOLDEN_FRAME
    assert f[0] == 0xA6                                   # magic
    assert f[1] == 0x01                                   # version
    assert f[2] == 0x81                                   # FOLLOW_LINE | arm
    assert f[3] == 4                                      # mission_state EXPLORE
    assert f[4] == 42                                     # seq
    assert struct.unpack_from("<b", f, 5)[0] == 5         # node_x
    assert struct.unpack_from("<b", f, 6)[0] == -3        # node_y
    assert f[7] == 2                                      # move_direction Y_POS
    assert struct.unpack_from("<H", f, 8)[0] == 250       # target_altitude cm
    assert struct.unpack_from("<h", f, 10)[0] == 8192     # line_dx  0.5  Q14
    assert struct.unpack_from("<h", f, 12)[0] == -12288   # line_dy -0.75 Q14
    assert struct.unpack_from("<b", f, 26)[0] == 7        # marker_id
    assert f[27] == 204                                   # line_confidence
    assert f[28] == 153                                   # marker_confidence
    assert f[29] == 0xAD                                  # flags
    assert f[30] == 0x03                                  # flags2
    assert f[31] == 40                                    # speed_scale
    # CRC16-CCITT little-endian over bytes 0..31.
    assert struct.unpack_from("<H", f, 32)[0] == 0xFF1E


def test_crc_known_vector():
    """CRC16-CCITT(0xFFFF, 0x1021) over "123456789" is 0x29B1 — the same
    check the C side runs (Protocol.CrcKnownVector)."""
    assert fc_proto_crc16_ccitt(b"123456789") == 0x29B1


def test_crc_covers_payload_and_rejects_flip():
    frame = bytearray(pack_mcu_command(_golden_command()))
    crc = struct.unpack_from("<H", frame, 32)[0]
    frame[31] ^= 0x01  # corrupt speed_scale
    recomputed = fc_proto_crc16_ccitt(bytes(frame[:32]))
    assert recomputed != crc


def test_arm_bit_toggles_mode_byte_bit7():
    cmd = McuCommand(mode=int(ControlMode.HOLD))
    assert pack_mcu_command(cmd, arm=True)[2] & 0x80
    assert not (pack_mcu_command(cmd, arm=False)[2] & 0x80)
    # bits 0..6 always carry the ControlMode enum.
    cmd = McuCommand(mode=int(ControlMode.EMERGENCY_LAND))  # 7
    assert pack_mcu_command(cmd, arm=False)[2] == 7


def test_seq_override_and_wrap():
    cmd = McuCommand(seq=1)
    assert pack_mcu_command(cmd, seq=None)[4] == 1     # falls back to cmd.seq
    assert pack_mcu_command(cmd, seq=300)[4] == 300 & 0xFF  # wraps to 44


def test_q14_clamps_to_plus_minus_two_rail():
    cmd = McuCommand(line_dx=2.0, line_dy=-2.0, vx_est=5.0, vy_est=-5.0)
    f = pack_mcu_command(cmd)
    assert struct.unpack_from("<h", f, 10)[0] == 32767    # +2.0 saturates
    assert struct.unpack_from("<h", f, 12)[0] == -32768   # -2.0 exact rail
    assert struct.unpack_from("<h", f, 22)[0] == 32767    # +5.0 -> rail
    assert struct.unpack_from("<h", f, 24)[0] == -32768   # -5.0 -> rail


def test_altitude_clamps_zero_to_ten_metres():
    assert struct.unpack_from("<H", pack_mcu_command(McuCommand(target_altitude=20.0)), 8)[0] == 1000
    assert struct.unpack_from("<H", pack_mcu_command(McuCommand(target_altitude=-1.0)), 8)[0] == 0


def test_confidence_float_to_u8():
    # 0..1 float maps to 0..255, out-of-range clamps.
    assert pack_mcu_command(McuCommand(line_confidence=1.0))[27] == 255
    assert pack_mcu_command(McuCommand(line_confidence=0.0))[27] == 0
    assert pack_mcu_command(McuCommand(line_confidence=2.0))[27] == 255
    assert pack_mcu_command(McuCommand(marker_confidence=-1.0))[28] == 0


def test_marker_id_none_is_minus_one():
    assert struct.unpack_from("<b", pack_mcu_command(McuCommand(marker_id=-1)), 26)[0] == -1


def test_node_index_i8_signedness():
    f = pack_mcu_command(McuCommand(node_x=-3, node_y=127))
    assert f[5] == 0xFD                                    # -3 as i8
    assert struct.unpack_from("<b", f, 5)[0] == -3
    assert struct.unpack_from("<b", f, 6)[0] == 127


def test_flags_bit_layout():
    cmd = McuCommand(
        vertical_line=True,
        horizontal_line=True,
        intersection_detected=True,
        intersection_forward=True,
        intersection_left=True,
        intersection_right=True,
        intersection_backward=True,
        marker_detected=True,
        vel_est_valid=True,
        emergency=True,
    )
    f = pack_mcu_command(cmd)
    assert f[29] == 0xFF   # all eight flags bits set
    assert f[30] == 0x03   # vel_est_valid | emergency


def test_reexport_from_mission_module_produces_same_bytes():
    """mission.py re-exports the packer; the wire bytes are identical."""
    from line_tracer.mission import McuCommand as MC, pack_mcu_command as pack
    assert pack(_golden_command(), seq=42, arm=True) == GOLDEN_FRAME
    assert MC is McuCommand


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
