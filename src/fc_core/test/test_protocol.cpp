/*
 * test_protocol.cpp
 *
 * Encode/decode roundtrip across the 24-byte downlink and 40-byte
 * uplink frames. Quantization tolerance is set to the Q14/Q15 LSB.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "fc_core/protocol.h"
#include "fc_core/controller.h"
#include "fc_core/mission_ctrl.h"
}

namespace {

float frand(float lo, float hi) {
    float u = (float)std::rand() / (float)RAND_MAX;
    return lo + u * (hi - lo);
}

}  // namespace

TEST(Protocol, DownlinkRoundtrip_QuantizationOnly) {
    std::srand(1234);

    for (int i = 0; i < 200; i++) {
        fc_proto_down_t in{};
        in.mode         = (uint8_t)(((i & 0x03) | (i & 1 ? FC_PROTO_MODE_ARM_BIT : 0)));
        in.seq          = (uint8_t)i;
        in.roll_sp      = frand(-0.5f, 0.5f);     /* rad */
        in.pitch_sp     = frand(-0.5f, 0.5f);
        in.yawrate_sp   = frand(-1.5f, 1.5f);     /* rad/s, within Q14 range */
        in.vz_sp        = frand(-1.0f, 1.0f);
        in.thrust_norm  = frand( 0.0f, 1.0f);
        in.timestamp_ms = (uint32_t)i * 5u;
        in.flags        = 0u;

        uint8_t buf[FC_PROTO_DOWN_LEN] = {0};
        ASSERT_TRUE(fc_proto_encode_down(&in, buf));

        fc_proto_down_t out{};
        ASSERT_TRUE(fc_proto_decode_down(buf, &out));

        EXPECT_EQ(out.mode,         in.mode);
        EXPECT_EQ(out.seq,          in.seq);
        EXPECT_EQ(out.timestamp_ms, in.timestamp_ms);
        EXPECT_EQ(out.flags,        in.flags);

        /* Q14 LSB = 1/16384 ≈ 6.1e-5; allow 2 LSB tolerance. */
        EXPECT_NEAR(out.roll_sp,    in.roll_sp,    2.0f / 16384.0f);
        EXPECT_NEAR(out.pitch_sp,   in.pitch_sp,   2.0f / 16384.0f);
        EXPECT_NEAR(out.yawrate_sp, in.yawrate_sp, 2.0f / 16384.0f);
        EXPECT_NEAR(out.vz_sp,      in.vz_sp,      2.0f / 16384.0f);

        /* Q15 LSB = 1/32767. */
        EXPECT_NEAR(out.thrust_norm, in.thrust_norm, 2.0f / 32767.0f);
    }
}

TEST(Protocol, DownlinkDecode_RejectsCorruptedCrc) {
    fc_proto_down_t in{};
    in.mode = (uint8_t)1u | FC_PROTO_MODE_ARM_BIT;
    in.thrust_norm = 0.5f;

    uint8_t buf[FC_PROTO_DOWN_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_down(&in, buf));

    buf[5] ^= 0x01;  /* flip one bit in the pitch_sp field */

    fc_proto_down_t out{};
    EXPECT_FALSE(fc_proto_decode_down(buf, &out));
}

TEST(Protocol, DownlinkDecode_RejectsWrongMagic) {
    fc_proto_down_t in{};
    in.thrust_norm = 0.5f;
    uint8_t buf[FC_PROTO_DOWN_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_down(&in, buf));
    buf[0] = 0x00;
    fc_proto_down_t out{};
    EXPECT_FALSE(fc_proto_decode_down(buf, &out));
}

TEST(Protocol, UplinkRoundtrip_BitExactFloats) {
    fc_proto_up_t in{};
    in.state = 3u;
    in.seq   = 42u;
    in.roll = 0.123f;  in.pitch = -0.456f; in.yaw = 1.234f;
    in.p = 0.01f; in.q = -0.02f; in.r = 0.03f;
    in.alt_lidar = 1.234f;
    in.vbatt_volts = 12.6f;
    in.flag_word = 0xABCDu;

    uint8_t buf[FC_PROTO_UP_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_up(&in, buf));

    fc_proto_up_t out{};
    ASSERT_TRUE(fc_proto_decode_up(buf, &out));

    EXPECT_EQ(out.state, in.state);
    EXPECT_EQ(out.seq,   in.seq);
    EXPECT_FLOAT_EQ(out.roll,        in.roll);
    EXPECT_FLOAT_EQ(out.pitch,       in.pitch);
    EXPECT_FLOAT_EQ(out.yaw,         in.yaw);
    EXPECT_FLOAT_EQ(out.p,           in.p);
    EXPECT_FLOAT_EQ(out.q,           in.q);
    EXPECT_FLOAT_EQ(out.r,           in.r);
    EXPECT_FLOAT_EQ(out.alt_lidar,   in.alt_lidar);
    EXPECT_FLOAT_EQ(out.vbatt_volts, in.vbatt_volts);
    EXPECT_EQ(out.flag_word,         in.flag_word);
}

TEST(Protocol, CrcKnownVector) {
    /* CRC16-CCITT(0xFFFF init, poly 0x1021) over "123456789" = 0x29B1. */
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    uint16_t crc = fc_proto_crc16_ccitt(data, sizeof(data));
    EXPECT_EQ(crc, 0x29B1u);
}

TEST(Protocol, ApplyDown_PopulatesGlobalComp) {
    fc_proto_down_t msg{};
    msg.mode = (uint8_t)1u | FC_PROTO_MODE_ARM_BIT;
    msg.roll_sp = 0.1f;
    msg.pitch_sp = -0.05f;
    msg.yawrate_sp = 0.0f;
    msg.thrust_norm = 0.5f;

    fc_proto_apply_down(&msg, 12345u);

    EXPECT_EQ(COMP.mode, 1u);
    EXPECT_EQ(COMP.arm, 1u);
    EXPECT_NEAR(COMP.roll_sp,    0.1f,  1e-6f);
    EXPECT_NEAR(COMP.pitch_sp,  -0.05f, 1e-6f);
    EXPECT_NEAR(COMP.thrust_norm, 0.5f, 1e-6f);
    EXPECT_EQ(COMP.last_ms, 12345u);
}

/* ---------------- Mission downlink (0xA6) ---------------- */

TEST(ProtocolMission, Roundtrip_QuantizationOnly) {
    std::srand(9876);

    for (int i = 0; i < 200; i++) {
        fc_proto_mission_t in{};
        in.mode           = (uint8_t)((i % 8) | (i & 1 ? FC_PROTO_MODE_ARM_BIT : 0));
        in.mission_state  = (uint8_t)(i % 12);
        in.seq            = (uint8_t)i;
        in.node_x         = (int8_t)(i - 100);
        in.node_y         = (int8_t)(50 - i);
        in.move_direction = (uint8_t)(i % 4);
        in.target_altitude    = frand(0.0f, 10.0f);
        in.line_dx            = frand(-1.9f, 1.9f);
        in.line_dy            = frand(-1.9f, 1.9f);
        in.line_angle_error   = frand(-1.5f, 1.5f);
        in.marker_error_x     = frand(-1.0f, 1.0f);
        in.marker_error_y     = frand(-1.0f, 1.0f);
        in.marker_yaw_error   = frand(-0.5f, 0.5f);
        in.vx_est             = frand(-1.9f, 1.9f);
        in.vy_est             = frand(-1.9f, 1.9f);
        in.marker_id          = (int8_t)((i % 5) - 1);
        in.line_confidence    = (uint8_t)(i & 0xFF);
        in.marker_confidence  = (uint8_t)((255 - i) & 0xFF);
        in.flags              = (uint8_t)(i & 0xFF);
        in.flags2             = (uint8_t)(i & 0x01);

        uint8_t buf[FC_PROTO_MISSION_LEN] = {0};
        ASSERT_TRUE(fc_proto_encode_mission(&in, buf));
        EXPECT_EQ(buf[0], FC_PROTO_MISSION_MAGIC);

        fc_proto_mission_t out{};
        ASSERT_TRUE(fc_proto_decode_mission(buf, &out));

        EXPECT_EQ(out.mode,           in.mode);
        EXPECT_EQ(out.mission_state,  in.mission_state);
        EXPECT_EQ(out.seq,            in.seq);
        EXPECT_EQ(out.node_x,         in.node_x);
        EXPECT_EQ(out.node_y,         in.node_y);
        EXPECT_EQ(out.move_direction, in.move_direction);
        EXPECT_EQ(out.marker_id,      in.marker_id);
        EXPECT_EQ(out.line_confidence,   in.line_confidence);
        EXPECT_EQ(out.marker_confidence, in.marker_confidence);
        EXPECT_EQ(out.flags,          in.flags);
        EXPECT_EQ(out.flags2,         in.flags2);

        /* target_altitude quantizes to 1 cm. */
        EXPECT_NEAR(out.target_altitude, in.target_altitude, 0.01f);
        /* Q14 LSB = 1/16384; allow 2 LSB. */
        EXPECT_NEAR(out.line_dx,            in.line_dx,            2.0f / 16384.0f);
        EXPECT_NEAR(out.line_dy,            in.line_dy,            2.0f / 16384.0f);
        EXPECT_NEAR(out.line_angle_error,   in.line_angle_error,   2.0f / 16384.0f);
        EXPECT_NEAR(out.marker_error_x,     in.marker_error_x,     2.0f / 16384.0f);
        EXPECT_NEAR(out.marker_error_y,     in.marker_error_y,     2.0f / 16384.0f);
        EXPECT_NEAR(out.marker_yaw_error,   in.marker_yaw_error,   2.0f / 16384.0f);
        EXPECT_NEAR(out.vx_est,             in.vx_est,             2.0f / 16384.0f);
        EXPECT_NEAR(out.vy_est,             in.vy_est,             2.0f / 16384.0f);
    }
}

TEST(ProtocolMission, Q14SaturatesAtPlusMinusTwo) {
    fc_proto_mission_t in{};
    in.line_dx            = 2.0f;    /* at the +/-2.0 Q14 rail */
    in.line_dy            = -2.0f;
    in.marker_error_x     = -2.0f;
    in.vx_est             = 5.0f;    /* well past the rail */
    in.vy_est             = -5.0f;

    uint8_t buf[FC_PROTO_MISSION_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_mission(&in, buf));
    fc_proto_mission_t out{};
    ASSERT_TRUE(fc_proto_decode_mission(buf, &out));

    /* +2.0 saturates to 32767/16384 ~= 1.99994; -2.0 is exact. */
    EXPECT_NEAR(out.line_dx,             1.99994f, 1.0f / 16384.0f);
    EXPECT_NEAR(out.line_dy,            -2.0f,     1.0f / 16384.0f);
    EXPECT_NEAR(out.marker_error_x,     -2.0f,     1.0f / 16384.0f);
    /* Beyond the rail clamps to the same rail values. */
    EXPECT_NEAR(out.vx_est,  1.99994f, 1.0f / 16384.0f);
    EXPECT_NEAR(out.vy_est, -2.0f,     1.0f / 16384.0f);
}

TEST(ProtocolMission, FlagsBitsPreserved) {
    fc_proto_mission_t in{};
    in.flags = FC_PROTO_MFLAG_VERTICAL_LINE | FC_PROTO_MFLAG_RIGHT
             | FC_PROTO_MFLAG_MARKER_DETECTED;
    in.flags2 = FC_PROTO_MFLAG2_VEL_EST_VALID | FC_PROTO_MFLAG2_EMERGENCY;

    uint8_t buf[FC_PROTO_MISSION_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_mission(&in, buf));
    fc_proto_mission_t out{};
    ASSERT_TRUE(fc_proto_decode_mission(buf, &out));

    EXPECT_TRUE(out.flags & FC_PROTO_MFLAG_VERTICAL_LINE);
    EXPECT_TRUE(out.flags & FC_PROTO_MFLAG_RIGHT);
    EXPECT_TRUE(out.flags & FC_PROTO_MFLAG_MARKER_DETECTED);
    EXPECT_FALSE(out.flags & FC_PROTO_MFLAG_HORIZONTAL_LINE);
    EXPECT_FALSE(out.flags & FC_PROTO_MFLAG_INTERSECTION);
    EXPECT_FALSE(out.flags & FC_PROTO_MFLAG_LEFT);
    EXPECT_TRUE(out.flags2 & FC_PROTO_MFLAG2_VEL_EST_VALID);
    EXPECT_TRUE(out.flags2 & FC_PROTO_MFLAG2_EMERGENCY);
}

TEST(ProtocolMission, Decode_RejectsCorruptedCrc) {
    fc_proto_mission_t in{};
    in.mode = (uint8_t)FC_PROTO_MODE_ARM_BIT | 1u;
    in.line_dx = 0.3f;

    uint8_t buf[FC_PROTO_MISSION_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_mission(&in, buf));

    buf[11] ^= 0x01;  /* flip a bit in line_dx */

    fc_proto_mission_t out{};
    EXPECT_FALSE(fc_proto_decode_mission(buf, &out));
}

TEST(ProtocolMission, Decode_RejectsWrongMagic) {
    fc_proto_mission_t in{};
    uint8_t buf[FC_PROTO_MISSION_LEN] = {0};
    ASSERT_TRUE(fc_proto_encode_mission(&in, buf));
    buf[0] = 0xA5;  /* the setpoint-downlink magic, not the mission magic */
    fc_proto_mission_t out{};
    EXPECT_FALSE(fc_proto_decode_mission(buf, &out));
}

TEST(ProtocolMission, ApplyMission_PopulatesGlobalState) {
    fc_proto_mission_t msg{};
    msg.mode = (uint8_t)FC_CTRL_FOLLOW_LINE | FC_PROTO_MODE_ARM_BIT;
    msg.seq = 7u;
    msg.target_altitude = 2.0f;

    MISSION.valid = false;
    fc_proto_apply_mission(&msg, 55555u);

    EXPECT_TRUE(MISSION.valid);
    EXPECT_EQ(MISSION.last_ms, 55555u);
    EXPECT_EQ(MISSION.cmd.seq, 7u);
    EXPECT_NEAR(MISSION.cmd.target_altitude, 2.0f, 1e-6f);
}
