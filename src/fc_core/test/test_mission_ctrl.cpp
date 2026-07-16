/*
 * test_mission_ctrl.cpp
 *
 * Unit tests for the MCU outer loop (fc_mission_tick): every ControlMode's
 * mapping, the takeoff burst, land cutoff/disarm, the velocity-loop vs
 * open-loop attitude branch, attitude/thrust clamps, and the load-bearing
 * FLU->NED yawrate sign. Expected values are hand-computed from the ported
 * gains (fc_mission_gains_default).
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "fc_core/mission_ctrl.h"
#include "fc_core/controller.h"
}

namespace {

constexpr float G = 9.80665f;

fc_proto_mission_t make_cmd(fc_control_mode_t mode, bool arm) {
    fc_proto_mission_t c{};
    c.mode = (uint8_t)mode | (arm ? FC_PROTO_MODE_ARM_BIT : 0u);
    c.target_altitude = 2.0f;
    c.move_direction = FC_DIR_X_POS;
    return c;
}

fc_mission_meas_t make_meas(float alt, float vz, bool valid) {
    fc_mission_meas_t m{};
    m.altitude = alt;
    m.vz = vz;
    m.vx_body = 0.0f;
    m.vy_body = 0.0f;
    m.vel_valid = valid;
    return m;
}

class MissionCtrl : public ::testing::Test {
protected:
    void SetUp() override {
        fc_mission_gains_default(&fc_mission_gains);
        fc_now_ms = 1000u;
        COMP = compsp_t{};
    }
};

}  // namespace

TEST_F(MissionCtrl, HoldHoldsAltitudeWithZeroVelocity) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_HOLD, true);
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.roll_sp,    0.0f, 1e-6f);
    EXPECT_NEAR(COMP.pitch_sp,   0.0f, 1e-6f);
    EXPECT_NEAR(COMP.yawrate_sp, 0.0f, 1e-6f);
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);   /* hover, alt_err=0 */
    EXPECT_EQ(COMP.arm, 1u);
    EXPECT_EQ(COMP.mode, (uint8_t)attithrmode);
    EXPECT_EQ(COMP.last_ms, fc_now_ms);
}

TEST_F(MissionCtrl, FollowLineXTravelUsesLineDx) {
    /* Open-loop branch (no velocity estimate). X_POS travel: cruise on
       +x_body, perp (from line_dx) on +y_body, wz from line_angle_error.
       line_dy is the wrong-axis offset and must be ignored. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.move_direction = FC_DIR_X_POS;
    c.line_dx = 2.0f;               /* +y_body demand (wire clamp max) */
    c.line_dy = 9.0f;               /* wrong axis -> ignored */
    c.line_angle_error = 0.1f;      /* +CCW */
    c.flags = FC_PROTO_MFLAG_VERTICAL_LINE;
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, false);

    fc_mission_tick(&c, &m, 0.02f);

    /* vx=cruise=0.2, vy=kp_xy*2.0=0.4, |v|=0.447 -> clamp to 0.4:
       vx=0.17889, vy=0.35777. Open-loop: pitch=vx/g, roll=-vy/g. */
    EXPECT_NEAR(COMP.pitch_sp,  0.17889f / G, 1e-4f);
    EXPECT_NEAR(COMP.roll_sp,  -0.35777f / G, 1e-4f);
    EXPECT_GT(COMP.pitch_sp, 0.0f);            /* forward cruise -> nose down/+x */
    EXPECT_LT(COMP.roll_sp,  0.0f);            /* +y demand from line_dx -> -roll */
    EXPECT_NEAR(COMP.yawrate_sp, -0.3f, 1e-4f);  /* -kp_yaw*angle, FLU->NED */
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);
    EXPECT_EQ(COMP.arm, 1u);
}

TEST_F(MissionCtrl, FollowLineYTravelUsesLineDy) {
    /* Y_POS travel: cruise on +y_body, perp (from line_dy) on +x_body.
       line_dx is the wrong-axis offset and must be ignored. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.move_direction = FC_DIR_Y_POS;
    c.line_dy = 2.0f;               /* +x_body demand (wire clamp max) */
    c.line_dx = 9.0f;               /* wrong axis -> ignored */
    c.flags = FC_PROTO_MFLAG_HORIZONTAL_LINE;
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, false);

    fc_mission_tick(&c, &m, 0.02f);

    /* vy=cruise=0.2, vx=kp_xy*2.0=0.4, |v|=0.447 -> clamp to 0.4:
       vx=0.35777, vy=0.17889. Open-loop: pitch=vx/g, roll=-vy/g. */
    EXPECT_NEAR(COMP.pitch_sp,  0.35777f / G, 1e-4f);
    EXPECT_NEAR(COMP.roll_sp,  -0.17889f / G, 1e-4f);
    EXPECT_GT(COMP.pitch_sp, 0.0f);            /* +x_body demand from line_dy -> +pitch */
    EXPECT_LT(COMP.roll_sp,  0.0f);            /* +y cruise -> -roll */
}

TEST_F(MissionCtrl, FollowLineLateralGainKeepsCruiseThroughClamp) {
    /* Regression for the r77 lateral divergence. At a 1 m line offset
       (typical worst in-view error at 2 m altitude) the lateral demand
       kp_xy*1.0 must not saturate the max_vxy vector clamp: the clamp
       preserves direction, so a saturating lateral demand also cuts the
       forward cruise. With the old kp_xy=0.8 the clamp halved vx and the
       lateral stiffness sat far above what the attitude loop can track. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.move_direction = FC_DIR_X_POS;
    c.line_dx = 1.0f;
    c.flags = FC_PROTO_MFLAG_VERTICAL_LINE;
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);   /* at rest, vel loop */

    fc_mission_tick(&c, &m, 0.02f);

    /* Unsqueezed cruise: pitch = kp_vel*cruise = 0.02 exactly. */
    EXPECT_NEAR(COMP.pitch_sp, 0.02f, 1e-4f);
    /* Lateral demand stays proportional (no clamp): roll = -kp_vel*kp_xy. */
    EXPECT_NEAR(COMP.roll_sp, -0.1f * 0.2f * 1.0f, 1e-4f);
}

TEST_F(MissionCtrl, FollowLineXTravelHoldsWithoutVerticalLine) {
    /* X travel with the vertical presence bit clear -> HOLD behavior. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.move_direction = FC_DIR_X_POS;
    c.line_dx = 0.5f;
    c.line_angle_error = 0.1f;
    c.flags = FC_PROTO_MFLAG_HORIZONTAL_LINE;   /* wrong axis bit -> still HOLD */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, false);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.pitch_sp,   0.0f, 1e-6f);
    EXPECT_NEAR(COMP.roll_sp,    0.0f, 1e-6f);
    EXPECT_NEAR(COMP.yawrate_sp, 0.0f, 1e-6f);
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);
}

TEST_F(MissionCtrl, FollowLineYTravelHoldsWithoutHorizontalLine) {
    /* Y travel with the horizontal presence bit clear -> HOLD behavior. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.move_direction = FC_DIR_Y_POS;
    c.line_dy = 0.5f;
    c.flags = FC_PROTO_MFLAG_VERTICAL_LINE;     /* wrong axis bit -> still HOLD */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, false);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.pitch_sp,   0.0f, 1e-6f);
    EXPECT_NEAR(COMP.roll_sp,    0.0f, 1e-6f);
    EXPECT_NEAR(COMP.yawrate_sp, 0.0f, 1e-6f);
}

TEST_F(MissionCtrl, AlignMarkerMarkerErrorLaw) {
    /* Velocity-loop branch, drone at rest: pitch=kp_vel*vx_cmd. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_ALIGN_MARKER, true);
    c.marker_error_x = 0.8f;
    c.marker_error_y = -0.4f;
    c.flags = FC_PROTO_MFLAG_MARKER_DETECTED;
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    fc_mission_tick(&c, &m, 0.02f);

    /* vx=kp_xy*0.8=0.16, vy=kp_xy*-0.4=-0.08 (|v|<max, no clamp).
       pitch=+kp_vel*0.16=0.016, roll=-kp_vel*-0.08=+0.008. */
    EXPECT_NEAR(COMP.pitch_sp, 0.016f, 1e-5f);
    EXPECT_NEAR(COMP.roll_sp,  0.008f, 1e-5f);
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);
    EXPECT_EQ(COMP.arm, 1u);
}

TEST_F(MissionCtrl, AlignMarkerFallsBackToHoldWhenNoMarker) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_ALIGN_MARKER, true);
    c.marker_error_x = 0.5f;         /* must be ignored */
    c.marker_error_y = 0.5f;
    c.flags = 0u;                    /* marker_detected clear */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.pitch_sp, 0.0f, 1e-6f);
    EXPECT_NEAR(COMP.roll_sp,  0.0f, 1e-6f);
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);
}

TEST_F(MissionCtrl, SearchLineSlowCruiseInMoveDirection) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_SEARCH_LINE, true);
    c.move_direction = FC_DIR_Y_POS;   /* +y_body cruise */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    fc_mission_tick(&c, &m, 0.02f);

    /* vy=cruise=0.2, vx=0. vel-loop at rest: roll=-kp_vel*vy=-0.02. */
    EXPECT_NEAR(COMP.pitch_sp, 0.0f,   1e-6f);
    EXPECT_NEAR(COMP.roll_sp, -0.02f,  1e-5f);
    EXPECT_EQ(COMP.arm, 1u);
}

TEST_F(MissionCtrl, LandOnMarkerTracksDescentRate) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_LAND_ON_MARKER, true);
    fc_mission_meas_t m = make_meas(1.0f, -0.30f, true);   /* above cutoff, at target vz */

    fc_mission_tick(&c, &m, 0.02f);

    /* target forced to 0 -> land law: thrust=hover+kd*(land_vz-vz)
       = 0.33 + 0.20*(-0.30 - -0.30) = 0.33. Still armed. */
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);
    EXPECT_EQ(COMP.arm, 1u);

    /* At vz=0 the descent law demands less: 0.33-0.06=0.27 -> clamp min 0.28. */
    fc_mission_meas_t m2 = make_meas(1.0f, 0.0f, true);
    fc_mission_tick(&c, &m2, 0.02f);
    EXPECT_NEAR(COMP.thrust_norm, 0.28f, 1e-4f);
}

TEST_F(MissionCtrl, LandOnMarkerCutoffDisarms) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_LAND_ON_MARKER, true);
    fc_mission_meas_t m = make_meas(0.10f, 0.0f, true);   /* below land_cutoff_alt */

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.thrust_norm, 0.0f, 1e-6f);
    EXPECT_EQ(COMP.arm, 0u);
    EXPECT_NEAR(COMP.roll_sp,    0.0f, 1e-6f);
    EXPECT_NEAR(COMP.pitch_sp,   0.0f, 1e-6f);
    EXPECT_NEAR(COMP.yawrate_sp, 0.0f, 1e-6f);
}

TEST_F(MissionCtrl, StopCutsThrustAndDisarms) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_STOP, true);   /* arm bit set but STOP wins */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.thrust_norm, 0.0f, 1e-6f);
    EXPECT_EQ(COMP.arm, 0u);
    EXPECT_NEAR(COMP.roll_sp,    0.0f, 1e-6f);
    EXPECT_NEAR(COMP.pitch_sp,   0.0f, 1e-6f);
    EXPECT_NEAR(COMP.yawrate_sp, 0.0f, 1e-6f);
}

TEST_F(MissionCtrl, EmergencyLandIgnoresErrorsAndDescends) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_EMERGENCY_LAND, true);
    c.marker_error_x = 5.0f;         /* huge errors, all must be ignored */
    c.marker_error_y = 5.0f;
    c.line_dx = 5.0f;
    c.line_dy = 5.0f;
    c.flags = 0xFFu;
    fc_mission_meas_t m = make_meas(1.0f, -0.30f, true);

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.pitch_sp, 0.0f, 1e-6f);   /* zero velocity, errors ignored */
    EXPECT_NEAR(COMP.roll_sp,  0.0f, 1e-6f);
    EXPECT_NEAR(COMP.thrust_norm, 0.33f, 1e-4f);  /* land law at target vz */
    EXPECT_EQ(COMP.arm, 1u);
}

TEST_F(MissionCtrl, TakeoffBurstBelowThreshold) {
    /* ALIGN with no marker -> HOLD (zero velocity), low + not rising. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_ALIGN_MARKER, true);
    c.flags = 0u;
    c.target_altitude = 2.0f;
    fc_mission_meas_t m = make_meas(0.05f, 0.0f, true);   /* alt<0.15, vz<0.2, alt_err>0.5 */

    fc_mission_tick(&c, &m, 0.02f);

    EXPECT_NEAR(COMP.thrust_norm, 0.43f, 1e-4f);   /* takeoff_thrust_norm */
    EXPECT_EQ(COMP.arm, 1u);
}

TEST_F(MissionCtrl, TakeoffBurstSuppressedOnceRising) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_ALIGN_MARKER, true);
    c.flags = 0u;
    c.target_altitude = 2.0f;
    fc_mission_meas_t m = make_meas(0.05f, 0.5f, true);   /* rising: vz>0.2 */

    fc_mission_tick(&c, &m, 0.02f);

    /* PD path: 0.33 + 0.17*(2.0-0.05) - 0.20*0.5 = 0.5615. */
    EXPECT_NEAR(COMP.thrust_norm, 0.5615f, 1e-3f);
}

TEST_F(MissionCtrl, VelocityLoopVersusOpenLoopAttitude) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_MOVE_TO_LANDMARK, true);
    c.move_direction = FC_DIR_X_POS;    /* vx=cruise=0.2, vy=0 */

    /* Open-loop: pitch = vx/g. */
    fc_mission_meas_t open_m = make_meas(2.0f, 0.0f, false);
    fc_mission_tick(&c, &open_m, 0.02f);
    EXPECT_NEAR(COMP.pitch_sp, 0.2f / G, 1e-5f);

    /* Velocity loop at rest: pitch = kp_vel*(0.2-0) = 0.02. */
    fc_mission_meas_t rest_m = make_meas(2.0f, 0.0f, true);
    fc_mission_tick(&c, &rest_m, 0.02f);
    EXPECT_NEAR(COMP.pitch_sp, 0.02f, 1e-5f);

    /* Velocity loop at commanded speed: error 0 -> level. */
    fc_mission_meas_t moving_m = make_meas(2.0f, 0.0f, true);
    moving_m.vx_body = 0.2f;
    fc_mission_tick(&c, &moving_m, 0.02f);
    EXPECT_NEAR(COMP.pitch_sp, 0.0f, 1e-6f);
}

TEST_F(MissionCtrl, AttitudeClampsAtMaxSetpoint) {
    fc_proto_mission_t c = make_cmd(FC_CTRL_ALIGN_MARKER, true);
    c.marker_error_x = 0.2f;
    c.marker_error_y = 0.0f;
    c.flags = FC_PROTO_MFLAG_MARKER_DETECTED;

    /* Large velocity errors saturate both axes to +max_atti_setpoint. */
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);
    m.vx_body = -5.0f;   /* pitch err positive */
    m.vy_body =  5.0f;   /* roll err positive */
    fc_mission_tick(&c, &m, 0.02f);
    EXPECT_NEAR(COMP.pitch_sp, 0.15f, 1e-6f);
    EXPECT_NEAR(COMP.roll_sp,  0.15f, 1e-6f);

    /* Opposite sign saturates to -max_atti_setpoint. */
    fc_mission_meas_t m2 = make_meas(2.0f, 0.0f, true);
    m2.vx_body =  5.0f;
    m2.vy_body = -5.0f;
    fc_mission_tick(&c, &m2, 0.02f);
    EXPECT_NEAR(COMP.pitch_sp, -0.15f, 1e-6f);
    EXPECT_NEAR(COMP.roll_sp,  -0.15f, 1e-6f);
}

TEST_F(MissionCtrl, ThrustClampsAtMaxAndMin) {
    /* Max: big altitude error, above takeoff threshold so PD (not burst). */
    fc_proto_mission_t c = make_cmd(FC_CTRL_HOLD, true);
    c.target_altitude = 3.0f;
    fc_mission_meas_t hi = make_meas(0.5f, 0.0f, true);
    fc_mission_tick(&c, &hi, 0.02f);
    EXPECT_NEAR(COMP.thrust_norm, 0.60f, 1e-4f);   /* 0.33+0.17*2.5 clamps to max */

    /* Min: rising fast at target -> PD demands below the floor. */
    c.target_altitude = 2.0f;
    fc_mission_meas_t lo = make_meas(2.0f, 2.0f, true);
    fc_mission_tick(&c, &lo, 0.02f);
    EXPECT_NEAR(COMP.thrust_norm, 0.28f, 1e-4f);   /* 0.33-0.20*2.0 clamps to min */
}

TEST_F(MissionCtrl, YawrateSignIsNegatedFromWz) {
    /* The fleet-critical FLU(+CCW) -> NED(+CW) flip: +wz -> -yawrate_sp. */
    fc_proto_mission_t c = make_cmd(FC_CTRL_FOLLOW_LINE, true);
    c.flags = FC_PROTO_MFLAG_VERTICAL_LINE;
    c.line_dx = 0.0f;
    fc_mission_meas_t m = make_meas(2.0f, 0.0f, true);

    c.line_angle_error = 0.5f;         /* +CCW -> wz=+1.5 */
    fc_mission_tick(&c, &m, 0.02f);
    EXPECT_NEAR(COMP.yawrate_sp, -1.5f, 1e-4f);

    c.line_angle_error = -0.5f;        /* -CCW -> wz=-1.5 */
    fc_mission_tick(&c, &m, 0.02f);
    EXPECT_NEAR(COMP.yawrate_sp, 1.5f, 1e-4f);
}
