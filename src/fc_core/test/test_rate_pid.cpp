/*
 * test_rate_pid.cpp
 *
 * One-tick PID step: verify RATEControl produces kp * err + ki * dt * err
 * given the documented gains and a step from zero. ControllerInit must
 * be called first so pid_rate is in a known state.
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "fc_core/controller.h"
#include "fc_core/linalg.h"
}

TEST(RatePID, SingleTick_RollAxisStepResponse) {
    ControllerInit();

    /* kp_p = 0.8 * 0.5 = 0.4, ki_p = kp_p * 0.1 = 0.04 */
    const float kp_p = 0.4f;
    const float ki_p = 0.04f;
    const float dt = 1.0f / 500.0f;
    const float err = 0.5f;     /* rad/s, well above the 0.04 rad/s deadband */

    vec3d pqr_des = vec(err, 0.0f, 0.0f);
    vec3d pqr     = vec(0.0f, 0.0f, 0.0f);

    vec3d LMN = RATEControl(pqr_des, pqr, dt);

    /* integral = dt * err clamped well within +/- 0.75
     * LMN.x = kp * err + ki * (dt * err) */
    float expected = kp_p * err + ki_p * (dt * err);
    EXPECT_NEAR(LMN.x, expected, 1e-5f);
    EXPECT_NEAR(LMN.y, 0.0f, 1e-6f);
    EXPECT_NEAR(LMN.z, 0.0f, 1e-6f);
}

TEST(RatePID, Deadband_BlocksTinyErrors) {
    ControllerInit();
    const float dt = 1.0f / 500.0f;

    /* maxratecmd * 0.04 = 1.0 * 0.04 = 0.04 rad/s deadband.
     * Push an error of 0.02 rad/s, well below deadband. */
    vec3d pqr_des = vec(0.02f, 0.0f, 0.0f);
    vec3d pqr     = vec(0.0f, 0.0f, 0.0f);

    /* deadband zeroes pqr_des before subtraction; integral picks up the
     * negative-pqr term instead of the desired-rate. With pqr=0 the
     * error is exactly zero. */
    vec3d LMN = RATEControl(pqr_des, pqr, dt);
    EXPECT_NEAR(LMN.x, 0.0f, 1e-6f);
}
