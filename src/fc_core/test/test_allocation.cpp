/*
 * test_allocation.cpp
 *
 * Verifies the mixer produces equal per-motor thrust for a pure hover
 * (no torque commanded) with the firmware's mass and arm geometry.
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "fc_core/controller.h"
#include "fc_core/linalg.h"
}

TEST(Allocation, Hover_AllMotorsEqual) {
    const float mass = 1.182f;
    const float g = 9.80665f;
    const float Fz = -mass * g;             /* NED: negative Z = up */
    const float dx = 365.490f / 1000.0f / 2.0f;
    const float dy = 335.235f / 1000.0f / 2.0f;

    thrvec T = Allocation(vec(0.0f, 0.0f, 0.0f), vec(dx, dy, 1.0f), Fz);

    const float per_motor = -Fz / 4.0f;     /* expected hover thrust per rotor */
    EXPECT_NEAR(T.T1, per_motor, 1e-4f);
    EXPECT_NEAR(T.T2, per_motor, 1e-4f);
    EXPECT_NEAR(T.T3, per_motor, 1e-4f);
    EXPECT_NEAR(T.T4, per_motor, 1e-4f);

    /* All four motors must agree to within rounding. */
    EXPECT_NEAR(T.T1, T.T2, 1e-5f);
    EXPECT_NEAR(T.T1, T.T3, 1e-5f);
    EXPECT_NEAR(T.T1, T.T4, 1e-5f);
}

TEST(Allocation, RollTorque_MotorsDifferential) {
    /* Apply a pure positive roll torque (L>0) with hover thrust.
     * Per the firmware mixer sign pattern:
     *   T1 = +aL ..., T2 = +aL ..., T3 = -aL ..., T4 = -aL ...
     * so motors 1 & 2 should spool up, motors 3 & 4 should spool down. */
    const float Fz = -1.182f * 9.80665f;
    const float dx = 0.1827f;
    const float dy = 0.1676f;

    thrvec T = Allocation(vec(0.5f, 0.0f, 0.0f), vec(dx, dy, 1.0f), Fz);

    EXPECT_GT(T.T1, -Fz / 4.0f);
    EXPECT_GT(T.T2, -Fz / 4.0f);
    EXPECT_LT(T.T3, -Fz / 4.0f);
    EXPECT_LT(T.T4, -Fz / 4.0f);
}

TEST(Allocation, ClampToZero_NoNegativeThrust) {
    /* Very large negative N (yaw torque) should still produce
     * non-negative per-motor thrusts (firmware clamps to [0, 100]). */
    thrvec T = Allocation(vec(0.0f, 0.0f, 500.0f), vec(0.18f, 0.17f, 1.0f), 0.0f);
    EXPECT_GE(T.T1, 0.0f);
    EXPECT_GE(T.T2, 0.0f);
    EXPECT_GE(T.T3, 0.0f);
    EXPECT_GE(T.T4, 0.0f);
}
