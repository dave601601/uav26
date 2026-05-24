/*
 * test_atti_pid.cpp
 *
 * Closed-loop attitude tracking. Wraps the ATTI -> RATE cascade in a
 * minimal rigid-body model and checks that a step setpoint is tracked
 * within tolerance after a short transient.
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "fc_core/controller.h"
#include "fc_core/linalg.h"
}

namespace {

/* Crude single-axis rigid body: roll only.
 *   pdot = L / Ixx
 *   roll_dot = p
 * Inertia matches the SDF (Ixx ≈ 0.011 kg·m^2 after rescaling for
 * mass 1.182 kg). */
struct RollPlant {
    float roll = 0.0f;
    float p = 0.0f;
    float Ixx = 0.011f;

    void step(float L, float dt) {
        p    += (L / Ixx) * dt;
        roll += p * dt;
    }
};

}  // namespace

TEST(AttiPID, StepResponse_RollTracksWithin25Pct) {
    ControllerInit();

    RollPlant plant;
    const float dt = 1.0f / 500.0f;
    /* The firmware deadbands the rate command at maxratecmd*0.04 = 0.04 rad/s,
     * which leaves a steady-state attitude offset of ~0.05 rad. We pick a
     * setpoint large enough that the constant plateau falls inside 25%. */
    const float roll_des = 0.3f;      /* rad ≈ 17.2° */

    /* 5-second simulation (2500 ticks) so the cascade settles. */
    for (int i = 0; i < 2500; i++) {
        vec3d atti = vec(plant.roll, 0.0f, 0.0f);
        vec3d pqr  = vec(plant.p,    0.0f, 0.0f);

        vec3d pqr_des = ATTIControl(vec(roll_des, 0.0f, 0.0f), atti, pqr);
        vec3d LMN     = RATEControl(pqr_des, pqr, dt);
        plant.step(LMN.x, dt);
    }

    EXPECT_NEAR(plant.roll, roll_des, 0.25f * roll_des);
    EXPECT_LT(std::fabs(plant.p), 5.0f);
}

TEST(AttiPID, Hover_StaysNearZero) {
    ControllerInit();

    RollPlant plant;
    const float dt = 1.0f / 500.0f;

    /* Roll setpoint = 0 from rest. Plant should stay near zero. */
    for (int i = 0; i < 250; i++) {
        vec3d atti = vec(plant.roll, 0.0f, 0.0f);
        vec3d pqr  = vec(plant.p,    0.0f, 0.0f);

        vec3d pqr_des = ATTIControl(vec(0.0f, 0.0f, 0.0f), atti, pqr);
        vec3d LMN     = RATEControl(pqr_des, pqr, dt);
        plant.step(LMN.x, dt);
    }

    EXPECT_NEAR(plant.roll, 0.0f, 1e-3f);
}
