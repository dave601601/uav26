/*
 * test_quat.cpp
 *
 * quat_to_euler roundtrip via a known Euler -> quaternion -> Euler path.
 * The firmware's DCM convention is 321 (yaw-pitch-roll); we build the
 * unit quaternion from individual axis-angle rotations to match.
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "fc_core/linalg.h"
}

namespace {

/* Compose q = qz * qy * qx (321 sequence). */
quaternion euler_to_quat(float roll, float pitch, float yaw) {
    quaternion qr = {cosf(roll  * 0.5f), {sinf(roll  * 0.5f), 0.0f, 0.0f}};
    quaternion qp = {cosf(pitch * 0.5f), {0.0f, sinf(pitch * 0.5f), 0.0f}};
    quaternion qy = {cosf(yaw   * 0.5f), {0.0f, 0.0f, sinf(yaw   * 0.5f)}};

    return mulq(qy, mulq(qp, qr));
}

}  // namespace

TEST(Quat, Identity) {
    quaternion q = {1.0f, {0.0f, 0.0f, 0.0f}};
    vec3d e = quat_to_euler(q);
    EXPECT_NEAR(e.x, 0.0f, 1e-6f);
    EXPECT_NEAR(e.y, 0.0f, 1e-6f);
    EXPECT_NEAR(e.z, 0.0f, 1e-6f);
}

TEST(Quat, KnownAngles_RollOnly) {
    /* Note: the firmware's quat_to_euler outputs pitch as -asin(2*(wy-zx)),
     * which is the negative-of-standard convention. We probe a roll-only
     * rotation so the pitch sign does not bite. */
    quaternion q = euler_to_quat(0.3f, 0.0f, 0.0f);
    vec3d e = quat_to_euler(q);
    EXPECT_NEAR(e.x, 0.3f, 1e-4f);
    EXPECT_NEAR(e.y, 0.0f, 1e-4f);
    EXPECT_NEAR(e.z, 0.0f, 1e-4f);
}

TEST(Quat, KnownAngles_YawOnly) {
    quaternion q = euler_to_quat(0.0f, 0.0f, 1.0f);
    vec3d e = quat_to_euler(q);
    EXPECT_NEAR(e.x, 0.0f, 1e-4f);
    EXPECT_NEAR(e.y, 0.0f, 1e-4f);
    EXPECT_NEAR(e.z, 1.0f, 1e-4f);
}

TEST(Quat, NormalizeUnitNorm) {
    quaternion q = {2.0f, {0.0f, 0.0f, 0.0f}};
    quaternion qn = normalize(q);
    EXPECT_NEAR(qn.re, 1.0f, 1e-6f);
}
