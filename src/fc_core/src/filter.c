/*
 * filter.c (verbatim port from UAV26 STM32 firmware; Author: Segang).
 *
 * 6x6 ESKF for IMU+GPS, and a 2-state ESKF for LiDAR-driven altitude.
 */

#include "fc_core/filter.h"
#include <string.h>

static void mat6_zero(float A[6][6]) { memset(A, 0, sizeof(float) * 6 * 6); }

static void mat6_eye(float A[6][6]) {
    mat6_zero(A);
    for (int i = 0; i < 6; i++) A[i][i] = 1.0f;
}

static void mat6_transpose(const float A[6][6], float AT[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            AT[j][i] = A[i][j];
        }
    }
}

static void mat6_mul(const float A[6][6], const float B[6][6], float C[6][6]) {
    float tmp[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[k][j];
            tmp[i][j] = s;
        }
    }
    memcpy(C, tmp, sizeof(tmp));
}

static bool mat6_inv(const float A[6][6], float Ainv[6][6]) {
    float aug[6][12];

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            aug[i][j] = A[i][j];
            aug[i][j + 6] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < 6; col++) {
        int pivot = col;
        float max_abs = fabsf(aug[pivot][col]);

        for (int r = col + 1; r < 6; r++) {
            float v = fabsf(aug[r][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = r;
            }
        }

        if (max_abs < 1e-8f) return false;

        if (pivot != col) {
            for (int c = 0; c < 12; c++) {
                float t = aug[col][c];
                aug[col][c] = aug[pivot][c];
                aug[pivot][c] = t;
            }
        }

        float div = aug[col][col];
        for (int c = 0; c < 12; c++) aug[col][c] /= div;

        for (int r = 0; r < 6; r++) {
            if (r == col) continue;
            float f = aug[r][col];
            for (int c = 0; c < 12; c++) {
                aug[r][c] -= f * aug[col][c];
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            Ainv[i][j] = aug[i][j + 6];
        }
    }

    return true;
}

static float F[6][6];
static float Q[6][6];

void eskf6_init(eskf6_t *kf, float dt) {
    kf->pos_w = vec(0.0f, 0.0f, 0.0f);
    kf->vel_w = vec(0.0f, 0.0f, 0.0f);

    mat6_zero(kf->P);

    for (int i = 0; i < 3; i++) {
        kf->P[i][i]     = 10.0f * 10.0f;
        kf->P[i + 3][i + 3] = 3.0f * 3.0f;
    }

    kf->sigma_acc     = 0.5f;
    kf->sigma_gps_pos = 2.0f;
    kf->sigma_gps_vel = 0.5f;

    mat6_eye(F);
    mat6_zero(Q);

    for (int i = 0; i < 3; i++) {
        F[i][i + 3] = dt;
    }

    const float qa = kf->sigma_acc * kf->sigma_acc;

    for (int i = 0; i < 3; i++) {
        Q[i][i]         = 0.25f * dt * dt * dt * dt * qa;
        Q[i][i + 3]     = 0.5f  * dt * dt * dt * qa;
        Q[i + 3][i]     = Q[i][i + 3];
        Q[i + 3][i + 3] = dt * dt * qa;
    }
}

void eskf6_predict(eskf6_t *kf, vec3d acc_world, float dt) {
    kf->pos_w = addv( kf->pos_w, addv( mulf(dt, kf->vel_w), mulf(0.5f * dt * dt, acc_world) ));
    kf->vel_w = addv(kf->vel_w, mulf(dt, acc_world));

    float FP[6][6], FT[6][6], FPFt[6][6];
    mat6_mul(F, kf->P, FP);
    mat6_transpose(F, FT);
    mat6_mul(FP, FT, FPFt);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            kf->P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
}

bool eskf6_update_gps(eskf6_t *kf, vec3d gps_pos_w, vec3d gps_vel_w) {
    float y[6] = {
        gps_pos_w.x - kf->pos_w.x,
        gps_pos_w.y - kf->pos_w.y,
        gps_pos_w.z - kf->pos_w.z,
        gps_vel_w.x - kf->vel_w.x,
        gps_vel_w.y - kf->vel_w.y,
        gps_vel_w.z - kf->vel_w.z
    };

    float S[6][6];
    memcpy(S, kf->P, sizeof(S));

    const float rp = kf->sigma_gps_pos * kf->sigma_gps_pos;
    const float rv = kf->sigma_gps_vel * kf->sigma_gps_vel;

    for (int i = 0; i < 3; i++) S[i][i] += rp;
    for (int i = 3; i < 6; i++) S[i][i] += rv;

    float Sinv[6][6];
    if (!mat6_inv(S, Sinv)) return false;

    float K[6][6];
    mat6_mul(kf->P, Sinv, K);

    float dx[6] = {0};
    for (int i = 0; i < 6; i++) {
        float s = 0.0f;
        for (int k = 0; k < 6; k++) s += K[i][k] * y[k];
        dx[i] = s;
    }

    kf->pos_w.x += dx[0];
    kf->pos_w.y += dx[1];
    kf->pos_w.z += dx[2];

    kf->vel_w.x += dx[3];
    kf->vel_w.y += dx[4];
    kf->vel_w.z += dx[5];

    float KP[6][6];
    float P_old[6][6];

    memcpy(P_old, kf->P, sizeof(P_old));
    mat6_mul(K, P_old, KP);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            kf->P[i][j] = P_old[i][j] - KP[i][j];
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            float s = 0.5f * (kf->P[i][j] + kf->P[j][i]);
            kf->P[i][j] = s;
            kf->P[j][i] = s;
        }
        if (kf->P[i][i] < 1e-6f) kf->P[i][i] = 1e-6f;
    }

    return true;
}

void eskfz_init(eskfz_t *kf, float dt) {
    (void)dt;
    kf->pz = 0.0f;
    kf->vz = 0.0f;

    kf->P[0][0] = 10.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    kf->sigma_az            = 0.5f;
    kf->sigma_lidar_pz      = 0.05f;
}

void eskfz_predict(eskfz_t *kf, float az, float dt) {
    kf->pz = kf->pz + kf->vz * dt + 0.5f * az * dt * dt;
    kf->vz = kf->vz + az * dt;

    float F00 = 1.0f, F01 = dt;
    float F10 = 0.0f, F11 = 1.0f;

    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];

    float FP00 = F00 * P00 + F01 * P10;
    float FP01 = F00 * P01 + F01 * P11;
    float FP10 = F10 * P00 + F11 * P10;
    float FP11 = F10 * P01 + F11 * P11;

    float Pn00 = FP00 * F00 + FP01 * F01;
    float Pn01 = FP00 * F10 + FP01 * F11;
    float Pn10 = FP10 * F00 + FP11 * F01;
    float Pn11 = FP10 * F10 + FP11 * F11;

    float qa = kf->sigma_az * kf->sigma_az;

    kf->P[0][0] = Pn00 + 0.25f * dt * dt * dt * dt * qa;
    kf->P[0][1] = Pn01 + 0.5f * dt * dt * dt * qa;
    kf->P[1][0] = Pn10 + 0.5f * dt * dt * dt * qa;
    kf->P[1][1] = Pn11 + dt * dt * qa;
}

void eskfz_update_lidar(eskfz_t *kf, float distance) {
    float y = distance - kf->pz;

    float R = kf->sigma_lidar_pz * kf->sigma_lidar_pz;
    float S = kf->P[0][0] + R;

    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->pz += K0 * y;
    kf->vz += K1 * y;

    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];

    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[1][0] = P10 - K1 * P00;
    kf->P[1][1] = P11 - K1 * P01;

    float s = 0.5f * (kf->P[0][1] + kf->P[1][0]);
    kf->P[0][1] = s;
    kf->P[1][0] = s;
}
