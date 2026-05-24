/*
 * filter.h (HAL-less port)
 *
 * 6-state position/velocity ESKF and a scalar-Z position/velocity ESKF
 * driven by LiDAR. Math is identical to the firmware copy; the only
 * change is replacing `#include "main.h"` with `linalg.h`.
 */

#ifndef FC_CORE_FILTER_H_
#define FC_CORE_FILTER_H_

#include "fc_core/linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vec3d pos_w;
    vec3d vel_w;

    float P[6][6];

    float sigma_acc;
    float sigma_gps_pos;
    float sigma_gps_vel;
} eskf6_t;

void eskf6_init(eskf6_t *kf, float dt);
void eskf6_predict(eskf6_t *kf, vec3d acc_world, float dt);
bool eskf6_update_gps(eskf6_t *kf, vec3d gps_pos_w, vec3d gps_vel_w);

typedef struct{
    float pz;
    float vz;
    float P[2][2];

    float sigma_az;
    float sigma_lidar_pz;
} eskfz_t;

void eskfz_init(eskfz_t *kf, float dt);
void eskfz_predict(eskfz_t *kf, float az, float dt);
void eskfz_update_lidar(eskfz_t *kf, float distance);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_FILTER_H_ */
