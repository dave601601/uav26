/*
 * imu.h (HAL-less port)
 *
 * EBIMU-9DOF-V6 26-byte frame layout. HAL UART init dropped; pure parser
 * over a 26-byte buffer is exposed.
 */

#ifndef FC_CORE_IMU_H_
#define FC_CORE_IMU_H_

#include "fc_core/planner.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_BUFFER_EBIMU    26

typedef struct{
    quaternion q;
    vec3d pqr;
    vec3d acc;
    uint8_t valid;
} ebimu_t;

#define T_MSB   0
#define T_LSB   1
#define CHK_MSB 2
#define CHK_LSB 3
#define SOP_MSB 4
#define SOP_LSB 5
#define QZ_MSB  6
#define QZ_LSB  7
#define QY_MSB  8
#define QY_LSB  9
#define QX_MSB  10
#define QX_LSB  11
#define QW_MSB  12
#define QW_LSB  13
#define GX_MSB  14
#define GX_LSB  15
#define GY_MSB  16
#define GY_LSB  17
#define GZ_MSB  18
#define GZ_LSB  19
#define AX_MSB  20
#define AX_LSB  21
#define AY_MSB  22
#define AY_LSB  23
#define AZ_MSB  24
#define AZ_LSB  25

ebimu_t IMUparse_frame(const uint8_t* frame26, ebimu_t prev);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_IMU_H_ */
