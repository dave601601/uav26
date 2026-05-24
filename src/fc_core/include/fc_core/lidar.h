/*
 * lidar.h (HAL-less port)
 *
 * Micolink rangefinder + optical-flow 27-byte frame layout from the
 * firmware. HAL UART/DMA init dropped; pure parser over a 27-byte
 * aligned frame is exposed.
 */

#ifndef FC_CORE_LIDAR_H_
#define FC_CORE_LIDAR_H_

#include "fc_core/planner.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_BUFFER_LIDAR    64
#define LIDAR_FRAME_LEN     27

#define ML_HEAD_VAL   0xEF
#define ML_MSG_ID_VAL 0x51
#define ML_LEN_VAL    20

#define ML_HEAD   0
#define ML_DEV    1
#define ML_SYS    2
#define ML_MSG    3
#define ML_SEQ    4
#define ML_LEN    5
#define ML_PAY0   6
#define ML_CHK    26

#define PL_TIME0      (ML_PAY0 + 0)
#define PL_DIST0      (ML_PAY0 + 4)
#define PL_STRENGTH   (ML_PAY0 + 8)
#define PL_PRECISION  (ML_PAY0 + 9)
#define PL_DISSTAT    (ML_PAY0 + 10)
#define PL_RSV1       (ML_PAY0 + 11)
#define PL_FLOWVX0    (ML_PAY0 + 12)
#define PL_FLOWVY0    (ML_PAY0 + 14)
#define PL_FLOWQUAL   (ML_PAY0 + 16)
#define PL_FLOWSTAT   (ML_PAY0 + 17)
#define PL_RSV2_0     (ML_PAY0 + 18)
#define PL_RSV2_1     (ML_PAY0 + 19)

typedef enum {
    LIDAR_OUTDATED_DATA = 0,
    LIDAR_NEW_DATA      = 1,
    LIDAR_CHECKSUM_FAIL = 2,
    LIDAR_FRAME_FAIL    = 3
} lidar_valid_e;

typedef struct {
    lidar_valid_e valid;

    uint8_t  dev_id;
    uint8_t  sys_id;
    uint8_t  seq;

    uint32_t time_ms;

    float    distance_m;
    float    vel_x_mps;
    float    vel_y_mps;

    uint8_t  strength;
    uint8_t  precision;
    uint8_t  dis_status;
    uint8_t  flow_quality;
    uint8_t  flow_status;
} lidar_t;

lidar_t LIDARparse_frame(const uint8_t* frame27, lidar_t prev);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_LIDAR_H_ */
