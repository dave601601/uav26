/*
 * lidar_parse.c
 *
 * Pure 27-byte Micolink rangefinder + optical-flow frame parser,
 * extracted from the firmware's lidar.c. Caller supplies an aligned
 * 27-byte buffer (byte 0 = 0xEF). DMA NDTR ring-buffer logic is dropped.
 */

#include "fc_core/lidar.h"

static inline uint32_t rd_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static inline int16_t rd_i16_le(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline float flow_to_mps(int16_t flow_cms_at_1m, float height_m) {
    return ((float)flow_cms_at_1m * height_m) / 100.0f;
}

static uint32_t lidar_prev_time = 0;

lidar_t LIDARparse_frame(const uint8_t* frame27, lidar_t prev) {
    lidar_t res = prev;
    res.valid = LIDAR_OUTDATED_DATA;

    if (frame27[ML_HEAD] != ML_HEAD_VAL ||
        frame27[ML_MSG]  != ML_MSG_ID_VAL ||
        frame27[ML_LEN]  != ML_LEN_VAL) {
        return res;
    }

    uint8_t checksum_read = frame27[ML_CHK];
    uint8_t checksum_calc = 0;
    for (uint8_t i = 0; i < ML_CHK; i++) {
        checksum_calc = (uint8_t)(checksum_calc + frame27[i]);
    }
    if (checksum_read != checksum_calc) {
        res.valid = LIDAR_CHECKSUM_FAIL;
        return res;
    }

    uint32_t time_ms = rd_u32_le(&frame27[PL_TIME0]);
    if (time_ms == lidar_prev_time) {
        return res;
    }
    lidar_prev_time = time_ms;

    uint32_t dist_mm = rd_u32_le(&frame27[PL_DIST0]);
    int16_t  fv_x    = rd_i16_le(&frame27[PL_FLOWVX0]);
    int16_t  fv_y    = rd_i16_le(&frame27[PL_FLOWVY0]);

    float dist_m = (float)dist_mm * 0.001f;
    float height_m = dist_m;

    res.valid       = LIDAR_NEW_DATA;
    res.dev_id      = frame27[ML_DEV];
    res.sys_id      = frame27[ML_SYS];
    res.seq         = frame27[ML_SEQ];
    res.time_ms     = time_ms;

    res.distance_m  = dist_m;
    res.vel_x_mps   = flow_to_mps(fv_x, height_m);
    res.vel_y_mps   = flow_to_mps(fv_y, height_m);

    res.strength    = frame27[PL_STRENGTH];
    res.precision   = frame27[PL_PRECISION];
    res.dis_status  = frame27[PL_DISSTAT];
    res.flow_quality= frame27[PL_FLOWQUAL];
    res.flow_status = frame27[PL_FLOWSTAT];

    if (dist_mm == 0) {
        res.valid = LIDAR_FRAME_FAIL;
    }

    return res;
}
