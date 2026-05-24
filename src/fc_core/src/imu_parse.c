/*
 * imu_parse.c
 *
 * Pure 26-byte EBIMU frame parser, extracted from the firmware's imu.c.
 * No HAL UART or HAL_Delay calls. Caller is responsible for aligning the
 * 26-byte buffer at the SOP (bytes 4..5 == 0x55 0x55).
 */

#include "fc_core/imu.h"
#include <string.h>

static uint8_t IMU_BUF[DMA_BUFFER_EBIMU];
static uint16_t imu_checksum_state = 0;
static uint16_t imu_prev_time = 0;

ebimu_t IMUparse_frame(const uint8_t* frame26, ebimu_t prev){
    ebimu_t res = prev;
    res.valid = IMU_OUTDATED_DATA;
    memcpy(IMU_BUF, frame26, DMA_BUFFER_EBIMU);

    if(!(IMU_BUF[SOP_MSB] == 0x55 && IMU_BUF[SOP_LSB] == 0x55)){
        return res;
    }

    uint16_t time = (uint16_t)(IMU_BUF[T_MSB] << 8) | IMU_BUF[T_LSB];
    if((time == 0x3C6F) || (time == imu_prev_time)) {
        return res;
    }
    imu_prev_time = time;

    int16_t qz16 = (int16_t)((uint16_t)(IMU_BUF[QZ_MSB] << 8) | IMU_BUF[QZ_LSB]);
    int16_t qy16 = (int16_t)((uint16_t)(IMU_BUF[QY_MSB] << 8) | IMU_BUF[QY_LSB]);
    int16_t qx16 = (int16_t)((uint16_t)(IMU_BUF[QX_MSB] << 8) | IMU_BUF[QX_LSB]);
    int16_t qw16 = (int16_t)((uint16_t)(IMU_BUF[QW_MSB] << 8) | IMU_BUF[QW_LSB]);

    int16_t gx16 = (int16_t)((uint16_t)(IMU_BUF[GX_MSB] << 8) | IMU_BUF[GX_LSB]);
    int16_t gy16 = (int16_t)((uint16_t)(IMU_BUF[GY_MSB] << 8) | IMU_BUF[GY_LSB]);
    int16_t gz16 = (int16_t)((uint16_t)(IMU_BUF[GZ_MSB] << 8) | IMU_BUF[GZ_LSB]);

    int16_t ax16 = (int16_t)((uint16_t)(IMU_BUF[AX_MSB] << 8) | IMU_BUF[AX_LSB]);
    int16_t ay16 = (int16_t)((uint16_t)(IMU_BUF[AY_MSB] << 8) | IMU_BUF[AY_LSB]);
    int16_t az16 = (int16_t)((uint16_t)(IMU_BUF[AZ_MSB] << 8) | IMU_BUF[AZ_LSB]);

    uint16_t checksum_read = ((uint16_t)(IMU_BUF[CHK_MSB] << 8) | IMU_BUF[CHK_LSB]);
    imu_checksum_state = 0;
    for (uint8_t i = 0; i < DMA_BUFFER_EBIMU; i++){
        if(i == CHK_MSB || i == CHK_LSB) continue;
        imu_checksum_state += IMU_BUF[i];
    }
    if(checksum_read != imu_checksum_state){
        res.valid = IMU_CHECKSUM_FAIL;
        return res;
    }

    res.valid = IMU_NEW_DATA;

    res.q.im.z = (float)qz16 / 10000.0f;
    res.q.im.y = (float)qy16 / 10000.0f;
    res.q.im.x = (float)qx16 / 10000.0f;
    res.q.re   = (float)qw16 / 10000.0f;

    res.pqr.x = (float)gx16 / 10.0f * D2R;
    res.pqr.y = (float)gy16 / 10.0f * D2R;
    res.pqr.z = (float)gz16 / 10.0f * D2R;

    res.acc.x = (float)ax16 / 1000.0f;
    res.acc.y = (float)ay16 / 1000.0f;
    res.acc.z = (float)az16 / 1000.0f;

    return res;
}
