/*
 * sbus_parse.c
 *
 * Pure 25-byte SBUS frame parser, extracted from the firmware's sbus.c.
 * Caller hands a 25-byte buffer pre-aligned at SOP (byte 0 = 0x0F,
 * byte 24 = 0x00); the DMA ring-buffer + NDTR alignment dance from the
 * firmware copy is dropped.
 */

#include "fc_core/sbus.h"

static uint16_t sbus_channels[16] = {0};
static bool sbus_connect_flag;
static bool sbus_valid_flag;

sbus_t SBUSparse_frame(const uint8_t* frame25, sbus_t prev) {
    sbus_t res = prev;

    sbus_connect_flag = !(frame25[23] & 0x08);

    flag |= (sbus_connect_flag ? FLAG_SBUS_READY : 0);
    flag &= ~FLAG_SBUS_CONNECT;
    flag |= (sbus_connect_flag ? FLAG_SBUS_CONNECT : 0);

    if (!(frame25[0] == 0x0F && frame25[24] == 0x00 && sbus_connect_flag)) {
        return res;
    }

    sbus_valid_flag = true;
    flag &= ~FLAG_SBUS_VALID;
    flag |= (sbus_valid_flag ? FLAG_SBUS_VALID : 0);

    sbus_channels[0]  = (frame25[1]  | (frame25[2] << 8)) & 0x07FF;
    sbus_channels[1]  = ((frame25[2] >> 3) | (frame25[3] << 5)) & 0x07FF;
    sbus_channels[2]  = ((frame25[3] >> 6) | (frame25[4] << 2) | (frame25[5] << 10)) & 0x07FF;
    sbus_channels[3]  = ((frame25[5] >> 1) | (frame25[6] << 7)) & 0x07FF;
    sbus_channels[4]  = ((frame25[6] >> 4) | (frame25[7] << 4)) & 0x07FF;
    sbus_channels[5]  = ((frame25[7] >> 7) | (frame25[8] << 1) | (frame25[9] << 9)) & 0x07FF;
    sbus_channels[6]  = ((frame25[9] >> 2) | (frame25[10] << 6)) & 0x07FF;
    sbus_channels[7]  = ((frame25[10] >> 5) | (frame25[11] << 3)) & 0x07FF;
    sbus_channels[8]  = (frame25[12] | (frame25[13] << 8)) & 0x07FF;
    sbus_channels[9]  = ((frame25[13] >> 3) | (frame25[14] << 5)) & 0x07FF;
    sbus_channels[10] = ((frame25[14] >> 6) | (frame25[15] << 2) | (frame25[16] << 10)) & 0x07FF;
    sbus_channels[11] = ((frame25[16] >> 1) | (frame25[17] << 7)) & 0x07FF;
    sbus_channels[12] = ((frame25[17] >> 4) | (frame25[18] << 4)) & 0x07FF;
    sbus_channels[13] = ((frame25[18] >> 7) | (frame25[19] << 1) | (frame25[20] << 9)) & 0x07FF;
    sbus_channels[14] = ((frame25[20] >> 2) | (frame25[21] << 6)) & 0x07FF;
    sbus_channels[15] = ((frame25[21] >> 5) | (frame25[22] << 3)) & 0x07FF;

    res.flag = frame25[23];

    bool arming = (sbus_channels[SWL] > 1000) * sbus_connect_flag;
    res.armingflag = (uint8_t)arming;
    res.thrnorm    = clampfloat(((float)sbus_channels[THR] - 200.0f) / 1600.0f,  0.0f, 1.0f) * (float)arming;
    res.rollnorm   = clampfloat(((float)sbus_channels[ROL] - 1000.0f) / 800.0f, -1.0f, 1.0f) * (float)arming;
    res.pitchnorm  = clampfloat(((float)sbus_channels[PIT] - 1000.0f) / 800.0f, -1.0f, 1.0f) * (float)arming;
    res.yawnorm    = clampfloat(((float)sbus_channels[YAW] - 1000.0f) / 800.0f, -1.0f, 1.0f) * (float)arming;
    res.RPnorm     = clampfloat(((float)sbus_channels[PTR] - 1000.0f) / 800.0f, -1.0f, 1.0f) * (float)arming;
    res.LPnorm     = clampfloat(((float)sbus_channels[PTL] - 1000.0f) / 800.0f, -1.0f, 1.0f) * (float)arming;
    res.LS         = (char)arming;
    res.RS         = (sbus_channels[SWR] > 600) ? ((sbus_channels[SWR] > 1400) ? 0 : 127) : (char)255;

    return res;
}
