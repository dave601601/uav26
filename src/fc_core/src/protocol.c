/*
 * protocol.c
 *
 * Companion <-> FC fixed-frame binary codec for USART2. See protocol.h
 * for the byte layout.
 */

#include "fc_core/protocol.h"
#include "fc_core/linalg.h"
#include "fc_core/mission_ctrl.h"
#include <string.h>

/* -------- Little-endian field helpers -------- */
static inline void put_u8(uint8_t* p, uint8_t v) { p[0] = v; }
static inline uint8_t get_u8(const uint8_t* p)   { return p[0]; }

static inline void put_i8(uint8_t* p, int8_t v) { p[0] = (uint8_t)v; }
static inline int8_t get_i8(const uint8_t* p)   { return (int8_t)p[0]; }

static inline void put_u16_le(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline uint16_t get_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void put_i16_le(uint8_t* p, int16_t v) {
    put_u16_le(p, (uint16_t)v);
}
static inline int16_t get_i16_le(const uint8_t* p) {
    return (int16_t)get_u16_le(p);
}

static inline void put_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static inline uint32_t get_u32_le(const uint8_t* p) {
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static inline void put_f32_le(uint8_t* p, float v) {
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32_le(p, u);
}
static inline float get_f32_le(const uint8_t* p) {
    uint32_t u = get_u32_le(p);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

/* -------- Q-format fixed-point helpers -------- */
#define Q14_SCALE 16384.0f
#define Q15_SCALE 32767.0f

static inline int16_t f_to_q14(float v) {
    float s = v * Q14_SCALE;
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
}
static inline float q14_to_f(int16_t q) { return (float)q / Q14_SCALE; }

static inline uint16_t f_to_q15(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)(v * Q15_SCALE);
}
static inline float q15_to_f(uint16_t q) { return (float)q / Q15_SCALE; }

/* Altitude in metres <-> centimetres on the wire, clamped to 0..10 m. */
static inline uint16_t m_to_cm(float m) {
    float cm = m * 100.0f;
    if (cm < 0.0f)    cm = 0.0f;
    if (cm > 1000.0f) cm = 1000.0f;
    return (uint16_t)(cm + 0.5f);
}
static inline float cm_to_m(uint16_t cm) { return (float)cm / 100.0f; }

/* -------- CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) -------- */
uint16_t fc_proto_crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* -------- Downlink (companion -> FC) -------- */
bool fc_proto_encode_down(const fc_proto_down_t* in,
                          uint8_t out_buf[FC_PROTO_DOWN_LEN])
{
    if (!in || !out_buf) return false;

    put_u8 (out_buf + 0,  FC_PROTO_DOWN_MAGIC);
    put_u8 (out_buf + 1,  FC_PROTO_VERSION);
    put_u8 (out_buf + 2,  in->mode);
    put_u8 (out_buf + 3,  in->seq);
    put_i16_le(out_buf + 4,  f_to_q14(in->roll_sp));
    put_i16_le(out_buf + 6,  f_to_q14(in->pitch_sp));
    put_i16_le(out_buf + 8,  f_to_q14(in->yawrate_sp));
    put_i16_le(out_buf + 10, f_to_q14(in->vz_sp));
    put_u16_le(out_buf + 12, f_to_q15(in->thrust_norm));
    put_u32_le(out_buf + 14, in->timestamp_ms);
    put_u32_le(out_buf + 18, in->flags);

    uint16_t crc = fc_proto_crc16_ccitt(out_buf, FC_PROTO_DOWN_LEN - 2u);
    put_u16_le(out_buf + 22, crc);
    return true;
}

bool fc_proto_decode_down(const uint8_t in_buf[FC_PROTO_DOWN_LEN],
                          fc_proto_down_t* out)
{
    if (!in_buf || !out) return false;
    if (get_u8(in_buf + 0) != FC_PROTO_DOWN_MAGIC) return false;
    if (get_u8(in_buf + 1) != FC_PROTO_VERSION)    return false;

    uint16_t crc_calc = fc_proto_crc16_ccitt(in_buf, FC_PROTO_DOWN_LEN - 2u);
    uint16_t crc_read = get_u16_le(in_buf + 22);
    if (crc_calc != crc_read) return false;

    out->mode         = get_u8 (in_buf + 2);
    out->seq          = get_u8 (in_buf + 3);
    out->roll_sp      = q14_to_f(get_i16_le(in_buf + 4));
    out->pitch_sp     = q14_to_f(get_i16_le(in_buf + 6));
    out->yawrate_sp   = q14_to_f(get_i16_le(in_buf + 8));
    out->vz_sp        = q14_to_f(get_i16_le(in_buf + 10));
    out->thrust_norm  = q15_to_f(get_u16_le(in_buf + 12));
    out->timestamp_ms = get_u32_le(in_buf + 14);
    out->flags        = get_u32_le(in_buf + 18);
    return true;
}

/* -------- Uplink (FC -> companion) -------- */
bool fc_proto_encode_up(const fc_proto_up_t* in,
                        uint8_t out_buf[FC_PROTO_UP_LEN])
{
    if (!in || !out_buf) return false;

    put_u8 (out_buf + 0, FC_PROTO_UP_MAGIC);
    put_u8 (out_buf + 1, FC_PROTO_VERSION);
    put_u8 (out_buf + 2, in->state);
    put_u8 (out_buf + 3, in->seq);
    put_f32_le(out_buf + 4,  in->roll);
    put_f32_le(out_buf + 8,  in->pitch);
    put_f32_le(out_buf + 12, in->yaw);
    put_f32_le(out_buf + 16, in->p);
    put_f32_le(out_buf + 20, in->q);
    put_f32_le(out_buf + 24, in->r);
    put_f32_le(out_buf + 28, in->alt_lidar);
    put_f32_le(out_buf + 32, in->vbatt_volts);
    put_u16_le(out_buf + 36, in->flag_word);

    uint16_t crc = fc_proto_crc16_ccitt(out_buf, FC_PROTO_UP_LEN - 2u);
    put_u16_le(out_buf + 38, crc);
    return true;
}

bool fc_proto_decode_up(const uint8_t in_buf[FC_PROTO_UP_LEN],
                        fc_proto_up_t* out)
{
    if (!in_buf || !out) return false;
    if (get_u8(in_buf + 0) != FC_PROTO_UP_MAGIC) return false;
    if (get_u8(in_buf + 1) != FC_PROTO_VERSION)  return false;

    uint16_t crc_calc = fc_proto_crc16_ccitt(in_buf, FC_PROTO_UP_LEN - 2u);
    uint16_t crc_read = get_u16_le(in_buf + 38);
    if (crc_calc != crc_read) return false;

    out->state       = get_u8 (in_buf + 2);
    out->seq         = get_u8 (in_buf + 3);
    out->roll        = get_f32_le(in_buf + 4);
    out->pitch       = get_f32_le(in_buf + 8);
    out->yaw         = get_f32_le(in_buf + 12);
    out->p           = get_f32_le(in_buf + 16);
    out->q           = get_f32_le(in_buf + 20);
    out->r           = get_f32_le(in_buf + 24);
    out->alt_lidar   = get_f32_le(in_buf + 28);
    out->vbatt_volts = get_f32_le(in_buf + 32);
    out->flag_word   = get_u16_le(in_buf + 36);
    return true;
}

void fc_proto_apply_down(const fc_proto_down_t* msg, uint32_t now_ms) {
    if (!msg) return;
    COMP.mode         = (uint8_t)(msg->mode & FC_PROTO_MODE_MASK);
    COMP.arm          = (uint8_t)((msg->mode & FC_PROTO_MODE_ARM_BIT) ? 1u : 0u);
    COMP.roll_sp      = msg->roll_sp;
    COMP.pitch_sp     = msg->pitch_sp;
    COMP.yawrate_sp   = msg->yawrate_sp;
    COMP.vz_sp        = msg->vz_sp;
    COMP.thrust_norm  = msg->thrust_norm;
    COMP.last_ms      = now_ms;
}

/* -------- Mission downlink (companion -> FC) -------- */
bool fc_proto_encode_mission(const fc_proto_mission_t* in,
                             uint8_t out_buf[FC_PROTO_MISSION_LEN])
{
    if (!in || !out_buf) return false;

    put_u8 (out_buf + 0,  FC_PROTO_MISSION_MAGIC);
    put_u8 (out_buf + 1,  FC_PROTO_VERSION);
    put_u8 (out_buf + 2,  in->mode);
    put_u8 (out_buf + 3,  in->mission_state);
    put_u8 (out_buf + 4,  in->seq);
    put_i8 (out_buf + 5,  in->node_x);
    put_i8 (out_buf + 6,  in->node_y);
    put_u8 (out_buf + 7,  in->move_direction);
    put_u16_le(out_buf + 8,  m_to_cm(in->target_altitude));
    put_i16_le(out_buf + 10, f_to_q14(in->line_dx));
    put_i16_le(out_buf + 12, f_to_q14(in->line_dy));
    put_i16_le(out_buf + 14, f_to_q14(in->line_angle_error));
    put_i16_le(out_buf + 16, f_to_q14(in->marker_error_x));
    put_i16_le(out_buf + 18, f_to_q14(in->marker_error_y));
    put_i16_le(out_buf + 20, f_to_q14(in->marker_yaw_error));
    put_i16_le(out_buf + 22, f_to_q14(in->vx_est));
    put_i16_le(out_buf + 24, f_to_q14(in->vy_est));
    put_i8 (out_buf + 26, in->marker_id);
    put_u8 (out_buf + 27, in->line_confidence);
    put_u8 (out_buf + 28, in->marker_confidence);
    put_u8 (out_buf + 29, in->flags);
    put_u8 (out_buf + 30, in->flags2);

    uint16_t crc = fc_proto_crc16_ccitt(out_buf, FC_PROTO_MISSION_LEN - 2u);
    put_u16_le(out_buf + 31, crc);
    return true;
}

bool fc_proto_decode_mission(const uint8_t in_buf[FC_PROTO_MISSION_LEN],
                             fc_proto_mission_t* out)
{
    if (!in_buf || !out) return false;
    if (get_u8(in_buf + 0) != FC_PROTO_MISSION_MAGIC) return false;
    if (get_u8(in_buf + 1) != FC_PROTO_VERSION)       return false;

    uint16_t crc_calc = fc_proto_crc16_ccitt(in_buf, FC_PROTO_MISSION_LEN - 2u);
    uint16_t crc_read = get_u16_le(in_buf + 31);
    if (crc_calc != crc_read) return false;

    out->mode               = get_u8 (in_buf + 2);
    out->mission_state      = get_u8 (in_buf + 3);
    out->seq                = get_u8 (in_buf + 4);
    out->node_x             = get_i8 (in_buf + 5);
    out->node_y             = get_i8 (in_buf + 6);
    out->move_direction     = get_u8 (in_buf + 7);
    out->target_altitude    = cm_to_m(get_u16_le(in_buf + 8));
    out->line_dx            = q14_to_f(get_i16_le(in_buf + 10));
    out->line_dy            = q14_to_f(get_i16_le(in_buf + 12));
    out->line_angle_error   = q14_to_f(get_i16_le(in_buf + 14));
    out->marker_error_x     = q14_to_f(get_i16_le(in_buf + 16));
    out->marker_error_y     = q14_to_f(get_i16_le(in_buf + 18));
    out->marker_yaw_error   = q14_to_f(get_i16_le(in_buf + 20));
    out->vx_est             = q14_to_f(get_i16_le(in_buf + 22));
    out->vy_est             = q14_to_f(get_i16_le(in_buf + 24));
    out->marker_id          = get_i8 (in_buf + 26);
    out->line_confidence    = get_u8 (in_buf + 27);
    out->marker_confidence  = get_u8 (in_buf + 28);
    out->flags              = get_u8 (in_buf + 29);
    out->flags2             = get_u8 (in_buf + 30);
    return true;
}

void fc_proto_apply_mission(const fc_proto_mission_t* msg, uint32_t now_ms) {
    if (!msg) return;
    MISSION.cmd     = *msg;
    MISSION.last_ms = now_ms;
    MISSION.valid   = true;
}
