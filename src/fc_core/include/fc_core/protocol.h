/*
 * protocol.h
 *
 * Companion <-> FC binary frames carried over USART2 (real hardware) or
 * shared structs in the sim. The byte layout is the source of truth so
 * sim and real wire agree.
 *
 * Downlink (companion -> FC): 24 bytes little-endian. 200 Hz nominal.
 * Uplink   (FC -> companion): 40 bytes little-endian. 100 Hz nominal.
 *
 * Fixed-point conventions:
 *   Q14 = float * 16384, range +/-2.0 -> +/-32768 (used for setpoints in rad / rad/s)
 *   Q15 = float * 32767, range  0..1   -> 0..32767 (used for thrust_norm)
 */

#ifndef FC_CORE_PROTOCOL_H_
#define FC_CORE_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#include "fc_core/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_PROTO_DOWN_MAGIC  0xA5u
#define FC_PROTO_UP_MAGIC    0x5Au
#define FC_PROTO_VERSION     0x01u

#define FC_PROTO_DOWN_LEN    24u
#define FC_PROTO_UP_LEN      40u

/* Bit 7 of the `mode` byte requests arm; bits 0..6 carry the MODE enum. */
#define FC_PROTO_MODE_ARM_BIT 0x80u
#define FC_PROTO_MODE_MASK    0x7Fu

#define FC_PROTO_FLAG_TAKEOFF   (1u << 0)
#define FC_PROTO_FLAG_LAND      (1u << 1)
#define FC_PROTO_FLAG_HOLD_ALT  (1u << 2)

typedef struct {
    uint8_t  mode;          /* enum MODE | arm bit */
    uint8_t  seq;
    float    roll_sp;       /* rad */
    float    pitch_sp;      /* rad */
    float    yawrate_sp;    /* rad/s */
    float    vz_sp;         /* m/s, vel-mode only */
    float    thrust_norm;   /* 0..1 */
    uint32_t timestamp_ms;
    uint32_t flags;
} fc_proto_down_t;

typedef struct {
    uint8_t  state;
    uint8_t  seq;
    float    roll;
    float    pitch;
    float    yaw;
    float    p;
    float    q;
    float    r;
    float    alt_lidar;
    float    vbatt_volts;
    uint16_t flag_word;
} fc_proto_up_t;

uint16_t fc_proto_crc16_ccitt(const uint8_t* data, size_t len);

/* Encode/decode return true on success, false on CRC or magic mismatch. */
bool fc_proto_encode_down(const fc_proto_down_t* in,
                          uint8_t out_buf[FC_PROTO_DOWN_LEN]);
bool fc_proto_decode_down(const uint8_t in_buf[FC_PROTO_DOWN_LEN],
                          fc_proto_down_t* out);

bool fc_proto_encode_up(const fc_proto_up_t* in,
                        uint8_t out_buf[FC_PROTO_UP_LEN]);
bool fc_proto_decode_up(const uint8_t in_buf[FC_PROTO_UP_LEN],
                        fc_proto_up_t* out);

/* Convenience: write a decoded downlink frame into the global COMP
 * struct used by Control(). Updates COMP.last_ms with now_ms. */
void fc_proto_apply_down(const fc_proto_down_t* msg, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_PROTOCOL_H_ */
