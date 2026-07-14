/*
 * protocol.h
 *
 * Companion <-> FC binary frames carried over USART2 (real hardware) or
 * shared structs in the sim. The byte layout is the source of truth so
 * sim and real wire agree.
 *
 * Downlink (companion -> FC): 24 bytes little-endian. 200 Hz nominal.
 * Uplink   (FC -> companion): 40 bytes little-endian. 100 Hz nominal.
 * Mission  (companion -> FC): 33 bytes little-endian. 20-50 Hz nominal.
 *
 * Fixed-point conventions:
 *   Q14 = float * 16384, range +/-2.0 -> +/-32768 (used for setpoints in rad / rad/s)
 *   Q15 = float * 32767, range  0..1   -> 0..32767 (used for thrust_norm)
 *
 * Mission downlink byte layout (source of truth; encode/decode below and
 * the Python struct pack must match it exactly):
 *
 *   off  field                type   encoding
 *    0   magic                u8     0xA6
 *    1   version              u8     FC_PROTO_VERSION
 *    2   mode                 u8     ControlMode (bits 0..6) | arm bit 0x80
 *    3   mission_state        u8     MissionState (telemetry only)
 *    4   seq                  u8     wraps
 *    5   node_x               i8     grid index
 *    6   node_y               i8     grid index
 *    7   move_direction       u8     MoveDirection
 *    8   target_altitude      u16    cm, 0..10 m
 *   10   line_dx              i16    Q14 m,   clamp +/-2.0 (vertical line, body +y)
 *   12   line_dy              i16    Q14 m,   clamp +/-2.0 (horizontal line, body +x)
 *   14   line_angle_error     i16    Q14 rad, FLU +CCW
 *   16   marker_error_x       i16    Q14 m,   body +x
 *   18   marker_error_y       i16    Q14 m,   body +y
 *   20   marker_yaw_error     i16    Q14 rad
 *   22   vx_est               i16    Q14 m/s, body +x
 *   24   vy_est               i16    Q14 m/s, body +y
 *   26   marker_id            i8     -1 = none
 *   27   line_confidence      u8     0..255
 *   28   marker_confidence    u8     0..255
 *   29   flags                u8     see FC_PROTO_MFLAG_* below
 *   30   flags2               u8     see FC_PROTO_MFLAG2_* below
 *   31   crc16                u16    CRC16-CCITT over bytes 0..30
 *
 * Line information travels as the full [dx, dy, flag] triple: both
 * grid-line offsets (line_dx from the nearest vertical line, line_dy from
 * the nearest horizontal line) plus per-line presence bits (flags bit0/1).
 * The MCU picks the FOLLOW_LINE error by move_direction; the companion does
 * no axis selection for the offsets (only line_angle_error is travel-selected).
 */

#ifndef FC_CORE_PROTOCOL_H_
#define FC_CORE_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#include "fc_core/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_PROTO_DOWN_MAGIC    0xA5u
#define FC_PROTO_UP_MAGIC      0x5Au
#define FC_PROTO_MISSION_MAGIC 0xA6u
#define FC_PROTO_VERSION       0x01u

#define FC_PROTO_DOWN_LEN    24u
#define FC_PROTO_UP_LEN      40u
#define FC_PROTO_MISSION_LEN 33u

/* Bit 7 of the `mode` byte requests arm; bits 0..6 carry the MODE enum
 * (setpoint downlink) or the ControlMode enum (mission downlink). */
#define FC_PROTO_MODE_ARM_BIT 0x80u
#define FC_PROTO_MODE_MASK    0x7Fu

#define FC_PROTO_FLAG_TAKEOFF   (1u << 0)
#define FC_PROTO_FLAG_LAND      (1u << 1)
#define FC_PROTO_FLAG_HOLD_ALT  (1u << 2)

/* Mission `flags` byte (offset 29). bit0/bit1 are the per-line presence
 * flags of the [dx, dy, flag] contract (vertical -> line_dx, horizontal
 * -> line_dy); the branch/intersection/marker bits follow. */
#define FC_PROTO_MFLAG_VERTICAL_LINE    (1u << 0)
#define FC_PROTO_MFLAG_HORIZONTAL_LINE  (1u << 1)
#define FC_PROTO_MFLAG_INTERSECTION     (1u << 2)
#define FC_PROTO_MFLAG_FWD              (1u << 3)
#define FC_PROTO_MFLAG_LEFT             (1u << 4)
#define FC_PROTO_MFLAG_RIGHT            (1u << 5)
#define FC_PROTO_MFLAG_BACK             (1u << 6)
#define FC_PROTO_MFLAG_MARKER_DETECTED  (1u << 7)

/* Mission `flags2` byte (offset 30). */
#define FC_PROTO_MFLAG2_VEL_EST_VALID   (1u << 0)
#define FC_PROTO_MFLAG2_EMERGENCY       (1u << 1)

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

/* Mission downlink (McuCommand). All errors are metric/radian; the
 * companion converts pixels with altitude/f before sending. */
typedef struct {
    uint8_t  mode;                /* ControlMode | arm bit 0x80 */
    uint8_t  mission_state;       /* MissionState, telemetry */
    uint8_t  seq;
    int8_t   node_x;
    int8_t   node_y;
    uint8_t  move_direction;      /* MoveDirection */
    float    target_altitude;     /* m  (wire u16 cm) */
    float    line_dx;             /* m,   Q14, clamp +/-2.0, vertical line, body +y */
    float    line_dy;             /* m,   Q14, clamp +/-2.0, horizontal line, body +x */
    float    line_angle_error;    /* rad, Q14, FLU +CCW */
    float    marker_error_x;      /* m,   Q14, body +x */
    float    marker_error_y;      /* m,   Q14, body +y */
    float    marker_yaw_error;    /* rad, Q14 */
    float    vx_est;              /* m/s, Q14, body +x */
    float    vy_est;              /* m/s, Q14, body +y */
    int8_t   marker_id;           /* -1 = none */
    uint8_t  line_confidence;
    uint8_t  marker_confidence;
    uint8_t  flags;               /* FC_PROTO_MFLAG_* */
    uint8_t  flags2;              /* FC_PROTO_MFLAG2_* */
} fc_proto_mission_t;

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

bool fc_proto_encode_mission(const fc_proto_mission_t* in,
                             uint8_t out_buf[FC_PROTO_MISSION_LEN]);
bool fc_proto_decode_mission(const uint8_t in_buf[FC_PROTO_MISSION_LEN],
                             fc_proto_mission_t* out);

/* Convenience: write a decoded downlink frame into the global COMP
 * struct used by Control(). Updates COMP.last_ms with now_ms. */
void fc_proto_apply_down(const fc_proto_down_t* msg, uint32_t now_ms);

/* Store a decoded mission frame into the global mission-command state
 * (mission_ctrl.h) with now_ms as its freshness stamp. The outer loop
 * (fc_mission_tick) consumes it; the sim node / STM32 ISR calls this. */
void fc_proto_apply_mission(const fc_proto_mission_t* msg, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_PROTOCOL_H_ */
