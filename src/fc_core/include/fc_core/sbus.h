/*
 * sbus.h (HAL-less port)
 *
 * Channel layout and the sbus_t struct are unchanged from the firmware.
 * The DMA-based SBUSinit() prototype is dropped; a pure 25-byte-frame
 * parser is exposed instead.
 */

#ifndef FC_CORE_SBUS_H_
#define FC_CORE_SBUS_H_

#include "fc_core/planner.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_BUFFER_SBUS 64

typedef struct{
    float thrnorm;
    float yawnorm;
    float rollnorm;
    float pitchnorm;
    float RPnorm;
    float LPnorm;
    char RS;
    char LS;
    uint8_t armingflag;
    uint8_t flag;
} sbus_t;

#define ROL 0
#define PIT 1
#define THR 2
#define YAW 3
#define SWR 4
#define PTR 5
#define SWL 6
#define PTL 7

/* Pure parser: takes a 25-byte SBUS frame already aligned at SOP (0x0F)
 * and the end-byte (0x00). Returns updated sbus_t. */
sbus_t SBUSparse_frame(const uint8_t* frame25, sbus_t prev);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_SBUS_H_ */
