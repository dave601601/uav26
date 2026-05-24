/*
 * planner.h (HAL-less port)
 *
 * Originally part of the STM32 firmware; the HAL include is removed so
 * the header can compile on a host toolchain. The flag/mask defines are
 * preserved bit-for-bit with the firmware copy.
 */

#ifndef FC_CORE_PLANNER_H_
#define FC_CORE_PLANNER_H_

#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "fc_core/linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define _BV(x)      (1U << (x))

extern uint64_t flag;

#define FLAG_STARTED        _BV(0)
#define FLAG_ON_PROGRESS    _BV(1)
#define FLAG_ARMING         _BV(2)
#define FLAG_IMU_READY      _BV(3)
#define FLAG_SBUS_READY     _BV(4)
#define FLAG_SBUS_CONNECT   _BV(5)
#define FLAG_SBUS_VALID     _BV(6)
#define FLAG_GPS_READY      _BV(7)

#define IMU_OUTDATED_DATA   0
#define IMU_CHECKSUM_FAIL   1
#define IMU_NEW_DATA        2

#define MHZ 1000000

extern vec3d debugvec;

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_PLANNER_H_ */
