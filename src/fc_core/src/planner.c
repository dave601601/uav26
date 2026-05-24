/*
 * planner.c (verbatim port from UAV26 STM32 firmware; Author: Segang).
 *
 * Holds the global flag word and a debug vector shared across the
 * controller stack. Intentionally near-empty.
 */

#include "fc_core/planner.h"

uint64_t flag = 0x0000;

vec3d debugvec;
