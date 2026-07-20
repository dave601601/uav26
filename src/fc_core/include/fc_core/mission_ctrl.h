/*
 * mission_ctrl.h
 *
 * MCU-side mission outer loop. Pure C99 (no ROS, no dynamic allocation),
 * compiled into the fc_core sim library and, unchanged, into the STM32
 * firmware. It turns a decoded McuCommand (fc_proto_mission_t) plus the
 * MCU's own altitude / velocity measurements into the same COMP attitude
 * + thrust setpoint the legacy companion-setpoint path wrote, then
 * Control() (unchanged) runs the attitude/rate cascade below it.
 *
 * The two control laws are ported 1:1 from the Jetson reference in
 * line_tracer/dead_reckoning.py:
 *   compute_body_velocity  -> a body-velocity intent per ControlMode,
 *   body_vel_to_atti_thr   -> velocity-error P (or open-loop v/g) to
 *                             roll/pitch, altitude PD around hover with a
 *                             takeoff burst, and a land descent law.
 * Gains and defaults are the line_tracer node parameters and SetpointGains
 * values, except kp_xy (0.2, legacy-equivalent effective lateral
 * stiffness — rationale at fc_mission_gains); see fc_mission_gains_default().
 *
 * Frames: measurements and body-velocity intent are REP-103 FLU
 * (+x forward, +y left, +z up). The FLU->NED yawrate negation
 * (yawrate_sp = -wz) lives here, exactly as body_vel_to_atti_thr
 * documents; that sign is what the whole fleet's yaw lock depends on.
 */

#ifndef FC_CORE_MISSION_CTRL_H_
#define FC_CORE_MISSION_CTRL_H_

#include <stdint.h>
#include <stdbool.h>

#include "fc_core/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ControlMode, shared Jetson/MCU (MISSION_INTERFACE.md section 1). The
 * mission `mode` byte carries one of these in bits 0..6 plus arm bit 0x80. */
typedef enum {
    FC_CTRL_HOLD            = 0,
    FC_CTRL_FOLLOW_LINE     = 1,
    FC_CTRL_ALIGN_MARKER    = 2,
    FC_CTRL_SEARCH_LINE     = 3,
    FC_CTRL_MOVE_TO_LANDMARK = 4,
    FC_CTRL_LAND_ON_MARKER  = 5,
    FC_CTRL_STOP            = 6,
    FC_CTRL_EMERGENCY_LAND  = 7
} fc_control_mode_t;

/* MoveDirection, body FLU (+x forward, +y left). */
typedef enum {
    FC_DIR_X_POS = 0,
    FC_DIR_X_NEG = 1,
    FC_DIR_Y_POS = 2,
    FC_DIR_Y_NEG = 3
} fc_move_direction_t;

/* What the MCU measures / knows this tick. In sim the fc_sim_node fills
 * these from gz truth; on hardware altitude/vz come from the lidar and
 * (vx_body, vy_body) from the companion DR (vx_est/vy_est in the frame),
 * with vel_valid taken from flags2 vel_est_valid. */
typedef struct {
    float altitude;   /* m, world up */
    float vz;         /* m/s, world up */
    float vx_body;    /* m/s, body +x forward */
    float vy_body;    /* m/s, body +y left */
    bool  vel_valid;  /* (vx_body, vy_body) usable for the velocity loop */
} fc_mission_meas_t;

/* Outer-loop gains. The first block is the line_tracer node's
 * velocity-shaping parameters, the second SetpointGains (dead_reckoning). */
typedef struct {
    float kp_xy;               /* lateral/marker position P -> body vel */
    float kp_yaw;              /* heading error P -> wz */
    float max_vxy;             /* body velocity magnitude clamp, m/s */
    float max_wz;              /* yawrate clamp, rad/s */
    float cruise;              /* forward demand along move_direction, m/s */

    float hover_thrust_norm;   /* thrust at hover */
    float kp_alt_thrust;       /* thrust per m of altitude error */
    float kd_alt_thrust;       /* thrust per m/s of vz */
    float max_atti_setpoint;   /* roll/pitch clamp, rad */
    float thrust_min;
    float thrust_max;
    float takeoff_z_threshold; /* below this altitude the burst may fire, m */
    float takeoff_thrust_norm; /* open-loop burst thrust */
    float kp_vel;              /* body-velocity error P -> attitude */
    float land_cutoff_alt;     /* below this, LAND cuts thrust + disarms, m */
    float land_descent_vz;     /* LAND target descent rate, m/s (negative) */
} fc_mission_gains_t;

/* Latest decoded McuCommand plus its freshness stamp. fc_proto_apply_mission
 * writes it; the caller gates fc_mission_tick on (now - last_ms) < stale. */
typedef struct {
    fc_proto_mission_t cmd;
    uint32_t last_ms;
    bool     valid;
} fc_mission_cmd_state_t;

extern fc_mission_cmd_state_t MISSION;

/* Active gains. Initialized to fc_mission_gains_default() at load; exposed
 * so the sim node can retune for the gz plant, like pid_rate in controller.c. */
extern fc_mission_gains_t fc_mission_gains;

/* Fill g with the ported line_tracer / SetpointGains defaults. */
void fc_mission_gains_default(fc_mission_gains_t* g);

/* One outer-loop tick: map cmd + meas to COMP (roll_sp, pitch_sp,
 * yawrate_sp, thrust_norm, arm) using fc_mission_gains, and refresh
 * COMP.last_ms = fc_now_ms. dt is accepted for interface stability; the
 * ported laws are proportional / PD on measured vz and use no integrator. */
void fc_mission_tick(const fc_proto_mission_t* cmd,
                     const fc_mission_meas_t* meas,
                     float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_MISSION_CTRL_H_ */
