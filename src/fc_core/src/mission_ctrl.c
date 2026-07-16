/*
 * mission_ctrl.c
 *
 * MCU-side mission outer loop; see mission_ctrl.h for the architecture and
 * frame conventions. Ports compute_body_velocity + body_vel_to_atti_thr
 * from line_tracer/dead_reckoning.py, with the same gains and defaults
 * except kp_xy (see the note at fc_mission_gains).
 *
 * Pure C99. clampfloat / g0 / sqrtf come in through controller.h ->
 * planner.h -> linalg.h; COMP, fc_now_ms and the MODE enum from
 * controller.h. mission_ctrl writes COMP but never touches Control().
 */

#include "fc_core/mission_ctrl.h"
#include "fc_core/controller.h"
#include "fc_core/linalg.h"

/* Global mission-command state; fc_proto_apply_mission fills it. */
fc_mission_cmd_state_t MISSION = {0};

/* Defaults applied at load so the module is usable without an init call.
 *
 * kp_xy deviates from the Jetson default (0.8) on purpose. The legacy
 * waypoint law ran kp_xy through the max_vxy vector clamp with a raw
 * along-track demand of ~2.4 m/s, which squeezed the effective lateral
 * gain to ~0.13-0.2 on a 3 m leg. FOLLOW_LINE replaces that demand with
 * the constant cruise (0.2), removing the squeeze: kp_xy=0.8 became a
 * 4-6x stiffer lateral loop than the one ever flown, and the attitude
 * cascade cannot follow it (r77/dbg2b: roll lags the setpoint ~1.4 s at
 * 0.4x amplitude -> negative damping, +/-1.4 m growing weave). 0.2
 * restores the proven effective stiffness. */
fc_mission_gains_t fc_mission_gains = {
    /* line_tracer node velocity-shaping defaults (kp_xy: see above) */
    .kp_xy   = 0.2f,
    .kp_yaw  = 3.0f,
    .max_vxy = 0.4f,
    .max_wz  = 2.5f,
    .cruise  = 0.2f,
    /* SetpointGains (dead_reckoning.py) */
    .hover_thrust_norm   = 0.33f,
    .kp_alt_thrust       = 0.17f,
    .kd_alt_thrust       = 0.20f,
    .max_atti_setpoint   = 0.15f,
    .thrust_min          = 0.28f,
    .thrust_max          = 0.60f,
    .takeoff_z_threshold = 0.15f,
    .takeoff_thrust_norm = 0.43f,
    .kp_vel              = 0.10f,
    .land_cutoff_alt     = 0.12f,
    .land_descent_vz     = -0.30f,
};

void fc_mission_gains_default(fc_mission_gains_t* g) {
    if (!g) return;
    g->kp_xy   = 0.2f;   /* legacy-equivalent effective stiffness, see above */
    g->kp_yaw  = 3.0f;
    g->max_vxy = 0.4f;
    g->max_wz  = 2.5f;
    g->cruise  = 0.2f;
    g->hover_thrust_norm   = 0.33f;
    g->kp_alt_thrust       = 0.17f;
    g->kd_alt_thrust       = 0.20f;
    g->max_atti_setpoint   = 0.15f;
    g->thrust_min          = 0.28f;
    g->thrust_max          = 0.60f;
    g->takeoff_z_threshold = 0.15f;
    g->takeoff_thrust_norm = 0.43f;
    g->kp_vel              = 0.10f;
    g->land_cutoff_alt     = 0.12f;
    g->land_descent_vz     = -0.30f;
}

/* Write the outer-loop result into COMP. mode stays attithrmode (Control()
 * runs its attitude+thrust path); last_ms is refreshed from fc_now_ms so
 * Control()'s stale-link check treats the companion as fresh. */
static void write_comp(float roll_sp, float pitch_sp, float yawrate_sp,
                       float thrust_norm, uint8_t arm) {
    COMP.roll_sp     = roll_sp;
    COMP.pitch_sp    = pitch_sp;
    COMP.yawrate_sp  = yawrate_sp;
    COMP.thrust_norm = thrust_norm;
    COMP.vz_sp       = 0.0f;
    COMP.mode        = (uint8_t)attithrmode;
    COMP.arm         = arm;
    COMP.last_ms     = fc_now_ms;
}

/* Vector-magnitude clamp: cap |(vx, vy)| at max_vxy while preserving the
 * direction (an axis-wise clamp would freeze a saturated command at 45deg). */
static void clamp_vxy(float* vx, float* vy, float max_vxy) {
    float m = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if (m > max_vxy && m > 0.0f) {
        float s = max_vxy / m;
        *vx *= s;
        *vy *= s;
    }
}

/* FOLLOW_LINE body velocity: cruise along the travel axis, kp_xy * the
 * grid-line offset for that axis on the perpendicular axis, and
 * wz = kp_yaw * line_angle_error. +/-x travel drives body +y from line_dx
 * (nearest vertical line); +/-y travel drives body +x from line_dy (nearest
 * horizontal line). The caller gates this on the matching presence bit. */
static void line_velocity(const fc_proto_mission_t* c,
                          const fc_mission_gains_t* g,
                          float* vx, float* vy, float* wz) {
    float along = g->cruise;
    switch (c->move_direction) {
        case FC_DIR_X_POS: *vx = +along; *vy = g->kp_xy * c->line_dx; break;
        case FC_DIR_X_NEG: *vx = -along; *vy = g->kp_xy * c->line_dx; break;
        case FC_DIR_Y_POS: *vy = +along; *vx = g->kp_xy * c->line_dy; break;
        case FC_DIR_Y_NEG: *vy = -along; *vx = g->kp_xy * c->line_dy; break;
        default:           *vx = +along; *vy = g->kp_xy * c->line_dx; break;
    }
    *wz = clampfloat(g->kp_yaw * c->line_angle_error, -g->max_wz, g->max_wz);
}

/* Slow cruise in the commanded MoveDirection axis (SEARCH_LINE /
 * MOVE_TO_LANDMARK); no lateral or heading correction. */
static void cruise_velocity(uint8_t dir, float cruise, float* vx, float* vy) {
    switch (dir) {
        case FC_DIR_X_POS: *vx = +cruise; break;
        case FC_DIR_X_NEG: *vx = -cruise; break;
        case FC_DIR_Y_POS: *vy = +cruise; break;
        case FC_DIR_Y_NEG: *vy = -cruise; break;
        default:           *vx = +cruise; break;
    }
}

/* Port of body_vel_to_atti_thr: velocity intent (vx, vy, wz) + target
 * altitude -> COMP. target_alt <= 0.05 selects the LAND descent law. */
static void atti_thrust(float vx_cmd, float vy_cmd, float wz,
                        float target_alt, const fc_mission_meas_t* m,
                        uint8_t arm_req, const fc_mission_gains_t* g) {
    /* Touchdown: LAND drove the target to ~0 and we are on the floor.
       Ordered first so no later clamp can resurrect the thrust. */
    if (target_alt <= 0.05f && m->altitude <= g->land_cutoff_alt) {
        write_comp(0.0f, 0.0f, 0.0f, 0.0f, 0u);
        return;
    }

    /* Attitude: a real velocity-error P when a body-velocity estimate is
       valid, else the open-loop v/g mapping (drag-bounded only). */
    float pitch_sp, roll_sp;
    if (m->vel_valid) {
        pitch_sp = +g->kp_vel * (vx_cmd - m->vx_body);
        roll_sp  = -g->kp_vel * (vy_cmd - m->vy_body);
    } else {
        pitch_sp = +vx_cmd / g0;
        roll_sp  = -vy_cmd / g0;
    }
    pitch_sp = clampfloat(pitch_sp, -g->max_atti_setpoint, g->max_atti_setpoint);
    roll_sp  = clampfloat(roll_sp,  -g->max_atti_setpoint, g->max_atti_setpoint);

    float alt_err = target_alt - m->altitude;
    float thrust;
    if (target_alt <= 0.05f) {
        /* LAND: track a fixed descent rate; trim-independent unlike the P law. */
        thrust = g->hover_thrust_norm
               + g->kd_alt_thrust * (g->land_descent_vz - m->vz);
        thrust = clampfloat(thrust, g->thrust_min, g->thrust_max);
    } else if (m->altitude < g->takeoff_z_threshold
               && m->vz < 0.2f
               && alt_err > 0.5f) {
        /* Takeoff burst: open-loop above hover to break ground contact.
           Suppressed once vz > 0.2 so the PD takes over after liftoff. */
        thrust = g->takeoff_thrust_norm;
    } else {
        thrust = g->hover_thrust_norm
               + g->kp_alt_thrust * alt_err
               - g->kd_alt_thrust * m->vz;
        thrust = clampfloat(thrust, g->thrust_min, g->thrust_max);
    }

    /* FLU +CCW wz -> firmware NED +CW. Load-bearing sign for the fleet. */
    write_comp(roll_sp, pitch_sp, -wz, thrust, arm_req);
}

void fc_mission_tick(const fc_proto_mission_t* cmd,
                     const fc_mission_meas_t* meas,
                     float dt) {
    (void)dt;  /* laws are proportional / PD on measured vz; no integrator */
    if (!cmd || !meas) return;

    const fc_mission_gains_t* g = &fc_mission_gains;
    uint8_t ctrl_mode = (uint8_t)(cmd->mode & FC_PROTO_MODE_MASK);
    uint8_t arm_req   = (cmd->mode & FC_PROTO_MODE_ARM_BIT) ? 1u : 0u;

    /* STOP: motors off, disarm immediately, ahead of any law. */
    if (ctrl_mode == FC_CTRL_STOP) {
        write_comp(0.0f, 0.0f, 0.0f, 0.0f, 0u);
        return;
    }

    bool vertical_line   = (cmd->flags & FC_PROTO_MFLAG_VERTICAL_LINE) != 0;
    bool horizontal_line = (cmd->flags & FC_PROTO_MFLAG_HORIZONTAL_LINE) != 0;
    bool marker_detected = (cmd->flags & FC_PROTO_MFLAG_MARKER_DETECTED) != 0;

    /* ---- body-velocity intent (compute_body_velocity port) ---- */
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    switch (ctrl_mode) {
        case FC_CTRL_FOLLOW_LINE: {
            /* Pick the followed line by travel axis and require its presence
               bit; missing bit -> zero velocity == HOLD behavior. */
            bool x_travel = (cmd->move_direction == FC_DIR_X_POS
                          || cmd->move_direction == FC_DIR_X_NEG);
            bool have = x_travel ? vertical_line : horizontal_line;
            if (have) line_velocity(cmd, g, &vx, &vy, &wz);
            break;
        }
        case FC_CTRL_ALIGN_MARKER:
            /* marker_detected=0 -> HOLD (covers the TAKEOFF climb). */
            if (marker_detected) {
                vx = g->kp_xy * cmd->marker_error_x;
                vy = g->kp_xy * cmd->marker_error_y;
                wz = clampfloat(g->kp_yaw * cmd->marker_yaw_error,
                                -g->max_wz, g->max_wz);
            }
            break;
        case FC_CTRL_SEARCH_LINE:
        case FC_CTRL_MOVE_TO_LANDMARK:
            cruise_velocity(cmd->move_direction, g->cruise, &vx, &vy);
            break;
        case FC_CTRL_LAND_ON_MARKER:
            /* Brake toward the marker while the land law descends. */
            vx = g->kp_xy * cmd->marker_error_x;
            vy = g->kp_xy * cmd->marker_error_y;
            break;
        case FC_CTRL_HOLD:
        case FC_CTRL_EMERGENCY_LAND:
        default:
            /* zero velocity */
            break;
    }
    clamp_vxy(&vx, &vy, g->max_vxy);

    /* ---- altitude target: LAND modes descend, others hold ---- */
    float target_alt;
    if (ctrl_mode == FC_CTRL_LAND_ON_MARKER || ctrl_mode == FC_CTRL_EMERGENCY_LAND)
        target_alt = 0.0f;
    else
        target_alt = cmd->target_altitude;

    atti_thrust(vx, vy, wz, target_alt, meas, arm_req, g);
}
