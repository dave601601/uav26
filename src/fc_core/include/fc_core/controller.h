/*
 * controller.h (port of firmware controller.h with the SBUS/UART
 * arbitration extension)
 *
 * controller.c remains bit-identical to the firmware copy except for the
 * input-source selector at the top of Control(), which picks either SBUS
 * sticks (manual) or a companion-supplied setpoint (compsp_t). The same
 * patched controller.c is meant to be cross-compiled into the embedded
 * firmware.
 */

#ifndef FC_CORE_CONTROLLER_H_
#define FC_CORE_CONTROLLER_H_

#include "fc_core/planner.h"
#include "fc_core/sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

float lowpass(float input, float prev, float alpha);
vec3d lowpass_vec3d(vec3d input, vec3d prev, float alpha);

typedef struct{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_sat_pos;
    float integral_sat_neg;
} PID;

typedef struct{
    vec3d kp;
    vec3d ki;
    vec3d kd;
    vec3d integral;
    vec3d integral_sat_pos;
    vec3d integral_sat_neg;
} PIDvec;

PID pidInit(float kp, float ki, float kd, float integral_sat);
PIDvec pidvecInit(vec3d kp, vec3d ki, vec3d kd, vec3d integral_sat);

/* PID state structs (defined in controller.c). Exposed for sim-side
 * retuning; the firmware uses the values set by ControllerInit. */
extern PIDvec pid_rate;
extern PIDvec pid_euler;
extern PIDvec pid_vel;

/* Deadband factors applied to RATEControl / ATTIControl (units of
 * maxratecmd / maxatticmd). Default 0.04 matches the firmware's SBUS
 * stick-center deadzone; fc_sim_node lowers this to ~0 to pass the
 * companion's precise setpoints through. */
extern float fc_rate_deadband_factor;
extern float fc_atti_deadband_factor;

enum MODE{
    pqrthrmode = 0,
    attithrmode,
    attivdmode,
    velmode,
    posmode
};

/* Companion setpoint struct populated either by the real USART2 RX ISR
 * on STM32 or by the sim FC node from a ROS Setpoint message. Layout is
 * shared between hardware and sim; the controller does not care which
 * side wrote it. */
typedef struct {
    float roll_sp;       /* rad */
    float pitch_sp;      /* rad */
    float yawrate_sp;    /* rad/s */
    float vz_sp;         /* m/s (vel mode only, currently unused) */
    float thrust_norm;   /* 0..1 */
    uint8_t mode;        /* enum MODE */
    uint8_t arm;         /* 0/1 */
    uint32_t last_ms;    /* monotonic ms of last good frame */
} compsp_t;

extern compsp_t COMP;

/* Caller (sim node or STM32 main) must keep this monotonic ms counter
 * fresh so the controller's stale-link check works. */
extern uint32_t fc_now_ms;

void ControllerInit(void);

thrvec Control(vec3d NED, vec3d vel, vec3d Euler, vec3d pqr, sbus_t sbus);

thrvec Allocation(vec3d LMN, vec3d dim, float Fz);
thrvec16 F2PWM(thrvec T_force);

vec3d RATEControl(vec3d pqrdes, vec3d pqr, float dt);
vec3d ATTIControl(vec3d attides, vec3d atti, vec3d pqr);
vec3d VELControl(vec3d veldes, vec3d vel, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_CONTROLLER_H_ */
