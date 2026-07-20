/*
 * controller.c (port of UAV26 STM32 firmware controller.c; Author: Segang).
 *
 * Math, gains, mixer geometry and PID structure are identical to the
 * firmware copy. The single change is an input-source mux at the top of
 * Control(): when the SBUS SWR switch is in the high (autonomous)
 * position and the companion link is fresh, the per-tick setpoints come
 * from the COMP struct (populated by the protocol codec) instead of
 * SBUS sticks. SBUS still gates motors via armingflag (ch6/SWL); the
 * safety pilot can always preempt by flipping SWR off high. If
 * autonomous is selected but the companion link is stale (older than
 * COMP_STALE_MS), the mux falls back to level attitude and
 * slightly-below-hover thrust until the link returns or the pilot
 * switches out of autonomous.
 *
 * In sim, fc_sim_node hands the controller a synthesized sbus_t with
 * armingflag=1 and RS=0, so the same arbitration produces companion
 * control of the simulated drone without modifying Control() further.
 *
 * The same patched controller.c is intended to compile into the
 * embedded firmware as well.
 */

#include "fc_core/controller.h"

float lowpass(float input, float prev, float alpha) {
    return prev + alpha * (input - prev);
}

vec3d lowpass_vec3d(vec3d input, vec3d prev, float alpha) {
    vec3d out;
    out.x = prev.x + alpha * (input.x - prev.x);
    out.y = prev.y + alpha * (input.y - prev.y);
    out.z = prev.z + alpha * (input.z - prev.z);
    return out;
}

PIDvec pidvecInit(vec3d kp, vec3d ki, vec3d kd, vec3d integral_sat){
    PIDvec pid;
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integral = vec(0, 0, 0);
    pid.integral_sat_pos = integral_sat;
    pid.integral_sat_neg = mulf(-1, integral_sat);
    return pid;
}

/* Non-static so the sim node can retune after ControllerInit(): the
 * firmware gains match the real airframe, not the gz-sim motor model. */
PIDvec pid_rate;
PIDvec pid_euler;
PIDvec pid_vel;

/* Companion-setpoint state for the source mux below. The owner (sim node
 * or STM32 USART2 ISR) ticks fc_now_ms and writes COMP per good frame. */
compsp_t COMP = {0};
uint32_t fc_now_ms = 0;

void ControllerInit(void){
    float kp = 0.8f;
    float mulpq = 0.5f;
    float muly = 1.0f;
    float mulrp = 0.8f;
    float mulvxy = 1.0f;
    float mulvz = 10.0f;

    pid_rate = pidvecInit(
                vec(kp * mulpq, kp * mulpq, kp * muly),
                vec(kp * mulpq * 0.1f, kp * mulpq * 0.1f, kp * muly * 0.1f),
                vec(0.0f, 0.0f, 0.0f),
                vec(0.75f, 0.75f, 0.75f));
    pid_euler = pidvecInit(
                vec(mulrp, mulrp, 0.0f),
                vec(0.0f, 0.0f, 0.0f),
                vec(0.1f, 0.1f, 0.0f),
                vec(0.0f, 0.0f, 0.0f));
    pid_vel = pidvecInit(
                vec(kp * mulvxy, kp * mulvxy, kp * mulvz),
                vec(kp * mulvxy * 0.1f, kp * mulvxy * 0.1f, kp * mulvz * 0.2f),
                vec(0.0f, 0.0f, 0.0f),
                vec(0.5f, 0.5f, 0.3f * 9.81f));
}

/* Raised from the firmware's 1.0 rad/s, which clipped the companion's
 * +/-2.5 rad/s yaw setpoints and left steady-state yaw drift; re-tune
 * on real hardware once mixer/motor asymmetries are characterized. */
static float maxratecmd = 3.0f;                       /* rad/s */
static float maxatticmd = 30.0f * PI / 180.0f;        /* rad   */

/* Fractions of maxratecmd / maxatticmd. The rate factor was scaled 1/3
 * when maxratecmd rose 1.0 -> 3.0 rad/s, keeping the absolute deadband
 * at 0.04 rad/s; fc_sim_node overrides both to 0.001 for sim. */
float fc_rate_deadband_factor = 0.0133f;
float fc_atti_deadband_factor = 0.04f;
static float maxvelXYcmd = 2.0f;
static float maxvelDcmd = 1.0f;

static float pcmd = 0;
static float qcmd = 0;
static float rcmd = 0;
static float rollcmd = 0;
static float pitchcmd = 0;
static float yawcmd = 0;
static float velXcmd = 0;
static float velYcmd = 0;
static float velDcmd = 0;

static float dx = 365.490f / 1000.0f / 2.0f;
static float dy = 335.235f / 1000.0f / 2.0f;
static float mass_org = 1.182f;
static float mass = 0.0f;

/* Per-motor max thrust, grams-force; 900 g matches the 4S power train
 * (bench 800-1000 g/motor). WARNING: F2PWM's curve is still calibrated
 * for the old 600 g train; thrust-stand recalibration before flight. */
static float max_thrust_g_per_motor = 900.0f;

static enum MODE mode = attithrmode;

/* Companion stale threshold, ms. 200 tolerates a 20-30 Hz publisher with
 * jitter; below ~50 the check races and yields phantom fallback ticks. */
#define COMP_STALE_MS 200u

/* Stale-link descent thrust, slightly below hover (~0.33 for this frame
 * on the 900 g/motor 4S train) so the drone settles gently. */
#define COMP_STALE_THRUST_NORM 0.27f

thrvec Control(vec3d NED, vec3d vel, vec3d Euler, vec3d pqr, sbus_t sbus){
    (void)NED; (void)vel;
    vec3d LMN = vec(0.0f, 0.0f, 0.0f); float thrust = 0.0f;

    /* ---------- input source mux (companion vs SBUS sticks) ---------- */
    bool autonomous_armed = (sbus.RS == 0);
    bool comp_fresh = autonomous_armed
                   && ((fc_now_ms - COMP.last_ms) < COMP_STALE_MS);

    /* Normalized stick-equivalents fed into the rest of the loop. */
    float roll_in  = sbus.rollnorm;
    float pitch_in = sbus.pitchnorm;
    float yaw_in   = sbus.yawnorm;
    float thr_in   = sbus.thrnorm;

    if (comp_fresh) {
        /* Companion setpoints (rad, rad/s) mapped back to the [-1,1]/[0,1]
         * SBUS convention so the rest of Control() matches the firmware. */
        roll_in  = clampfloat(COMP.roll_sp    / maxatticmd, -1.0f, 1.0f);
        pitch_in = clampfloat(COMP.pitch_sp   / maxatticmd, -1.0f, 1.0f);
        yaw_in   = clampfloat(COMP.yawrate_sp / maxratecmd, -1.0f, 1.0f);
        thr_in   = clampfloat(COMP.thrust_norm, 0.0f, 1.0f);
    } else if (autonomous_armed) {
        /* Autonomous but companion stale: level attitude and below-hover
         * thrust until the link returns or the pilot leaves autonomous. */
        roll_in = 0.0f; pitch_in = 0.0f; yaw_in = 0.0f;
        thr_in  = COMP_STALE_THRUST_NORM * (sbus.armingflag ? 1.0f : 0.0f);
    }

    if(mode == pqrthrmode){
        pcmd = maxratecmd * roll_in;
        qcmd = maxratecmd * pitch_in;
        rcmd = maxratecmd * yaw_in;
        thrust = -4.0f * max_thrust_g_per_motor / 1000.0f * 9.81f * thr_in;

        LMN = RATEControl(vec(pcmd, qcmd, rcmd), pqr, 1.0f / 500.0f);
    }
    if(mode == attithrmode){
        rollcmd  = maxatticmd * roll_in;
        pitchcmd = maxatticmd * pitch_in;
        rcmd     = maxratecmd * yaw_in;
        thrust   = -4.0f * max_thrust_g_per_motor / 1000.0f * 9.81f * thr_in;

        vec3d pqr_cmd = ATTIControl(vec(rollcmd, pitchcmd, 0.0f), Euler, pqr);
        pqr_cmd.z = rcmd;

        LMN = RATEControl(pqr_cmd, pqr, 1.0f / 500.0f);
    }

    return Allocation(LMN, vec(dx, dy, 1.0f), thrust);
}

vec3d RATEControl(vec3d pqrdes, vec3d pqr, float dt){
    vec3d _LMN;
    vec3d pqr_des = deadbandvec(pqrdes, mulf(maxratecmd * fc_rate_deadband_factor, vec(1.0f, 1.0f, 1.0f)));
    vec3d pqr_err = subv(pqr_des, pqr);
    pid_rate.integral = clampvec(addv(pid_rate.integral, mulf(dt, pqr_err)), pid_rate.integral_sat_neg, pid_rate.integral_sat_pos);
    _LMN = addv(mulv(pid_rate.kp, pqr_err), mulv(pid_rate.ki, pid_rate.integral));
    return _LMN;
}

vec3d ATTIControl(vec3d attides, vec3d atti, vec3d pqr){
    vec3d pqr_des;
    vec3d att_des = deadbandvec(attides, mulf(maxatticmd * fc_atti_deadband_factor, vec(1.0f, 1.0f, 1.0f)));
    vec3d att_err = subv(att_des, atti);
    pqr_des = subv(mulv(pid_euler.kp, att_err), mulv(pid_euler.kd, pqr));
    return pqr_des;
}

vec3d VELControl(vec3d veldes, vec3d vel, float dt){
    /* Stub in firmware copy; left as-is. */
    (void)veldes; (void)vel; (void)dt;
    return vec(0.0f, 0.0f, 0.0f);
}

/* KNOWN BUG (firmware parity): a and b are swapped between the roll and
 * pitch terms (~9% asymmetry); fix only with a paired firmware re-test. */
thrvec Allocation(vec3d LMN, vec3d dim, float Fz){
    thrvec T;
    float dxL = dim.x, dyL = dim.y;
    float a = 1.0f / (4.0f * dxL);
    float b = 1.0f / (4.0f * dyL);
    float L = LMN.x, M = LMN.y, N = LMN.z;
    T.T1 = 1.0f * clampfloat(+ a * L - b * M - N - Fz / 4.0f, 0.0f, 100.0f);
    T.T2 = 1.0f * clampfloat(+ a * L + b * M + N - Fz / 4.0f, 0.0f, 100.0f);
    T.T3 = 1.0f * clampfloat(- a * L + b * M - N - Fz / 4.0f, 0.0f, 100.0f);
    T.T4 = 1.0f * clampfloat(- a * L - b * M + N - Fz / 4.0f, 0.0f, 100.0f);
    return T;
}

thrvec16 F2PWM(thrvec T_force){
    thrvec16 res;
    res.T1 = clampuint16t((uint16_t)((T_force.T1 * (-0.0219f * T_force.T1 + 0.2968f)) * (2047.0f - 48.0f) + 48.0f), 48, 2047);
    res.T2 = clampuint16t((uint16_t)((T_force.T2 * (-0.0219f * T_force.T2 + 0.2968f)) * (2047.0f - 48.0f) + 48.0f), 48, 2047);
    res.T3 = clampuint16t((uint16_t)((T_force.T3 * (-0.0219f * T_force.T3 + 0.2968f)) * (2047.0f - 48.0f) + 48.0f), 48, 2047);
    res.T4 = clampuint16t((uint16_t)((T_force.T4 * (-0.0219f * T_force.T4 + 0.2968f)) * (2047.0f - 48.0f) + 48.0f), 48, 2047);
    return res;
}
