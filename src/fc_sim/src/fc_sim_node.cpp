// fc_sim_node.cpp
//
// Drives the simulated drone in Gazebo via fc_core. The control loop
// runs at the sim-clock rate (every /clock tick = every Gazebo physics
// step). IMU + range come from Gazebo through ros_gz_bridge; setpoints
// come from line_tracer (or any companion app) on /fc/setpoint. Motor
// commands go back through the bridge as actuator_msgs/Actuators.
//
// The sim FC always operates in autonomous mode: a synthetic sbus_t is
// fed to Control() with armingflag set from the Setpoint.arm bit and
// RS=0 so the source mux picks COMP.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <fc_sim_msgs/msg/setpoint.hpp>
#include <fc_sim_msgs/msg/telemetry.hpp>
#include <fc_sim_msgs/msg/mcu_command.hpp>

#include <cmath>
#include <chrono>
#include <algorithm>

extern "C" {
#include "fc_core/controller.h"
#include "fc_core/protocol.h"
#include "fc_core/mission_ctrl.h"
#include "fc_core/linalg.h"
}

using namespace std::chrono_literals;

namespace {

// Default motor-thrust coefficient — must match motorConstant in the
// uav26_quad SDF. ω_i = sqrt(T_i / k_f). 8.0e-06 with ω_max 1050 rad/s
// models the real 2212 920KV / 4S power train (~900 gf max per motor).
constexpr double kDefaultMotorConstant = 8.0e-06;

// A mission command older than this is treated as stale; the mission outer
// loop then stops overwriting COMP and the legacy path / stale fallback holds.
constexpr uint32_t kMissionStaleMs = 300u;

// Conversion from sensor_msgs/Imu (FLU body, ENU world) to the frame
// the firmware's mixer + controller actually operate in.
//
// The mixer signs in Allocation() are NED for roll + yaw but FLU for
// pitch (T1..T4 patterns), and the firmware's quat_to_euler has
// eul.y = -asinf(sinp), so:
//   roll  : FLU == NED (x-axis common). pqr.x not flipped.
//   pitch : Euler ends up FLU (firmware quat's -asin combined with the
//           shim's -e.y); mixer's M sign is also FLU. pqr.y must ALSO
//           be FLU = +msg.angular_velocity.y. Flipping q.y broke this
//           (pitch_fw was FLU, pqr.y was NED) and caused positive-
//           feedback divergence on any pitch setpoint — masked
//           previously by the oversized body collision pinning the
//           drone before it could actually rotate.
//   yaw   : Euler is NED (shim's -e.z), mixer's N sign is NED, and
//           pqr.z = -msg.z = +omega_z_NED — all match.
// Accel was previously sign-flipped on y, z but is currently unused.
struct EulerNed {
    float roll;
    float pitch;
    float yaw;
};

EulerNed flu_quat_to_ned_euler(double w, double x, double y, double z) {
    quaternion q{(float)w, {(float)x, (float)y, (float)z}};
    vec3d e = quat_to_euler(q);
    EulerNed out;
    out.roll  = e.x;
    out.pitch = -e.y;
    out.yaw   = -e.z;
    return out;
}

}  // namespace


class FcSimNode : public rclcpp::Node {
public:
    FcSimNode()
        : Node("fc_sim_node")
    {
        // ---- Parameters ----
        motor_constant_ = this->declare_parameter<double>(
            "motor_constant", kDefaultMotorConstant);
        max_motor_omega_ = this->declare_parameter<double>(
            "max_motor_omega", 1050.0);
        publish_telemetry_hz_ = this->declare_parameter<double>(
            "telemetry_hz", 100.0);
        // Auto-prime the drone with a safe level-attitude hover thrust
        // from t=0 so gz physics can't tumble it during the unmotored
        // settle period. As soon as a real /fc/setpoint arrives, that
        // overrides. Hover prime stays armed until an external
        // unarmed setpoint disarms it.
        auto_hover_init_ = this->declare_parameter<bool>("auto_hover_init", true);
        // Feed-forward for the prime's altitude hold — the clean-run
        // hover point on the 900 g/motor 4S train (r52: 0.334; the
        // earlier 0.38 was measured on a zombie-contaminated run).
        auto_hover_thrust_ = this->declare_parameter<double>(
            "auto_hover_thrust_norm", 0.335);
        prime_alt_target_ = this->declare_parameter<double>(
            "prime_alt_target", 1.2);
        prime_kp_alt_ = this->declare_parameter<double>("prime_kp_alt", 0.5);
        prime_kv_alt_ = this->declare_parameter<double>("prime_kv_alt", 0.4);

        // ---- Subscriptions ----
        rclcpp::QoS qos_sensors(rclcpp::KeepLast(10));
        qos_sensors.best_effort();

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_sensors,
            [this](const sensor_msgs::msg::Imu::SharedPtr m) { onImu(*m); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_truth", qos_sensors,
            [this](const nav_msgs::msg::Odometry::SharedPtr m) { onOdom(*m); });

        sub_setpoint_ = create_subscription<fc_sim_msgs::msg::Setpoint>(
            "/fc/setpoint", rclcpp::QoS(10),
            [this](const fc_sim_msgs::msg::Setpoint::SharedPtr m) { onSetpoint(*m); });

        sub_mcu_ = create_subscription<fc_sim_msgs::msg::McuCommand>(
            "/fc/mcu_command", rclcpp::QoS(10),
            [this](const fc_sim_msgs::msg::McuCommand::SharedPtr m) { onMcuCommand(*m); });

        sub_clock_ = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(10),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr m) { onClock(*m); });

        // ---- Publications ----
        pub_actuators_ = create_publisher<actuator_msgs::msg::Actuators>(
            "/uav26_quad/command/motor_speed", rclcpp::QoS(10));

        pub_telemetry_ = create_publisher<fc_sim_msgs::msg::Telemetry>(
            "/fc/telemetry", rclcpp::QoS(10));

        // Telemetry timer — Node::create_timer honors use_sim_time.
        const auto period = std::chrono::duration<double>(1.0 / publish_telemetry_hz_);
        telem_timer_ = create_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publishTelemetry(); });

        ControllerInit();

        if (auto_hover_init_) {
            COMP.mode = 1;                             // attithrmode
            COMP.arm = 1;
            COMP.roll_sp = 0.0f;
            COMP.pitch_sp = 0.0f;
            COMP.yawrate_sp = 0.0f;
            COMP.vz_sp = 0.0f;
            COMP.thrust_norm = (float)auto_hover_thrust_;
            COMP.last_ms = 0;       // will be refreshed each /clock tick below
        }

        // Sim retune: the firmware gains were hand-tuned for the real
        // airframe. In gz the integral term winds up against IMU noise
        // and flips the drone within ~25 s. Disable rate integrators
        // and dial the rate kp down somewhat. The attitude PID is left
        // alone — it has no integrator and its kp damps tilt.
        const bool sim_retune = this->declare_parameter<bool>("sim_retune", true);
        if (sim_retune) {
            pid_rate.ki = vec(0.0f, 0.0f, 0.0f);
            // Sim retune: firmware gains were hand-tuned for the real
            // airframe. In sim the rate integrator winds up against
            // gz IMU noise, so zero rate_ki. Attitude/rate kp are kept
            // close to firmware defaults (0.8 / 0.4) — earlier rounds
            // halved them under the assumption the gz motor plant was
            // 1.3-1.6x hotter than the SDF said, but that turned out
            // to be a misdiagnosis of the now-fixed pitch sign bug
            // (q.y in the IMU shim) + the wedge-corner ground collision.
            // With both fixed, the firmware-native gains track cleanly.
            pid_rate.kp = vec(
                (float)this->declare_parameter<double>("rate_kp_p", 0.40),
                (float)this->declare_parameter<double>("rate_kp_q", 0.40),
                (float)this->declare_parameter<double>("rate_kp_r", 0.80));
            pid_euler.kp = vec(
                (float)this->declare_parameter<double>("atti_kp_roll",  0.80),
                (float)this->declare_parameter<double>("atti_kp_pitch", 0.80),
                0.0f);
            pid_euler.kd = vec(
                (float)this->declare_parameter<double>("atti_kd_roll",  0.20),
                (float)this->declare_parameter<double>("atti_kd_pitch", 0.20),
                0.0f);
            // Remove the firmware's SBUS-centering deadband entirely: it
            // zeroes SETPOINTS below the bound (measurement noise passes
            // regardless), and there are no sticks in sim. Even 0.001
            // (3 mrad/s of rate deadband) left a +/-7.5 mrad attitude
            // dead zone that turned mission FOLLOW_LINE trim commands
            // into a +/-0.4 m lateral limit cycle.
            fc_rate_deadband_factor = (float)this->declare_parameter<double>("rate_deadband", 0.0);
            fc_atti_deadband_factor = (float)this->declare_parameter<double>("atti_deadband", 0.0);
            RCLCPP_INFO(get_logger(),
                "sim retune: pid_rate.kp=(%.2f,%.2f,%.2f) ki=0; "
                "pid_euler.kp=(%.2f,%.2f,0) kd=(%.2f,%.2f,0)",
                pid_rate.kp.x, pid_rate.kp.y, pid_rate.kp.z,
                pid_euler.kp.x, pid_euler.kp.y,
                pid_euler.kd.x, pid_euler.kd.y);
        }

        RCLCPP_INFO(get_logger(),
            "fc_sim_node up. motor_constant=%.3e max_omega=%.1f rad/s telem=%.0f Hz",
            motor_constant_, max_motor_omega_, publish_telemetry_hz_);
    }

private:
    // --- subscriber callbacks ---
    void onImu(const sensor_msgs::msg::Imu& msg) {
        EulerNed e = flu_quat_to_ned_euler(
            msg.orientation.w, msg.orientation.x,
            msg.orientation.y, msg.orientation.z);

        euler_ned_ = vec((float)e.roll, (float)e.pitch, (float)e.yaw);
        pqr_ned_ = vec(
            (float)msg.angular_velocity.x,
            (float)msg.angular_velocity.y,        // FLU, matches pitch_fw frame
            (float)-msg.angular_velocity.z);      // NED, matches yaw_fw frame
        acc_ned_ = vec(
            (float)msg.linear_acceleration.x,
            (float)-msg.linear_acceleration.y,
            (float)-msg.linear_acceleration.z);
        have_imu_ = true;
    }

    void onOdom(const nav_msgs::msg::Odometry& msg) {
        // FLU body / ENU world. Until a real downward range sensor is
        // wired in, world-frame z is the altitude reading the firmware's
        // ESKFz would otherwise get from the Micolink lidar.
        alt_lidar_ = (float)msg.pose.pose.position.z;
        const float wx = (float)msg.pose.pose.position.x;   // ENU east
        const float wy = (float)msg.pose.pose.position.y;   // ENU north
        ned_pos_ = vec(
            (float)msg.pose.pose.position.y,     // ENU east  -> NED north (approx)
            (float)msg.pose.pose.position.x,     // ENU north -> NED east
            (float)-msg.pose.pose.position.z);   // ENU up    -> NED down
        // ENU heading of body +x from the odom quaternion; the mission outer
        // loop rotates world velocity into body FLU with this yaw.
        {
            const double qw = msg.pose.pose.orientation.w;
            const double qx = msg.pose.pose.orientation.x;
            const double qy = msg.pose.pose.orientation.y;
            const double qz = msg.pose.pose.orientation.z;
            yaw_enu_ = (float)std::atan2(2.0 * (qw * qz + qx * qy),
                                         1.0 - 2.0 * (qy * qy + qz * qz));
        }
        // World-frame vz for the auto-hover prime's damping term.
        // dt MUST come from the odometry message's own stamp: gating on
        // fc_now_ms (updated by a separate /clock subscription) races
        // the odom arrivals — dt reads 0 between clock ticks (vz stuck
        // at 0 while falling) and garbage across clock jumps (vz=20).
        uint32_t stamp_ms = (uint32_t)(msg.header.stamp.sec * 1000u
                          + msg.header.stamp.nanosec / 1000000u);
        if (std::fabs(alt_lidar_) < 50.0f && prime_prev_ms_ != 0
                && stamp_ms > prime_prev_ms_
                && (stamp_ms - prime_prev_ms_) < 500u) {
            float dt = (float)(stamp_ms - prime_prev_ms_) * 1e-3f;
            float vz = (alt_lidar_ - prime_prev_alt_) / dt;
            if (std::fabs(vz) < 30.0f) prime_vz_ = vz;
            // World-frame xy velocity for the mission velocity loop, same
            // stamp-gated finite difference as vz.
            float vx = (wx - prev_wx_) / dt;
            float vy = (wy - prev_wy_) / dt;
            if (std::fabs(vx) < 30.0f && std::fabs(vy) < 30.0f) {
                world_vx_ = vx;
                world_vy_ = vy;
            }
        }
        prime_prev_ms_ = stamp_ms;
        prime_prev_alt_ = alt_lidar_;
        prev_wx_ = wx;
        prev_wy_ = wy;
        have_odom_ = true;
    }

    void onSetpoint(const fc_sim_msgs::msg::Setpoint& msg) {
        fc_proto_down_t down{};
        down.mode = (uint8_t)((msg.mode & FC_PROTO_MODE_MASK)
                  | (msg.arm ? FC_PROTO_MODE_ARM_BIT : 0));
        down.roll_sp     = msg.roll_sp;
        down.pitch_sp    = msg.pitch_sp;
        down.yawrate_sp  = msg.yawrate_sp;
        down.vz_sp       = msg.vz_sp;
        down.thrust_norm = msg.thrust_norm;
        down.seq         = (uint8_t)(setpoint_seq_++);

        fc_proto_apply_down(&down, fc_now_ms);
        got_external_setpoint_ = true;
    }

    void onMcuCommand(const fc_sim_msgs::msg::McuCommand& msg) {
        // Fill the mission wire struct from the ROS message, mirroring
        // onSetpoint: arm folds into the mode byte's bit 0x80, confidences
        // map 0..1 -> 0..255, and the booleans pack into the flag bytes.
        fc_proto_mission_t m{};
        m.mode = (uint8_t)((msg.mode & FC_PROTO_MODE_MASK)
               | (msg.arm ? FC_PROTO_MODE_ARM_BIT : 0));
        m.mission_state    = msg.mission_state;
        m.seq              = msg.seq;
        m.node_x           = msg.node_x;
        m.node_y           = msg.node_y;
        m.move_direction   = msg.move_direction;
        m.target_altitude  = msg.target_altitude;
        m.line_dx          = msg.line_dx;
        m.line_dy          = msg.line_dy;
        m.line_angle_error = msg.line_angle_error;
        m.marker_error_x   = msg.marker_error_x;
        m.marker_error_y   = msg.marker_error_y;
        m.marker_yaw_error = msg.marker_yaw_error;
        m.vx_est           = msg.vx_est;
        m.vy_est           = msg.vy_est;
        m.marker_id        = msg.marker_id;
        m.line_confidence   = (uint8_t)std::clamp(msg.line_confidence   * 255.0f, 0.0f, 255.0f);
        m.marker_confidence = (uint8_t)std::clamp(msg.marker_confidence * 255.0f, 0.0f, 255.0f);
        m.flags = (uint8_t)(
              (msg.vertical_line         ? FC_PROTO_MFLAG_VERTICAL_LINE   : 0)
            | (msg.horizontal_line       ? FC_PROTO_MFLAG_HORIZONTAL_LINE : 0)
            | (msg.intersection_detected ? FC_PROTO_MFLAG_INTERSECTION    : 0)
            | (msg.intersection_forward  ? FC_PROTO_MFLAG_FWD             : 0)
            | (msg.intersection_left     ? FC_PROTO_MFLAG_LEFT            : 0)
            | (msg.intersection_right    ? FC_PROTO_MFLAG_RIGHT           : 0)
            | (msg.intersection_backward ? FC_PROTO_MFLAG_BACK            : 0)
            | (msg.marker_detected       ? FC_PROTO_MFLAG_MARKER_DETECTED : 0));
        m.flags2 = (uint8_t)(
              (msg.vel_est_valid ? FC_PROTO_MFLAG2_VEL_EST_VALID : 0)
            | (msg.emergency     ? FC_PROTO_MFLAG2_EMERGENCY     : 0));

        // Wire parity: round-trip through the codec exactly as onSetpoint's
        // down frame does, so the sim exercises encode/decode, not just apply.
        uint8_t buf[FC_PROTO_MISSION_LEN];
        fc_proto_mission_t decoded{};
        if (fc_proto_encode_mission(&m, buf)
                && fc_proto_decode_mission(buf, &decoded)) {
            fc_proto_apply_mission(&decoded, fc_now_ms);
            got_mission_command_ = true;
        }
    }

    void onClock(const rosgraph_msgs::msg::Clock& msg) {
        // Convert sim time to monotonic ms used by the controller's
        // stale-link logic.
        uint32_t now_ms = (uint32_t)(msg.clock.sec * 1000u
                                    + msg.clock.nanosec / 1000000u);
        fc_now_ms = now_ms;

        // Auto-hover-init keeps COMP fresh until a real setpoint OR a
        // mission command arrives (after which that path owns COMP.last_ms).
        // The prime is a real altitude hold, not a fixed thrust: a
        // constant near-hover thrust only cancels gravity, it does NOT
        // brake the spawn fall — every pre-2026-07 run slammed the
        // ground at 5-7 m/s and the "soft catch" was luck in how the
        // tumble settled (r44-r50). PD on altitude parks the drone at
        // prime_alt_target until the companion takes over, so the
        // pre-engagement phase never touches the ground at all.
        if (auto_hover_init_ && !got_external_setpoint_ && !got_mission_command_) {
            COMP.last_ms = now_ms;
            if (have_odom_) {
                // Rate-limited descent to the park altitude: command a
                // vertical-velocity target (never faster than 0.5 m/s)
                // and drive thrust with a P on the velocity error
                // around the hover feed-forward. A plain altitude PD
                // lets the fall accelerate through the thrust floor
                // and arrives at the park altitude too hot (r51 hit
                // the floor at ~3.6 m/s).
                float err = (float)prime_alt_target_ - alt_lidar_;
                float vz_des = std::clamp(
                    (float)prime_kp_alt_ * err, -0.5f, 0.5f);
                float thr = (float)auto_hover_thrust_
                          + (float)prime_kv_alt_ * (vz_des - prime_vz_);
                COMP.thrust_norm = std::clamp(thr, 0.20f, 0.60f);
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 500,
                    "prime: alt=%.2f vz=%.2f vz_des=%.2f thr=%.2f",
                    (double)alt_lidar_, (double)prime_vz_,
                    (double)vz_des, (double)COMP.thrust_norm);
            }
        }

        controlTick();
    }

    void controlTick() {
        if (!have_imu_) return;

        // Sanity gate. At sim startup gz's OdometryPublisher and IMU
        // sometimes report a 180°-tilted orientation before physics has
        // settled; the firmware reads this as "drone upside-down" and
        // commands a violent righting torque, which spins the drone on
        // the ground into a wedged state. Suppress motor output until
        // the IMU shows the drone within ~45° of level for a few
        // consecutive ticks.
        const float max_tilt_rad = 0.7854f;  // 45 deg
        bool level = std::fabs(euler_ned_.x) < max_tilt_rad
                  && std::fabs(euler_ned_.y) < max_tilt_rad;
        if (level) {
            level_streak_ = std::min<int>(level_streak_ + 1, 1000);
        } else {
            level_streak_ = 0;
        }
        const bool sanity_ok = level_streak_ > 20;   // ~40 ms at 500 Hz

        if (!sanity_ok) {
            // Publish zero motor speeds. Drone free-falls / sits;
            // critically, the firmware controller is NOT engaged so it
            // can't fight bad sensor data.
            actuator_msgs::msg::Actuators out;
            out.header.stamp = get_clock()->now();
            out.velocity.assign(4, 0.0);
            pub_actuators_->publish(out);
            return;
        }

        // Single-writer guard: two fc_sim instances silently fighting
        // over one drone (orphans from a previous run re-activated by
        // the new run's /clock) poisoned a week of runs (r42..r51).
        // count_publishers includes this node, so >1 means a rival.
        if (count_publishers("/uav26_quad/command/motor_speed") > 1) {
            RCLCPP_FATAL_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "another publisher on /uav26_quad/command/motor_speed — "
                "zombie fc_sim? Emitting zero motor speeds until it is "
                "gone (pkill fc_sim_node / parameter_bridge strays).");
            actuator_msgs::msg::Actuators out;
            out.header.stamp = get_clock()->now();
            out.velocity.assign(4, 0.0);
            pub_actuators_->publish(out);
            return;
        }

        // Mission outer loop precedence: when a fresh McuCommand exists, run
        // the MCU mission law and overwrite COMP for this tick. It runs after
        // any onSetpoint apply (a separate callback that already wrote COMP),
        // so if a Setpoint and a mission command are both fresh the mission
        // wins. With no fresh mission command this is a no-op and the legacy
        // Setpoint path stays byte-identical.
        if (MISSION.valid && (fc_now_ms - MISSION.last_ms) < kMissionStaleMs) {
            float dt = (ctrl_prev_ms_ != 0 && fc_now_ms > ctrl_prev_ms_)
                     ? (float)(fc_now_ms - ctrl_prev_ms_) * 1e-3f : 0.0f;
            fc_mission_meas_t meas{};
            meas.altitude = alt_lidar_;
            meas.vz = prime_vz_;
            const float c = std::cos(yaw_enu_), s = std::sin(yaw_enu_);
            meas.vx_body =  c * world_vx_ + s * world_vy_;   // world ENU -> body FLU
            meas.vy_body = -s * world_vx_ + c * world_vy_;
            meas.vel_valid = true;
            fc_mission_tick(&MISSION.cmd, &meas, dt);
        }
        ctrl_prev_ms_ = fc_now_ms;

        // Disarm: a fresh companion command (setpoint or mission) with
        // arm=false means touchdown/stop — motors off. Control()'s mux only
        // consults armingflag on the stale-link path, so gate here.
        if ((got_external_setpoint_ || got_mission_command_) && !COMP.arm) {
            actuator_msgs::msg::Actuators out;
            out.header.stamp = get_clock()->now();
            out.velocity.assign(4, 0.0);
            pub_actuators_->publish(out);
            return;
        }

        // Synthetic SBUS so the source mux in Control() routes to COMP
        // whenever the line_tracer (companion) has arm=true.
        sbus_t sbus{};
        sbus.armingflag = COMP.arm;
        sbus.RS = 0;        // autonomous source
        sbus.LS = (char)COMP.arm;

        thrvec T = Control(ned_pos_, vec(0, 0, 0), euler_ned_, pqr_ned_, sbus);

        publishMotorSpeeds(T);
    }

    void publishMotorSpeeds(const thrvec& T) {
        // Map firmware mixer indices (T1..T4) -> SDF rotor numbers
        // (rotor_0..rotor_3). Derivation in PROGRESS.md / design notes:
        //   SDF rotor_0 (FR) <- T4
        //   SDF rotor_1 (BL) <- T2
        //   SDF rotor_2 (FL) <- T1
        //   SDF rotor_3 (BR) <- T3
        // Velocity in rad/s from F = k_f * omega^2.
        auto to_omega = [&](float Tn) -> double {
            float t = std::max(0.0f, Tn);
            double w = std::sqrt((double)t / motor_constant_);
            return std::min(w, max_motor_omega_);
        };

        actuator_msgs::msg::Actuators out;
        out.header.stamp = get_clock()->now();
        out.velocity.resize(4);
        out.velocity[0] = to_omega(T.T4);  // FR
        out.velocity[1] = to_omega(T.T2);  // BL
        out.velocity[2] = to_omega(T.T1);  // FL
        out.velocity[3] = to_omega(T.T3);  // BR
        pub_actuators_->publish(out);
    }

    void publishTelemetry() {
        fc_sim_msgs::msg::Telemetry t{};
        t.state       = (uint8_t)((flag & 0xFFu));  // low byte of FC flag
        t.roll        = euler_ned_.x;
        t.pitch       = euler_ned_.y;
        t.yaw         = euler_ned_.z;
        t.p           = pqr_ned_.x;
        t.q           = pqr_ned_.y;
        t.r           = pqr_ned_.z;
        t.alt_lidar   = alt_lidar_;
        t.vbatt_volts = 12.6f;       // placeholder until battery monitor lands
        t.flag_word   = (uint16_t)(flag & 0xFFFFu);
        pub_telemetry_->publish(t);
    }

    // ---- members ----
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<fc_sim_msgs::msg::Setpoint>::SharedPtr sub_setpoint_;
    rclcpp::Subscription<fc_sim_msgs::msg::McuCommand>::SharedPtr sub_mcu_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_clock_;

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    rclcpp::Publisher<fc_sim_msgs::msg::Telemetry>::SharedPtr pub_telemetry_;
    rclcpp::TimerBase::SharedPtr telem_timer_;

    double motor_constant_;
    double max_motor_omega_;
    double publish_telemetry_hz_;

    vec3d euler_ned_ {0, 0, 0};
    vec3d pqr_ned_ {0, 0, 0};
    vec3d acc_ned_ {0, 0, 0};
    vec3d ned_pos_ {0, 0, 0};
    float alt_lidar_ = 0.0f;

    bool have_imu_  = false;
    bool have_odom_ = false;
    bool got_external_setpoint_ = false;
    bool got_mission_command_ = false;
    // World-frame xy velocity (ENU) and heading for the mission velocity loop.
    float world_vx_ = 0.0f;
    float world_vy_ = 0.0f;
    float yaw_enu_ = 0.0f;
    float prev_wx_ = 0.0f;
    float prev_wy_ = 0.0f;
    uint32_t ctrl_prev_ms_ = 0;
    bool auto_hover_init_ = false;
    double auto_hover_thrust_ = 0.335;
    double prime_alt_target_ = 1.2;
    double prime_kp_alt_ = 0.5;
    double prime_kv_alt_ = 0.4;
    float prime_vz_ = 0.0f;
    float prime_prev_alt_ = 0.0f;
    uint32_t prime_prev_ms_ = 0;
    int level_streak_ = 0;

    uint32_t setpoint_seq_ = 0;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FcSimNode>());
    rclcpp::shutdown();
    return 0;
}
