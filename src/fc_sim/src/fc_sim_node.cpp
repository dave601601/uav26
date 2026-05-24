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

#include <cmath>
#include <chrono>
#include <algorithm>

extern "C" {
#include "fc_core/controller.h"
#include "fc_core/protocol.h"
#include "fc_core/linalg.h"
}

using namespace std::chrono_literals;

namespace {

// Default motor-thrust coefficient — must match motorConstant in the
// uav26_quad SDF. ω_i = sqrt(T_i / k_f).
constexpr double kDefaultMotorConstant = 8.54858e-06;

// Conversion from sensor_msgs/Imu (FLU body, ENU world) to the
// firmware's NED body frame. Body rates: p stays, q and r flip.
// Acceleration: ax stays, ay and az flip. Euler angles derived from the
// orientation quaternion get pitch/yaw sign-flipped to match NED.
struct EulerNed {
    float roll;
    float pitch;
    float yaw;
};

EulerNed flu_quat_to_ned_euler(double w, double x, double y, double z) {
    quaternion q{(float)w, {(float)x, (float)y, (float)z}};
    vec3d e = quat_to_euler(q);  // firmware convention: roll, -pitch_std, yaw_std (close to FLU XYZ intrinsic)
    EulerNed out;
    out.roll  = e.x;
    out.pitch = -e.y;  // FLU -> NED pitch sign flip
    out.yaw   = -e.z;  // FLU -> NED yaw sign flip
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
            "max_motor_omega", 800.0);
        publish_telemetry_hz_ = this->declare_parameter<double>(
            "telemetry_hz", 100.0);

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
            (float)-msg.angular_velocity.y,
            (float)-msg.angular_velocity.z);
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
        ned_pos_ = vec(
            (float)msg.pose.pose.position.y,     // ENU east  -> NED north (approx)
            (float)msg.pose.pose.position.x,     // ENU north -> NED east
            (float)-msg.pose.pose.position.z);   // ENU up    -> NED down
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
    }

    void onClock(const rosgraph_msgs::msg::Clock& msg) {
        // Convert sim time to monotonic ms used by the controller's
        // stale-link logic.
        uint32_t now_ms = (uint32_t)(msg.clock.sec * 1000u
                                    + msg.clock.nanosec / 1000000u);
        fc_now_ms = now_ms;

        controlTick();
    }

    void controlTick() {
        if (!have_imu_) return;

        // Synthetic SBUS so the source mux in Control() routes to COMP
        // whenever the line_tracer (companion) has arm=true.
        sbus_t sbus{};
        sbus.armingflag = COMP.arm;
        sbus.RS = 0;        // autonomous source
        sbus.LS = (char)COMP.arm;
        // thr/roll/pitch/yaw_norm left at zero — Control()'s mux will
        // override them when comp is fresh.

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

    uint32_t setpoint_seq_ = 0;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FcSimNode>());
    rclcpp::shutdown();
    return 0;
}
