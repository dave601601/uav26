# fc_sim

ROS2 C++ wrapper around `fc_core` that flies the simulated drone in Gazebo Harmonic.

## Planned (Open)

- **Gazebo gain retune.** The firmware gains in `controller.c` are tuned for the real-airframe inertia. In sim, hover is unstable: at thrust_norm just below hover the drone is glued to the ground; once it breaks loose, even small IMU noise drives the cascade into a full flip within ~25 s. Either the rate-PID integrator winds up or the deadband interaction with low-amplitude noise pumps the controller. First investigation should disable the rate-PID integral term (set ki=0), then either reduce kp or widen the deadband. Without retuning, sim is launchable but the drone can't sustain attitude.
- Verify the FLU↔NED shim against a +10° pitch step once hover is stable — positive pitch in FLU should produce -X body motion. Currently can't run this test because the drone doesn't hold attitude.
- Tier C: end-to-end line_tracer in Gazebo (TAKEOFF -> LINE_FOLLOW reaches a marker). Blocked on stable hover.

## Done

### Package + node + Tier-A integration with world (commits `773f8af`, `0f5f036`, predecessors, 2026-05-25)

- `src/fc_sim_msgs/` — Setpoint (mode + arm + RPY/yawrate setpoints + vz + thrust_norm) and Telemetry (state + RPY + body rates + alt + battery placeholder + flag word) messages. Mirrors the byte layout in `fc_core/protocol.h`.
- `src/fc_sim/` — ament_cmake C++ package depending on `fc_core`, `actuator_msgs`, `sensor_msgs`, `nav_msgs`, `rosgraph_msgs`, `fc_sim_msgs`.
- `fc_sim_node.cpp` — single rclcpp node:
  - Subs: `/imu` (FLU body), `/odom_truth` (provides altitude in lieu of a dedicated range sensor), `/fc/setpoint`, `/clock`.
  - FLU->NED shim on IMU entry: euler pitch/yaw sign-flipped, body rates y/z flipped, accel y/z flipped. Single point of frame conversion.
  - On each `/clock` tick (= every Gazebo physics step, 2 ms = 500 Hz exactly): synthesize an `sbus_t` (armingflag from `COMP.arm`, RS=0) so Control()'s mux routes to companion setpoints; call `Control()`; convert per-motor thrust [N] -> rad/s via `ω = sqrt(T / motor_constant)` (default `k_f = 8.54858e-06`); publish `actuator_msgs/Actuators` on `/uav26_quad/command/motor_speed`.
  - Motor index mapping: SDF rotor_0 (FR) <- T4, rotor_1 (BL) <- T2, rotor_2 (FL) <- T1, rotor_3 (BR) <- T3. Derived from the firmware mixer's L/M/N sign pattern against the NED body axes.
  - 100 Hz telemetry timer publishes `fc_sim_msgs/Telemetry`.

Verified end-to-end:
- `colcon build` clean across fc_core / fc_sim_msgs / fc_sim / world / line_tracer / line_tracer_msgs.
- Sim launches headless and the topic graph comes up: `/clock`, `/imu`, `/odom_truth`, `/uav26_quad/command/motor_speed`, `/fc/setpoint`, `/fc/telemetry`, `/camera/*`.
- `actuator_msgs/Actuators <-> gz.msgs.Actuators` bridge is supported by `ros_gz_bridge` upstream (Plan B fallback to direct gz-transport not needed).
- fc_sim_node publishes motor commands at 500 Hz; gz applies them and drone responds (altitude changes with thrust_norm).
- Empirical thrust calibration TBD: at thrust_norm=0.49 the drone is glued to the ground (static friction or motor-model nonlinearity), and at thrust_norm=0.9 it accelerates >40 m/s². Hover is currently somewhere in between; see Open above for the retune plan.

## Decisions

- Sim FC is a standalone ROS2 node, not a Gazebo system plugin. /clock-driven tick at the world's 2 ms step = 500 Hz exactly matches the firmware's TIM2 ISR cadence. The same node binary will be the eventual hardware adapter once the byte protocol is wired to a real serial port.
- Motor numbering is intentionally inverted from the SDF rotor indices: SDF rotor_0 ≠ firmware T1. This is the cost of letting the firmware mixer stay verbatim against its NED convention while the SDF stays FLU.
