# fc_sim

ROS2 C++ wrapper around `fc_core` that flies the simulated drone in Gazebo Harmonic.

## Planned (Open)

- Tighten step-response damping. Hover is stable but a 0.1 rad attitude step overshoots ~3-5x before settling. line_tracer feeds small smoothly-varying setpoints so this is acceptable for integration tests; tuning further means trading some hover stiffness for damping (lower kp_atti or higher kd_atti).
- Tier C: end-to-end line_tracer in Gazebo (TAKEOFF -> LINE_FOLLOW reaches a marker).
- **Startup-tumble flake.** When the sim launches, gz Harmonic's OdometryPublisher reports the drone's orientation as `q=(0, 0, 1, 0)` (= 180° about Y) for the first few physics steps before the simulation has settled. The firmware controller reads this as "drone upside-down" and commands a big righting torque, which actually tumbles the drone on the ground into a wedged orientation it can't escape. Reproducible across spawn heights, settle delays, and the `auto_hover_init` early-engagement workaround. Fix likely lives at the SDF/plugin layer (suppress the OdometryPublisher first ~0.1 s, or seed the IMU with identity). Until that's fixed, `flight_demo.launch.py` is unreliable; `hover_demo.launch.py` works most of the time.

## Decisions

- Sim retune happens at the node level, not by editing firmware-source gains. `fc_sim_node` zeros `pid_rate.ki` and lowers the rate/atti deadband factors after `ControllerInit()`. The firmware build keeps the original 0.04 ki and 0.04 deadband.
- The gz-sim multicopter motor model produces clean equal-thrust hover, but `rotorDragCoefficient` and `rollingMomentCoefficient` are set to zero in the SDF — they introduced asymmetric attitude perturbation under tilt that doesn't match the firmware's IMU model.
- Empirical hover thrust_norm in sim is ~0.500-0.510 (the firmware's `-4 * 0.6 * g * thrust_norm` mapping intersects gz physics at this point). line_tracer's altitude-hold P-controller closes around this value.
- FLU↔NED Euler shim: pitch and yaw both flipped on entry. The firmware's `quat_to_euler` has `eul.y = -asin(sinp)`, and gz odom appears to produce an NED-like quaternion for multicopter models, so the double-flip lands on the firmware-expected sign.

## Done

### Stable hover + axis-correct attitude tracking (commit `56fe60c`, 2026-05-25)

After Phase A-D landed, the first sim flights showed the drone flipping within ~25 s of a flat setpoint. Investigation isolated three issues, fixed in a single commit:

1. `pid_rate.ki = 0.04` was winding up against Gazebo's IMU noise — exposed `pid_rate / pid_euler / pid_vel` from `controller.c` (removed `static`) so `fc_sim_node` can zero ki at boot.
2. The 0.04 rad/s rate-command deadband (designed for SBUS stick-center noise) suppressed companion setpoints below ~5°. Added `fc_rate_deadband_factor` / `fc_atti_deadband_factor` globals; sim collapses them to 0.001.
3. `rotorDragCoefficient=8.06e-5` + `rollingMomentCoefficient=1e-6` on each rotor perturbed body attitude under motion. Set both to zero in the SDF.

Also moved drone spawn from z=0.15 to z=2.0 so ground-contact friction doesn't mask hover dynamics.

Verification:
- Hover with `thrust_norm=0.51` for 8 s: qx, qy stay at 1e-12 (effectively zero). z climbs ~12 m at constant 0.4 m/s² (slight excess over weight).
- Pitch +0.1 rad step: drone tips and moves +X (forward in FLU). Overshoots to ~28° before damping back. Direction correct.
- Roll +0.1 rad step: drone slides -Y in world (right in FLU). Direction correct.

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
