# fc_sim

ROS2 C++ wrapper around `fc_core` that flies the simulated drone in Gazebo Harmonic.

## Planned (Open)

- World-side integration: `world/launch/sim.launch.py` must spawn `fc_sim_node`. `bridge.yaml` must expose `/uav26_quad/command/motor_speed` (actuator_msgs/Actuators <-> gz.msgs.Actuators). `model.sdf` must drop the `MulticopterVelocityControl` plugin block (4× `MulticopterMotorModel` plugins stay).
- Verify the actuator_msgs <-> gz.msgs.Actuators bridge is supported by `ros_gz_bridge` on Jazzy. Fall back to direct gz-transport publishing from `fc_sim_node` if not.
- Verify the FLU<->NED shim by sending a +10° pitch setpoint and watching `/odom_truth` linear x decrease (positive pitch in FLU should produce -X body motion).

## Done

### Package skeleton + node (uncommitted, 2026-05-25)

- `src/fc_sim_msgs/` — Setpoint (mode + arm + RPY/yawrate setpoints + vz + thrust_norm) and Telemetry (state + RPY + body rates + alt + battery placeholder + flag word) messages. Mirrors the byte layout in `fc_core/protocol.h`.
- `src/fc_sim/` — ament_cmake C++ package depending on `fc_core`, `actuator_msgs`, `sensor_msgs`, `nav_msgs`, `rosgraph_msgs`, `fc_sim_msgs`.
- `fc_sim_node.cpp` — single rclcpp node:
  - Subs: `/imu` (FLU body), `/uav26_quad/range`, `/odom_truth`, `/fc/setpoint`, `/clock`.
  - FLU->NED shim on IMU entry: euler pitch/yaw sign-flipped, body rates y/z flipped, accel y/z flipped. Single point of frame conversion.
  - On each `/clock` tick: synthesize an `sbus_t` (armingflag from `COMP.arm`, RS=0) so Control()'s mux routes to companion setpoints; call `Control()`; convert per-motor thrust [N] -> rad/s via `ω = sqrt(T / motor_constant)` (default `k_f = 8.54858e-06`); publish `actuator_msgs/Actuators` on `/uav26_quad/command/motor_speed`.
  - Motor index mapping: SDF rotor_0 (FR) <- T4, rotor_1 (BL) <- T2, rotor_2 (FL) <- T1, rotor_3 (BR) <- T3. Derived from the firmware mixer's L/M/N sign pattern against the NED body axes; verify in Tier B.
  - 100 Hz telemetry timer publishes `fc_sim_msgs/Telemetry` from the cached IMU/range plus the global `flag` word.

## Decisions

- Sim FC is a standalone ROS2 node, not a Gazebo system plugin. /clock-driven tick at the world's 2 ms step = 500 Hz exactly matches the firmware's TIM2 ISR cadence. The same node binary will be the eventual hardware adapter once the byte protocol is wired to a real serial port.
- Motor numbering is intentionally inverted from the SDF rotor indices: SDF rotor_0 ≠ firmware T1. This is the cost of letting the firmware mixer stay verbatim against its NED convention while the SDF stays FLU.
