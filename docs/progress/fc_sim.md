# fc_sim

ROS2 C++ wrapper around `fc_core` that flies the simulated drone in Gazebo Harmonic, plus Python helper nodes (`hover_pub.py`, `flight_demo_pub.py`) and launch files for canned demos.

## Planned (Open)

- **Bake in the overnight sweep results.** A `gain_sweep.py` run (847 cells × 3 repeats, ~7 h wall-clock) was kicked off on 2026-05-25 to find better `(rate_kp, atti_kp, atti_kd)` triples than the firmware defaults. The CSV lives at `sweep_logs/results.csv`; rerun the script to see the top 20 by median score, then update `fc_sim_node`'s sim-retune defaults to match. Current defaults are `rate_kp=0.20, atti_kp=0.40, atti_kd=0.20`.
- **Ground-contact flake (UNSOLVED).** Even at motors commanded ω=620 rad/s (gz-verified via `gz topic -e`, F_total ≈ 17 N vs weight 11.8 N), the drone refuses to lift if it touches the ground first. DartSim contact constraint absorbs the upward force. Reproducible at sustained thrust_norm=0.75. Lift only happens when gz happens to catch the drone in free-fall — pure init timing luck. Mitigations attempted: airborne spawn (z=1.5), takeoff burst, sanity gate. None reliable. Real fixes to try: drop `mu` / friction on the body collision; replace body collision box with a sphere; remove body collision entirely; or accept it and let line_tracer drive a takeoff sequence that works for the airframe.
- **Startup-tumble flake (mitigated).** gz Harmonic's OdometryPublisher and IMU sensor briefly report `q=(0, 0, 1, 0)` (= 180° about Y) for the first few physics steps. The firmware reads this as "drone upside-down" and commands a big righting torque, which spins the drone on the ground into a wedged orientation. The sanity gate in `fc_sim_node` (motors stay at zero until IMU shows tilt < 45° for 20 consecutive ticks ≈ 40 ms) suppresses the runaway. Drone now stays at identity orientation through startup, but lift still depends on whether gz lets it leave the ground.
- Tighten step-response damping. A 0.1 rad attitude step still overshoots ~3-5× before settling. line_tracer feeds small smoothly-varying setpoints so this is acceptable for integration tests; the gain sweep is searching for a better trade-off.
- Tier C: end-to-end line_tracer in Gazebo (TAKEOFF → LINE_FOLLOW reaches a marker).

## Decisions

- Sim retune happens at the node level, not by editing firmware-source gains. `fc_sim_node` zeros `pid_rate.ki` and lowers the rate/atti deadband factors after `ControllerInit()`. The firmware build keeps the original 0.04 ki and 0.04 deadband.
- All retunable gains and deadband factors are exposed as **launch arguments** on `sim.launch.py` (and inherited by `hover_demo` / `flight_demo`). Override at run time without rebuilding: `ros2 launch fc_sim hover_demo.launch.py atti_kp_pitch:=0.20 atti_kd_pitch:=0.30`.
- The gz-sim multicopter motor model produces clean equal-thrust hover, but `rotorDragCoefficient` and `rollingMomentCoefficient` are set to zero in the SDF — they introduced asymmetric attitude perturbation under tilt that doesn't match the firmware's IMU model.
- Empirical hover thrust_norm in sim is ~0.500-0.510 when airborne (the firmware's `-4 * 0.6 * g * thrust_norm` mapping intersects gz physics at this point). The `hover_pub.py` PD altitude hold closes around this value.
- **FLU↔NED Euler shim**: pitch and yaw both flipped on entry. The firmware's `quat_to_euler` has `eul.y = -asin(sinp)`, and gz odom appears to produce an NED-like quaternion for multicopter models, so the double-flip lands on the firmware-expected sign. Body rates p stays, q and r flip; accel ax stays, ay and az flip. Confirmed empirically: pitch_sp = +0.1 makes drone move +X (forward in FLU); roll_sp = +0.1 makes drone move -Y world (= right in FLU).
- **Motor index mapping**: SDF `rotor_0` (FR) ← firmware T4, `rotor_1` (BL) ← T2, `rotor_2` (FL) ← T1, `rotor_3` (BR) ← T3. Derived from the firmware mixer's L/M/N sign pattern against the NED body axes.
- Sim FC is a standalone ROS2 node, not a Gazebo system plugin. /clock-driven tick at the world's 2 ms step = 500 Hz exactly matches the firmware's TIM2 ISR cadence. The same node binary will be the eventual hardware adapter once the byte protocol is wired to a real serial port.
- Sanity gate is a safety mechanism for sim only. On real hardware, the IMU never lies about orientation at startup; the gate is harmless there (level_streak reaches threshold within 40 ms of power-on if the drone is sitting upright).

## Done

### Gain sweep tooling + interrupt-safe overnight runs (commits `00276a8`–`0d8d049`, 2026-05-25)

`scripts/gain_sweep.py` — parallel docker-compose-run harness that sweeps `(rate_kp, atti_kp, atti_kd)` over a log-spaced grid. Each cell launches `hover_demo.launch.py` headless under a unique `ROS_DOMAIN_ID`, parses telemetry from the log, scores by `RMS(z − target_alt) + 0.5 × worst-case` over the last 15 s.

- Four preset grids: `small` (4), `coarse` (27), `fine` (125), `overnight` (847 cells = 11×11×7, ~7.9 h at 4 parallel × 3 repeats × 40 s).
- `--repeats N` runs each cell N times and ranks by median (handles DartSim ground-stuck flake).
- Results stream to `sweep_logs/results.csv` with per-row fsync, so kills/reboots don't lose data. `--resume` skips already-completed `(cell_idx, repeat_idx)` rows.
- Output uses tqdm progress bar with ETA + runs-per-minute. Dependencies declared via PEP 723 inline metadata (`uv run --script` shebang) — no pip / apt install.
- Score is in metres; the script also prints `log10(score)` and a bracket guide (`< -1.3` excellent, `-1.3 to -0.7` good, `-0.7 to -0.3` noisy, `> -0.3` broken). Pick from the good band.
- Ctrl-C handler kills outstanding docker containers cleanly so they don't leak into the host.

### `hover_demo` + `flight_demo` launch files + Python publishers (commits `9621e91`–`5df1d19`)

`src/fc_sim/launch/hover_demo.launch.py`:
- Wraps `world/sim.launch.py` and a tiny rclpy node `hover_pub.py` that PD-controls thrust_norm against `/odom_truth.z` to hold a target altitude.
- Launch args: `target_altitude`, `hover_thrust_norm`, `kp_alt`, `kd_alt`, `settle_delay`, plus the sim-side gain args (`rate_kp_p/q/r`, `atti_kp_roll/pitch`, `atti_kd_roll/pitch`) inherited from `sim.launch.py`.
- One-command sim hover: `ros2 launch fc_sim hover_demo.launch.py`.

`src/fc_sim/scripts/hover_pub.py`:
- 100 Hz publisher of `fc_sim_msgs/Setpoint` with `mode=ATTITHR, arm=true`.
- PD altitude hold (`kp_alt=0.04, kd_alt=0.10` defaults); 1 Hz telemetry log of `z`, `vz`, `err`, `thrust`.
- **Takeoff burst**: when `z < 0.30 m` and the drone is nearly stationary, command `thrust_norm=0.85` instead of PD output. Falls back to PD once airborne. Designed to break static ground friction; sometimes works.
- Thrust clamped to `[0.40, 0.75]`.

`src/fc_sim/launch/flight_demo.launch.py` + `scripts/flight_demo_pub.py`:
- 12-phase scripted sequence on top of the same PD altitude hold: HOLD → PITCH_FORWARD → HOLD → PITCH_BACK → HOLD → ROLL_RIGHT → HOLD → ROLL_LEFT → HOLD → YAW_RIGHT → YAW_LEFT → HOLD.
- Phase 1 (initial HOLD) doubles as takeoff: stays here until the drone is at target altitude (±0.2 m) AND `|vz| < 0.2 m/s` before advancing.
- Launch args: `target_altitude`, `phase_duration` (default 5 s), `pitch_amp` / `roll_amp` / `yaw_amp` for maneuver magnitudes.
- Useful for axis-by-axis controller verification once gz lets the drone fly.

### Sanity gate + takeoff burst + airborne spawn (commit `5df1d19`, 2026-05-25)

Three defensive measures against the gz startup tumble + ground-stuck issues:

1. **Sanity gate** in `fc_sim_node`: don't publish motor speeds until the IMU has reported tilt < 45° for 20 consecutive ticks (~40 ms). Suppresses the controller's panic response to gz's spurious initial orientation report.
2. **Takeoff burst** in `hover_pub.py` (described above).
3. **Spawn pose** raised from z=0.15 to z=1.5 in `competition.sdf` so motors catch the drone in free-fall before it touches the ground.

After these fixes, the drone consistently stays at identity orientation through startup but **still doesn't reliably lift** — the DartSim ground-contact problem remains unsolved.

### Stable hover + axis-correct attitude tracking (commit `56fe60c`, 2026-05-25)

After Phase A-D landed, the first sim flights showed the drone flipping within ~25 s of a flat setpoint. Three issues fixed in a single commit:

1. `pid_rate.ki = 0.04` was winding up against Gazebo's IMU noise — exposed `pid_rate / pid_euler / pid_vel` from `controller.c` (removed `static`) so `fc_sim_node` can zero ki at boot.
2. The 0.04 rad/s rate-command deadband (designed for SBUS stick-center noise) suppressed companion setpoints below ~5°. Added `fc_rate_deadband_factor` / `fc_atti_deadband_factor` globals; sim collapses them to 0.001.
3. `rotorDragCoefficient=8.06e-5` + `rollingMomentCoefficient=1e-6` on each rotor perturbed body attitude under motion. Set both to zero in the SDF.

Verification:
- Hover with `thrust_norm=0.51` for 8 s: qx, qy stay at 1e-12 (effectively zero). z climbs ~12 m at constant 0.4 m/s² (slight excess over weight).
- Pitch +0.1 rad step: drone tips and moves +X (forward in FLU). Overshoots to ~28° before damping back. Direction correct.
- Roll +0.1 rad step: drone slides -Y in world (right in FLU). Direction correct.

### Package + node + Tier-A integration with world (commits `773f8af`, `0f5f036`, predecessors, 2026-05-25)

- `src/fc_sim_msgs/` — Setpoint + Telemetry messages. Mirror the byte layout in `fc_core/protocol.h`.
- `src/fc_sim/` — ament_cmake C++ package depending on `fc_core`, `actuator_msgs`, `sensor_msgs`, `nav_msgs`, `rosgraph_msgs`, `fc_sim_msgs`.
- `fc_sim_node.cpp` — single rclcpp node:
  - Subs: `/imu` (FLU body), `/odom_truth` (provides altitude in lieu of a dedicated range sensor), `/fc/setpoint`, `/clock`.
  - FLU→NED shim on IMU entry; synthetic `sbus_t` (RS=0, armingflag from `COMP.arm`) so Control()'s source mux routes to COMP.
  - `/clock`-driven control tick at 500 Hz exactly (one Gazebo physics step = 2 ms).
  - Per-motor thrust → ω via `sqrt(T / motor_constant)`; publishes `actuator_msgs/Actuators` on `/uav26_quad/command/motor_speed`.

Verified end-to-end:
- `colcon build` clean across fc_core / fc_sim_msgs / fc_sim / world / line_tracer / line_tracer_msgs.
- Sim topic graph: `/clock`, `/imu`, `/odom_truth`, `/uav26_quad/command/motor_speed`, `/fc/setpoint`, `/fc/telemetry`, `/camera/*`.
- `actuator_msgs/Actuators ↔ gz.msgs.Actuators` bridge supported by `ros_gz_bridge` upstream (no gz-transport fallback needed).

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

