# fc_sim

ROS2 C++ wrapper around `fc_core` that flies the simulated drone in Gazebo Harmonic, plus Python helper nodes (`hover_pub.py`, `flight_demo_pub.py`) and launch files for canned demos.

## Done (recent)

### Prime altitude hold, disarm path, single-writer guard (2026-07-08)

- Auto-hover prime is now a real altitude hold (rate-limited descent
  to `prime_alt_target` 1.2 m, velocity-P around the hover
  feed-forward). The old fixed near-hover thrust only cancelled
  gravity — it never braked the spawn fall, so every run slammed the
  ground at 5-7 m/s and the "soft catch" was luck in how the tumble
  settled. The prime's vz derivative uses the odometry message stamp;
  gating on `fc_now_ms` (fed by a separate /clock subscription) raced
  the odom arrivals — dt read 0 between clock ticks (vz stuck at 0
  during the fall, r51) and garbage across jumps.
- Disarm: a fresh companion setpoint with arm=false zeroes the motors.
  `Control()`'s mux only consults armingflag on the stale-link path,
  so the gate lives in `controlTick`. line_tracer sends arm=false
  after its touchdown cutoff.
- Single-writer guard: if another publisher appears on
  `/uav26_quad/command/motor_speed`, log FATAL (throttled) and emit
  zero motor speeds. Orphaned fc_sim instances from earlier runs
  re-arm on the next run's /clock+/imu and silently fight it for the
  drone — this poisoned r42..r51 (see docker.md for the teardown
  contract).
- 4S defaults: kDefaultMotorConstant 8.0e-06, max_motor_omega 1050,
  auto_hover_thrust_norm 0.335 (r52 clean-run hover). Demo scripts
  (hover/flight/waypoint pubs + launches) rescaled by 0.667 so their
  commanded forces are unchanged.

## Planned (Open)

- **Startup-tumble flake (mitigated).** gz Harmonic's OdometryPublisher and IMU sensor briefly report `q=(0, 0, 1, 0)` (= 180° about Y) for the first few physics steps. The firmware reads this as "drone upside-down" and commands a big righting torque, which spins the drone on the ground into a wedged orientation. The sanity gate in `fc_sim_node` (motors stay at zero until IMU shows tilt < 45° for 20 consecutive ticks ≈ 40 ms) suppresses the runaway. Drone now stays at identity orientation through startup.
- Tier C: end-to-end line_tracer in Gazebo (TAKEOFF → LINE_FOLLOW reaches a marker).
- Mixer's roll-vs-pitch arm coefficients are swapped in `Allocation()` (uses `a=1/(4·dx)` for roll where geometry says `b=1/(4·dy)`). Costs a ~9 % asymmetry between roll and pitch torque per unit LMN since `dx/dy ≈ 1.09`. Firmware-source change so deferred until the same fix can ship with hardware testing.
- `quat_to_euler` in `linalg.c` returns `eul.y = -asinf(sinp)` (sign-flipped vs standard). The sim shim now accommodates this explicitly. Cleaner long-term fix is to correct the firmware function and drop the workaround in one coordinated commit; deferred until hardware re-test is possible.

## Decisions

- Sim retune happens at the node level, not by editing firmware-source gains. `fc_sim_node` zeros `pid_rate.ki` and lowers the rate/atti deadband factors after `ControllerInit()`. The firmware build keeps the original 0.04 ki and 0.04 deadband.
- All retunable gains and deadband factors are exposed as **launch arguments** on `sim.launch.py` (and inherited by `hover_demo` / `flight_demo`). Override at run time without rebuilding: `ros2 launch fc_sim hover_demo.launch.py atti_kp_pitch:=0.20 atti_kd_pitch:=0.30`.
- The gz-sim multicopter motor model produces clean equal-thrust hover, but `rotorDragCoefficient` and `rollingMomentCoefficient` are set to zero in the SDF — they introduced asymmetric attitude perturbation under tilt that doesn't match the firmware's IMU model.
- Empirical hover thrust_norm in sim is ~0.500-0.510 when airborne (the firmware's `-4 * 0.6 * g * thrust_norm` mapping intersects gz physics at this point). The `hover_pub.py` PD altitude hold closes around this value.
- **IMU shim per-axis convention** (after the 2026-05-25 sign fix): the mixer in `Allocation()` is mixed-frame — roll and yaw signs are NED, pitch is FLU (M sign flipped). The IMU shim has to match per axis: roll passes through (FLU == NED about x), pitch is FLU on both angle (`-e.y` cancels `quat_to_euler`'s `-asinf`) and rate (`+msg.angular_velocity.y`, no flip), yaw is NED on both (`-e.z` on angle, `-msg.angular_velocity.z` on rate). Confirmed empirically: pitch_sp = +0.1 → drone moves +X (forward in FLU); roll_sp = +0.1 → drone moves -Y world (= right in FLU); yawrate_sp = +0.3 → drone yaws right (qz goes negative in FLU world).
- **Motor index mapping**: SDF `rotor_0` (FR) ← firmware T4, `rotor_1` (BL) ← T2, `rotor_2` (FL) ← T1, `rotor_3` (BR) ← T3. Derived from the firmware mixer's L/M/N sign pattern against the NED body axes.
- Sim FC is a standalone ROS2 node, not a Gazebo system plugin. /clock-driven tick at the world's 2 ms step = 500 Hz exactly matches the firmware's TIM2 ISR cadence. The same node binary will be the eventual hardware adapter once the byte protocol is wired to a real serial port.
- Sanity gate is a safety mechanism for sim only. On real hardware, the IMU never lies about orientation at startup; the gate is harmless there (level_streak reaches threshold within 40 ms of power-on if the drone is sitting upright).

## Done

### Waypoint follower node (2026-05-25)

`scripts/waypoint_demo_pub.py` + `launch/waypoint_demo.launch.py`. Cascaded controller — outer P maps `(target − pos)` into a velocity setpoint, capped at `v_max`; inner P maps `(v_tgt − vel)` into pitch/roll setpoints, capped at `max_tilt`. Altitude held by the same PD pattern as `hover_pub.py`. Walks a list of waypoints in order, advancing when `dist < reach_threshold` AND `speed < stable_speed`, or after `wp_timeout` seconds. Patterns: `hover`, `forward_back`, `box`, `altitude` (defined in the script).

Tuning (after one round of fixing P-only overshoot): `kp_pos=0.70 1/s, v_max=0.5 m/s, kv=0.5 s/m, max_tilt=0.12 rad`. With these:

- `forward_back` (spawn → +3 m x → spawn): WP0 reached in 1.3 s @ dist 0.40, WP1 reached in 5.7 s @ dist 0.40 ending at `(4.60, 4.00, 1.97)`. WP2 (return-to-spawn) gets as close as `0.06 m` but then oscillates ±~2 m around the target — attitude inner-loop's ~80 % tracking + ~0.3 s lag leaves the velocity loop without enough authority to settle exactly on the setpoint without a feedforward term. Acceptable for "drone reaches waypoint" verification; needs feedforward or position-velocity profile to settle cleanly.

Headless-friendly: prints `>> WP N` on advance and a 1 Hz `WP N tgt=… pos=… v=… sp=…` status line, so GUI is not required to verify motion.

### Keyboard teleop node (2026-05-25)

`scripts/teleop_pub.py` + `launch/teleop.launch.py`. Drives `/fc/setpoint` from WASD/QE keys; altitude is held by the same inline PD `hover_pub.py` uses. Each key asserts its setpoint for 0.3 s then auto-decays to zero, so a forgotten finger can't run the drone away. Runs in two terminals on the same `ROS_DOMAIN_ID` (sim launch in A, teleop_pub in B) because cbreak-mode stdin can't be cleanly piped through `ros2 launch`. Useful for eyeballing attitude tracking interactively — `flight_demo` only verifies attitude; positional drift in HOLD phases is real (no position feedback, drag coeffs are zero) and most easily felt on a keyboard.

Key map: W/S pitch ±, A/D roll ±, Q/E yaw ±, R/F target altitude ±0.5 m, space = level + zero rates, X = arm/disarm, Z or Ctrl-C = quit.

### Pitch q.y sign fix + raise sim_retune defaults to firmware-native (2026-05-25)

After the sphere collision change exposed the controller's actual closed-loop behavior, a 0.05 rad pitch step diverged within 1 s — drone flipped past 90° and crashed. Trace through the sign convention showed:

- `quat_to_euler` (`linalg.c:543`) returns `eul.y = -asinf(sinp)` (sign-flipped).
- The IMU shim flipped pitch back (`out.pitch = -e.y`), landing pitch_fw in **FLU** convention (= +FLU_pitch = -NED_pitch).
- The mixer in `Allocation()` is also FLU for pitch (M sign-flipped vs standard NED, like the existing yaw mixer is NED).
- But the body-rate shim flipped `q.y` (`pqr_fw.y = -msg.angular_velocity.y`), putting pqr_fw.y in **NED** convention while pitch_fw was in FLU.

Cascaded loop saw `pqr_des = kp·att_err` (positive when pitch_fw needed to increase) but the measured `pqr_fw.y` had the opposite sign for the same physical rotation → `pqr_err` grew with the rotation → positive-feedback divergence on any non-zero pitch setpoint. Masked previously by the 0.40 m body-collision box wedging the drone before it could actually rotate.

Fix: `fc_sim_node.cpp:191` — remove the `q.y` flip so pqr.y is in FLU and matches pitch_fw. Roll axes had no flips (consistent), yaw shim still flips both (consistent NED). One-line change; comment block above the shim now documents the actual per-axis convention.

With the bug fixed, the firmware-native gains (`rate_kp=0.40, atti_kp=0.80, atti_kd=0.20`) track cleanly. Earlier rounds had halved these to 0.20 / 0.40 / 0.20 under the assumption the gz motor plant was "1.3-1.6 × hotter than the SDF said" — that turned out to be a misdiagnosis of the sign bug + the wedge-corner ground collision. Sim-retune defaults bumped back to firmware-native; rate_kp_r=0.80, rate_kp_p/q=0.40. Old comment claiming gz motor-plant gain mismatch removed.

Regression test: `flight_demo.launch.py phase_duration:=5 pitch_amp:=0.10 roll_amp:=0.10 yaw_amp:=0.3` runs the full 12-phase sequence (HOLD → PITCH_FWD → HOLD → PITCH_BACK → HOLD → ROLL_R → HOLD → ROLL_L → HOLD → YAW_R → YAW_L → HOLD, ~60 s total) without crashing for the first time. Drone tracks each commanded axis correctly, returns to level after each phase, altitude wanders within ±0.4 m of 2.0 m target during maneuvers (±0.01 m at pure hover).

### Sphere body collision: unblocks takeoff (2026-05-25)

`src/world/models/uav26_quad/model.sdf` body_collision changed from a 0.40 × 0.40 × 0.06 m box to a 0.05 m sphere at CoM. The oversized box (sized to the outline of the cross-arms, not the actual body) had four ground corners at (±0.20, ±0.20, ±0.03) that pivoted on contact: any micro-tilt from asymmetric motor spin-up wedged the drone on a single corner, and the thrust line through CoM couldn't break the contact constraint. Sphere has no edges to pivot on; landed drone CoM sits 5 cm above ground, supported on a point contact.

Effect: previously the gain sweep saw identical score 2.9550 (= target altitude RMS = drone never moved) across all 535 sampled cells regardless of `(rate_kp, atti_kp, atti_kd)`. With the sphere, the very first run hit z=1.99 m (target 2.0 m) and held with RMS error 0.01 m and thrust=0.500 dead-stable. ~300× improvement from a one-line SDF edit — confirming the "ground-stuck" Open item was geometric, not a tuning problem.

This also exposed the pitch sign bug above; the box had been masking it by mechanically preventing the drone from actually rotating in response to attitude commands.

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

