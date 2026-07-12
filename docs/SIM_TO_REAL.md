# Sim-to-real plan

Forward-looking checklist for moving the stack from Gazebo to the real
airframe (STM32 FC + Jetson companion, D435 downward, OV9281 sideways,
Micolink range/optical-flow module). Each item names the sim shortcut
it replaces. This file is the plan; the analyses behind it live in
`docs/progress/` (2026-07-09 entries).

## Estimator sources — replacing /odom_truth

The sim injects `/odom_truth` into the DR state (x, y, yaw) and the
altitude path. On hardware every channel needs a real source, and the
FC<->companion protocol already carries most of them: `fc_proto_up_t`
streams roll/pitch/yaw, p/q/r, `alt_lidar`, vbatt (`protocol.h`), and
the Micolink frame carries optical-flow vx/vy plus quality flags
(`lidar.h`). In every case the missing piece is the COMPANION-SIDE
CONSUMER, not the sensor or the wire format.

- [ ] Altitude: consume `alt_lidar` telemetry in place of the
      `use_odom_truth_altitude=false` depth-median path. Depth stays as
      a cross-check gate, never the primary: the D435 minimum range
      (~28 cm, spec) sits above both the touchdown cutoff
      (`land_cutoff_alt` 0.12 m) and the takeoff burst gate
      (`takeoff_z_threshold` 0.15 m), so the most safety-critical
      altitudes are unmeasurable by depth; and landing braking is the
      `kd_alt_thrust * vz` term (r48 measured what happens when braking
      falls short), which a differentiated 30 fps depth stream cannot
      feed cleanly. Better still: wire the ported-but-unconsumed ESKFz
      (`filter.c`, 2-state pz/vz, lidar sigma 5 cm + IMU prediction)
      and stream fused pz/vz instead of raw `alt_lidar`.
- [ ] Yaw-lock reference: drive `resolve_locked_yaw_error`'s current
      yaw from FC yaw telemetry, not the open-loop DR integral. The
      0.6 rad override assumes the yaw estimate is true — in sim it is
      (injected). With a drifting estimate the defense inverts: at
      30-60 deg true-vs-estimate divergence neither grid-line family
      classifies (psi_err None) and the lock holds a wrong reference,
      beyond ~60 deg perception captures the wrong mod-90 attractor —
      r61's failure returning through the estimator instead of through
      perception. During sweep legs nothing corrects true yaw at all
      (perception only steers during WAYPOINT_VISIT hover).
- [ ] Body-velocity feedback: consume Micolink flow vx/vy for the
      `kp_vel` velocity loop (sim: /odom_truth xy derivative). Gate on
      `flow_quality`.
- [ ] XY position: still open. Flow integration drifts; marker snaps
      are absolute fixes at intersections but only along the sweep.
      Sim-side record accuracy is snap-exact because DR is
      truth-injected — this is the standing M-B risk. Candidates: flow
      + snap dead reckoning (current plan), or the firmware's eskf6
      (IMU+GPS) if GPS is permitted outdoors.

## Control-loop placement — decided 2026-07-09, revisit on evidence

The altitude loop stays on the Jetson. `Control()` closes attitude and
rate only and discards NED/vel by design (`controller.c:147`); the
whole altitude stack — PD, takeoff burst, descent-rate landing,
touchdown cutoff — lives in `body_vel_to_atti_thr` and was tuned
through the r19..r52 saga. Moving it into fc_core means writing a
firmware altitude controller, implementing the vel-mode that the
protocol reserves but the controller ignores (`vz_sp`,
`controller.h:75` "currently unused"), porting the takeoff/landing
special cases, retuning, and a hardware re-test — on top of two
firmware items already waiting for paired bench time.

The companion-failure case is already covered on the FC: 200 ms of
setpoint silence fails over to a gentle descent at thrust 0.27
(`COMP_STALE_THRUST_NORM`), so a Jetson stall degrades softly rather
than freezing thrust.

Revisit (implement the vz_sp vel-mode and move altitude into fc_core)
only if hardware shows altitude oscillation correlated with vision
load, or the team firmware implements the mode independently.

## Perception deltas

- [ ] Side camera at the real OV9281's full 1280x800. The sim runs
      640x400 purely for RTF; resolution is the biggest single lever on
      usable slant range. Bracket stays 26 deg — the binding constraint
      is the band's inner edge (the adjacent row at 3 m), not detection
      range.
- [ ] Intrinsics + extrinsics calibration for both cameras; the mount
      parameters in params.yaml must mirror measured values, not the
      SDF pose they currently copy.
- [ ] Downward record-path gates: multi-frame (id, node) vote + a size
      gate at `fx * marker_size / altitude`. Tracked as M-E in
      PROGRESS.md (the r73 phantom is reproducible in sim); listed here
      because real grass adds sun/shadow structure the sim never
      sampled, widening the false-quad surface.
- [ ] Satin-ribbon specularity is unmodeled (white albedo only) — sun
      glare off the lines is a hardware-day unknown for both the Hough
      path and the marker quiet zone.

## Firmware deltas — need paired hardware re-test

- [ ] Mixer `Allocation()` swaps a/b between roll and pitch terms
      (~9 % asymmetry) — PROGRESS.md deferred item.
- [ ] `quat_to_euler` pitch sign flip — the sim shim compensates;
      hardware must fix the firmware or replicate the shim on the
      companion.
- [ ] Bench-verify the COMP stale failover (kill the companion process
      mid-hover; expect the 0.27-thrust settle, not a freeze).
