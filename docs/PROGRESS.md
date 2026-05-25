# UAV26 Progress

Index. Each topic file holds the actual work log. Entries go newest-first inside each topic.

## Topics

- [world](progress/world.md) — Gazebo Harmonic sim package (drone model, arena, bridge, launch)
- [line_tracer](progress/line_tracer.md) — vision, dead-reckoning, FSM, planning
- [fc_core](progress/fc_core.md) — STM32 firmware port as a pure C ROS2 package
- [fc_sim](progress/fc_sim.md) — ROS2 node that runs fc_core inside Gazebo, plus hover/flight demos
- [tools](progress/tools.md) — standalone scripts (gain_sweep, etc.)
- [docker](progress/docker.md) — build environment, compose layering, image rebuilds

## Open

### Where to resume (2026-05-25 r15 fix landed)

Drone now takes off in the sim. r15 shows TAKEOFF -> LINE_FOLLOW ->
WAYPOINT_VISIT firing, alt 2.26 m reached, one marker recorded
(coords wrong). The two pre-r15 failure modes (ground stick + odom
garbage cascade) are fixed; two smaller ones remain before the
end-to-end demo lands.

#### Remaining sim integration issues (observed in r15)

1. **Yaw drift at takeoff** (~20°). cruise_vx body +X becomes world
   (+X, -Y); drone flies off-grid to (148, -57) m. Until the firmware
   mixer / `quat_to_euler` sign bugs (deferred for hardware re-pair)
   are fixed, the algorithm side needs an active yaw-lock to initial
   heading during TAKEOFF / LINE_FOLLOW / WAYPOINT_VISIT.
   - **Where**: `state_machine.py` Behavior — add `lock_yaw_to_initial`
     flag; `line_tracer_node._on_dr_tick` sets `psi_err =
     -dr_state.yaw` under that flag.
2. **Altitude droops back to ground after takeoff overshoot.** Drone
   peaks at 2.26 m (above 2.0 target due to burst momentum), then PD
   undershoots and the drone smoothly descends to 0.05 m over ~5 s.
   - **Where**: `dead_reckoning.body_vel_to_atti_thr` — raise
     kp_alt_thrust, smooth burst release (e.g. ramp to hover_thrust as
     vz_truth becomes positive instead of a hard switch), or condition
     burst release on alt > 0.5 m AND vz > 0.

#### Fixed in r15

| Symptom | Fix |
|---|---|
| Ground stick at alt=0.05 m | Takeoff burst in `body_vel_to_atti_thr` (alt<0.30 + |vz|<0.2 + err>0.5 -> thrust=0.85); mirrors `hover_pub.py:86` |
| Odom garbage frames poison `kd_alt_thrust * vz_truth` | `_on_odom_truth` rejects frames with |z|>50 m or |vz|>30 m/s |

#### After M-A demo unblocks

- [ ] M-B: XY accuracy <0.5 m on recorded markers (this is 30 pt per WP).
- [ ] M-C: retrieval order (40 pt) + per-WP Z (20 pt).
- [ ] M-D: mission time bonus.
- [ ] M-E: robustness (multi-frame ID voting, lost-line yaw search).

#### Deferred firmware items (needs paired hardware re-test)

- [ ] Mixer `Allocation()` swaps `a`=1/(4·dx) and `b`=1/(4·dy) between roll and pitch terms — ~9 % asymmetry.
- [ ] `quat_to_euler` returns `eul.y = -asinf(sinp)` (sign-flipped); sim shim compensates explicitly.

## Conventions

- Add new entries to the **top** of the relevant topic file. Each bullet ships in the same commit as the code change it documents.
- Keep this index under ~50 lines. Move detail into a topic file.
- Split a topic file when it crosses ~500 lines.
- Conventional Commits: `feat`, `fix`, `refactor`, `perf`, `docs`, `test`, `chore`. Subject ≤ 72 chars, imperative.
- No `Co-authored-by: Claude` trailer in commit messages (per `CLAUDE.md`).

## Re-entry

1. `git log --oneline -25` — recent commits + their scope.
2. Read this index, then open the topic file(s) listed in **Open**.
3. Spin up the dev env: `docker compose up -d uav-aruco`. The container auto-runs `hover_demo` because of `compose.override.yml` — to drop to a shell instead: `docker compose run --rm uav-aruco bash`.
4. First-run / fresh-install bootstrap:
   ```
   docker compose run --rm uav-aruco bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --packages-ignore realsense2_camera realsense2_camera_msgs"
   ```
5. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **146 passing tests** (18 fc_core gtest + 128 line_tracer pytest, post-r15 takeoff burst + odom gate).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
