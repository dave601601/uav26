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

### Where to resume (2026-05-25 r23 happy path landed)

Drone takes off, holds 2.08 m, flies +X, hits marker 2, records, keeps
cruising. r23_tracer.png shows the first clean end-to-end run. Three
small follow-ups remain before the demo is visibly clean.

#### Remaining (not blocking for the visual demo)

1. RECORD coordinate is off by one cell (recorded (4, 0), GT (4, 4)).
   DR.x/y isn't synced from /odom_truth pose — only DR.yaw is. Extend
   the sim-only inject in `line_tracer_node._on_dr_tick` to also write
   `self._dr.state.x = msg.pose.pose.position.x` (and y).
2. Long-horizon trajectory curls ~200 m off-axis over 60 s. Yaw lock
   has a steady-state error against the firmware's residual drift.
   Bump `kp_yaw` or add an integral term to `compute_body_velocity`.
3. Garbage xy frames (|x|~4700 m) sneak past the existing |z|>50 /
   |vz|>30 filter in `_on_odom_truth`. Add an xy magnitude check.

#### Fixed in r23 bundle

| Symptom | Fix |
|---|---|
| Ground stick at alt=0.05 m | Takeoff burst (alt<0.30 + vz<0.2 + err>0.5 -> thrust=0.85); mirrors `hover_pub.py:86` |
| Odom garbage frames poison vz | `_on_odom_truth` rejects |z|>50; vz is finite-differenced from altitude (world frame) |
| Cruise heading drifts off-axis | Behavior.lock_yaw_to_initial + /odom_truth orientation injected into DR.yaw. TAKEOFF intentionally skips the lock to avoid wz-saturation against sphere ground contact. |
| Drone hits ground before sanity gate releases | Spawn z 1.5 -> 3.0 m |

#### Operational pitfall

r19-r22 looked like algorithm regressions but were CPU starvation:
prior `docker compose run --rm` left zombie containers alive (inner
`pkill -f 'gz sim'` didn't reap the bash wrapper), and the host hit
load avg 56. After stopping all uav-* containers and pruning, r23
(same code as r22) ran clean. Future runs need `pkill -9 -f 'ros2
launch'` in the inner cleanup, or `docker stop` from the host.

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
5. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **149 passing tests** (18 fc_core gtest + 131 line_tracer pytest, post-r23 with yaw lock + derivative vz + odom yaw inject + burst-on-fall).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
