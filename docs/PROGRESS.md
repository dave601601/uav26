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

### Where to resume (2026-05-25 end-of-session)

Algorithm side is **landed and unit-tested** (124/124 tests pass, including 28 new closed-loop + edge-case tests). FSM walks all 7 mission phases on a synthetic point-mass drone. **The blocker is sim integration**, not algorithm.

Concrete next-session task: pick ONE of the three sim-integration failure modes below and fix it. M-A demo is gated on this; nothing else is.

#### Sim integration failure modes (observed in r10 / r11 / r12 / r13 / r14)

1. **Drone sticks to ground at takeoff.** With the sphere `body_collision` (`src/world/models/uav26_quad/model.sdf:102`), the drone falls from spawn z=1.5 and lands on the sphere. DartSim's contact constraint absorbs the upward thrust component; even at `thrust_norm=0.62` (clamped max in line_tracer) drone stays at z=0.05. Removing the collision entirely also doesn't help — `/odom_truth.z` still reports 0.05 (the textured_floor's ground_plane has its own collision in `competition.sdf:42-ish`).
   - **Possible fixes**: takeoff burst (force `thrust_norm=0.85` for first 2 s before kp-PD kicks in — pattern in `flight_demo_pub.py`); spawn very high (z=10+) and accept the free-fall; remove ground_plane collision; use `acausal` body model.

2. **DartSim ODE collision detector aborts** on `dDebug`. Seen with firmware-native 0.40 rate / 0.80 atti gains (`src/world/launch/sim.launch.py:166-`). Currently dodged by halving to 0.20 / 0.40. Manifests as `/odom_truth` spewing `(-70000, 7000, -2.6e6)` garbage frames for a few ticks. plot_mission.py has a sanity filter that drops these (`abs(x)>200 or alt<-10`).
   - **Possible fixes**: keep half-gains for sim only (current workaround); use a different physics backend; reduce step size.

3. **Drone yaw drifts at startup**, so cruise_vx ends up moving the drone in -X / -Y instead of +X. Visible in r09 where drone went to (-90, 4) while commanded forward. Probably from sanity-gate releasing at non-identity yaw after a brief ground-contact spin.
   - **Possible fixes**: yaw lock to initial heading during TAKEOFF / LINE_FOLLOW; sanity gate also checks yaw rate (not just tilt).

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
5. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **142 passing tests** (18 fc_core gtest + 124 line_tracer pytest, post-2026-05-25 closed-loop rewrite).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
