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

### Where to resume (2026-07-08 late — full 4-marker mission lands)

r57 flies the complete competition flow (seed 42, 719 s): serpentine
sweep finds all four markers (recorded at exact GT cells, err 0.00 m),
ARRANGE tours them in ID order, drone homes and parks 0.53 m from
spawn. Platform fixes from the four-agent audit are logged per topic:
[line_tracer](progress/line_tracer.md) (sweep, velocity loop,
velocity-mode landing, FSM guards), [world](progress/world.md) (4S
motors, four-feet contact), [fc_sim](progress/fc_sim.md) (prime
altitude hold, disarm, single-writer guard),
[fc_core](progress/fc_core.md) (900 g thrust scale),
[docker](progress/docker.md) (zombie-FC teardown contract — read
before running anything).

Run missions ONLY via `scripts/run_mission.sh`:
`docker compose exec -T uav-aruco bash -s rNN 900 <
scripts/run_mission.sh` (900 s for the full mission).

#### Next milestones

- [x] M-B: 4-marker sweep + recorded XY (r57; sim-side accuracy is
      snap-exact because DR is truth-injected — hardware DR drift is
      the real M-B risk, revisit with the LIDAR/flow estimator).
- [ ] M-C: retrieval order verification output (40 pt) + per-WP Z
      (20 pt).
- [ ] M-D: mission time (sweep revisits empty rows; ~12 min now).
- [ ] M-E: robustness (multi-frame ID voting, lost-line yaw search).
- [ ] Test-reality gap (audit findings): run_mission test driver
      calls the real node methods instead of reimplementing them;
      MockDrone gets attitude lag + drag=0; one launch_testing
      headless-gz smoke tier.

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
5. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **159 passing tests** (18 fc_core gtest + 141 line_tracer pytest, post-r54).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
