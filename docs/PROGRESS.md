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

### Where to resume (2026-05-25 r38 — M-A demo done)

Full demo flow works: r38 walks TAKEOFF -> LINE_FOLLOW ->
WAYPOINT_VISIT (marker 2 recorded at the correct (4, 4)) ->
ARRANGE_BY_ID -> RETURN_PATH -> LAND, altitude rock-steady at 2.08 m.
All three r23 leftovers (DR x/y sync, yaw curl, garbage xy frames)
are resolved. Detail in [line_tracer](progress/line_tracer.md).

Known softness in r38: retrieval phases run with loose arrival
distances (5.0 / 3.0 m) and a 30 s RETURN_PATH timeout because there
is no body-velocity feedback yet. The real fix (body-velocity PD from
the /odom_truth derivative) is the entry point for M-B.

#### Next milestones

- [ ] M-B: XY accuracy <0.5 m on recorded markers (30 pt per WP) —
      body-vel PD, then tighten arrival dists, `mission_max_records`
      back to 4, grid sweep for off-axis corners.
- [ ] M-C: retrieval order (40 pt) + per-WP Z (20 pt).
- [ ] M-D: mission time bonus.
- [ ] M-E: robustness (multi-frame ID voting, lost-line yaw search).

#### Operational pitfall

r19-r22 looked like algorithm regressions but were CPU starvation:
prior `docker compose run --rm` left zombie containers alive (inner
`pkill -f 'gz sim'` didn't reap the bash wrapper), and the host hit
load avg 56. After stopping all uav-* containers and pruning, r23
(same code as r22) ran clean. Future runs need `pkill -9 -f 'ros2
launch'` in the inner cleanup, or `docker stop` from the host.

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
5. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **150 passing tests** (18 fc_core gtest + 132 line_tracer pytest, post-r38).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
