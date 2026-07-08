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

### Where to resume (2026-07-08 end — mission verified, incl. on video)

The full competition flow works and is visually verified: r57/r60
(seed 42, ~12 min) sweep the grid serpentine, record all four markers
at their exact GT cells (err 0.00 m), hover 3 s over each with the
ArUco overlay visible, tour them in ID order, home, and park ~0.5 m
from spawn. r60's downward-camera feed is archived as a 10x video via
the new recorder tool. Platform root-cause fixes from the four-agent
audit are logged per topic: [line_tracer](progress/line_tracer.md)
(sweep, velocity loop, velocity-mode landing, FSM guards),
[world](progress/world.md) (4S motors, four-feet contact),
[fc_sim](progress/fc_sim.md) (prime altitude hold, disarm,
single-writer guard), [fc_core](progress/fc_core.md) (900 g thrust
scale), [docker](progress/docker.md) (zombie-FC teardown contract +
Docker Desktop WSLg GUI plumbing — read before running anything).

Daily driver is `scripts/dev.sh`: `gui` (Gazebo window + tracer),
`view` (rqt_image_view on the detection overlay), `mission rNN 900`
(headless + FSM summary), `build`. It wraps the X11 bridge, container
recreation, and the zombie-sweep contract.

#### Next milestones

- [x] M-B: 4-marker sweep + recorded XY (r57/r60; sim-side accuracy
      is snap-exact because DR is truth-injected — hardware DR drift
      is the real M-B risk, revisit with the LIDAR/flow estimator).
- [ ] M-C: retrieval order verification output (40 pt) + per-WP Z
      (20 pt).
- [ ] M-D: mission time (sweep revisits empty rows; ~12 min now).
- [ ] M-E: robustness (multi-frame ID voting, lost-line yaw search;
      the 3 s hover already gives the voting window).
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
3. `scripts/dev.sh gui` (or `mission rNN 900`) — handles container up,
   rebuild-if-needed, X11 bridge, and teardown. `dev.sh view` shows
   the detection overlay. Container recreation drops
   `/workspace/install`; dev.sh rebuilds automatically.
4. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give **162 passing tests** (18 fc_core gtest + 144 line_tracer pytest, post-r57).
5. Analysis tools: `scripts/plot_mission.py <tracer.log>` (trajectory
   PNG + recorded-vs-GT diff; pass `--layout` with the runtime
   aruco_layout.yaml), `src/line_tracer/scripts/record_debug_video.py`
   (record the camera overlay during a run, encode an N-x video on the
   sim timeline). Host scripts under `scripts/` use `uv` via PEP 723
   inline metadata — no installs needed.
