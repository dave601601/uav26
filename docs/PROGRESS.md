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

- [ ] **DartSim ground-stuck (UNSOLVED).** Drone refuses to lift even at thrust_norm=0.75 sustained 20+ s if it touches the ground first. Mitigated by airborne spawn + sanity gate but not reliable. See [fc_sim](progress/fc_sim.md) Open for proposed fixes (lower friction, sphere collision, no body collision).
- [ ] **Bake in overnight gain sweep results.** A `scripts/gain_sweep.py --grid overnight` run is searching for better `(rate_kp, atti_kp, atti_kd)` defaults than the current `(0.20, 0.40, 0.20)`. When the CSV is in, update `fc_sim_node`'s sim-retune defaults.
- [ ] Tier C: line_tracer end-to-end smoke (TAKEOFF → LINE_FOLLOW reaches a marker). Blocked on reliable takeoff.
- [ ] Step-response damping. A 0.1 rad attitude step overshoots ~3-5×. Acceptable for line_tracer (continuous setpoints) but the sweep should find a better trade-off.
- [ ] Update `line_tracer`'s default `hover_thrust_norm` from 0.49 to 0.50 once the sim hover point is finalized.

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
5. Smoke test: `colcon test --packages-select fc_core line_tracer` should give 114 passing tests (18 fc_core gtest + 96 line_tracer pytest).
6. Standalone scripts live under `scripts/` and use `uv` via PEP 723 inline metadata (`#!/usr/bin/env -S uv run --script`). No pip/apt install needed — invoke directly.
