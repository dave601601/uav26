# UAV26 Progress

Index. Each topic file holds the actual work log. Entries go newest-first inside each topic.

## Topics

- [world](progress/world.md) — Gazebo Harmonic sim package (drone model, arena, bridge, launch)
- [line_tracer](progress/line_tracer.md) — vision, dead-reckoning, FSM, planning
- [fc_core](progress/fc_core.md) — STM32 firmware port as a pure C ROS2 package
- [fc_sim](progress/fc_sim.md) — ROS2 node that runs fc_core inside Gazebo
- [docker](progress/docker.md) — build environment, image rebuilds, pinned deps

## Open

- [ ] Tier C: line_tracer end-to-end smoke (TAKEOFF -> LINE_FOLLOW reaches a marker). Hover stability landed in `56fe60c`; the integration test is the next step.
- [ ] Tighten attitude step-response damping. Hover holds cleanly but a 0.1 rad step overshoots 3-5x. Acceptable for line_tracer (continuous setpoints) but worth tuning further. See [fc_sim](progress/fc_sim.md) Open.
- [ ] Update line_tracer's `hover_thrust_norm` default from 0.49 to ~0.50 to match the empirical sim hover point.

## Conventions

- Add new entries to the **top** of the relevant topic file. Each bullet ships in the same commit as the code change it documents.
- Keep this index under ~50 lines. Move detail into a topic file.
- Split a topic file when it crosses ~500 lines.

## Re-entry

1. `git log --oneline -15` for the recent commit shape.
2. Read this index, then open the topic file(s) listed under Open.
3. `docker compose up -d uav-aruco` (image: `uav-aruco:latest` from the repo Dockerfile).
4. `docker exec uav-aruco bash -lc "cd /workspace && colcon build --packages-select fc_core fc_sim_msgs fc_sim line_tracer world && colcon test --packages-select fc_core line_tracer"` for a sanity pass before resuming new work.
