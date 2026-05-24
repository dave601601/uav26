# UAV26 Progress

Index. Each topic file holds the actual work log. Entries go newest-first inside each topic.

## Topics

- [world](progress/world.md) — Gazebo Harmonic sim package (drone model, arena, bridge, launch)
- [line_tracer](progress/line_tracer.md) — vision, dead-reckoning, FSM, planning
- [fc_core](progress/fc_core.md) — STM32 firmware port as a pure C ROS2 package
- [fc_sim](progress/fc_sim.md) — ROS2 node that runs fc_core inside Gazebo
- [docker](progress/docker.md) — build environment, image rebuilds, pinned deps

## Open

- [ ] Tier B: retune controller gains against Gazebo dynamics. Current sim launches and the control loop runs end-to-end, but hover is unstable (drone flips within ~25 s of commanding a flat setpoint). Probable cause is rate-PID integral wind-up under IMU noise; first attempt should set ki=0 on the rate axes and revisit. See [fc_sim](progress/fc_sim.md) Open for details.
- [ ] Tier C: line_tracer end-to-end (TAKEOFF -> LINE_FOLLOW reaches a marker). Blocked on Tier B hover stability.
- [ ] Empirical hover thrust calibration. At thrust_norm=0.49 (theoretical hover) the drone is glued to the ground; need to find the lift-off thrust empirically and update line_tracer's `hover_thrust_norm` parameter accordingly.

## Conventions

- Add new entries to the **top** of the relevant topic file. Each bullet ships in the same commit as the code change it documents.
- Keep this index under ~50 lines. Move detail into a topic file.
- Split a topic file when it crosses ~500 lines.

## Re-entry

1. `git log --oneline -15` for the recent commit shape.
2. Read this index, then open the topic file(s) listed under Open.
3. `docker compose up -d uav-aruco` (image: `uav-aruco:latest` from the repo Dockerfile).
4. `docker exec uav-aruco bash -lc "cd /workspace && colcon build --packages-select fc_core fc_sim_msgs fc_sim line_tracer world && colcon test --packages-select fc_core line_tracer"` for a sanity pass before resuming new work.
