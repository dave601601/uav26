# UAV26 Progress

Index. Each topic file holds the actual work log. Entries go newest-first inside each topic.

## Topics

- [world](progress/world.md) — Gazebo Harmonic sim package (drone model, arena, bridge, launch)
- [line_tracer](progress/line_tracer.md) — vision, dead-reckoning, FSM, planning
- [fc_core](progress/fc_core.md) — STM32 firmware port as a pure C ROS2 package
- [fc_sim](progress/fc_sim.md) — ROS2 node that runs fc_core inside Gazebo
- [docker](progress/docker.md) — build environment, image rebuilds, pinned deps

## Open

- [ ] world: update model.sdf to firmware geometry (mass 1.182 kg, arms dx=0.183 dy=0.168), drop MulticopterVelocityControl, add downward range sensor
- [ ] world: marker_randomize.py + competition.sdf `<include>` reorganization (4 markers, `--seed`)
- [ ] line_tracer: switch the Twist publisher to fc_sim_msgs/Setpoint on `/fc/setpoint` plus an altitude-hold P-controller for thrust_norm
- [ ] Tier B verification: Gazebo hover + pitch step response with the ported controller
- [ ] Tier C verification: line_tracer end-to-end smoke (TAKEOFF -> LINE_FOLLOW reaches a marker)

## Conventions

- Add new entries to the **top** of the relevant topic file. Each bullet ships in the same commit as the code change it documents.
- Keep this index under ~50 lines. Move detail into a topic file.
- Split a topic file when it crosses ~500 lines.

## Re-entry

1. `git log --oneline -15` for the recent commit shape.
2. Read this index, then open the topic file(s) listed under Open.
3. `docker compose up -d uav-aruco` (image: `uav-aruco:latest` from the repo Dockerfile).
4. `docker exec uav-aruco bash -lc "cd /workspace && colcon build --packages-select fc_core fc_sim_msgs fc_sim line_tracer world && colcon test --packages-select fc_core line_tracer"` for a sanity pass before resuming new work.
