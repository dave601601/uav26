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

### Where to resume (2026-07-09 end — lookahead camera verified, M-D cut 26%)

r64 (seed 42) is the new reference mission: a sideways OV9281+6mm
lookahead camera (boresight +Y, 22 deg depression) lets the
serpentine fly only rows {4, 12}; the side camera votes markers on
the skipped rows onto grid nodes (candidates), the sweep
short-circuits once records+candidates cover all four, and
GOTO_CANDIDATE tours them for the downward-camera record. Search
phase 221 -> 163.5 sim s (-26%), all records err 0.00 m, no fallback,
yaw held at 0.00. Three latent platform defects fell en route (all in
[line_tracer](progress/line_tracer.md) + [world](progress/world.md)):
yawrate_sp NED sign reversal (every prior mission silently flew at
yaw ~ pi), mod-pi yaw-lock blindness to 90/180-deg flips, and marker
textures missing the ArUco quiet zone (fused with grid lines from
oblique views). 192 tests green (183 line_tracer pytest + 18 fc_core
gtest — see line_tracer.md for the new side_camera/candidate suites).

Older baseline (r57/r60 full-serpentine, downward-only) remains
described in [line_tracer](progress/line_tracer.md); operational
contracts unchanged: [docker](progress/docker.md) zombie-FC teardown
+ WSLg GUI plumbing — read before running anything.

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
- [x] M-D: search time — sideways lookahead camera + row-skip sweep
      + candidate short-circuit (r64: search 221 -> 163.5 sim s;
      further cuts would come from cruise speed, not coverage).
- [ ] M-E: robustness. Partly landed via M-D work (per-(id,node)
      multi-frame voting on the side camera; yaw-lock override).
      Remaining: mask ArUco quads before the Hough line detection
      (marker edges hijacked psi_err and spun the drone in r61),
      multi-frame ID voting on the DOWNWARD record path, lost-line
      recovery.
- [ ] Lookahead v2 (deferred by design): IPM rectification for the
      +8 m opportunistic band, full-res (1280x800) sensor if RTF
      allows, hardware bracket at 22 deg + intrinsics/extrinsics
      calibration for the real OV9281.
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
