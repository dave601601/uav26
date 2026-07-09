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

### Where to resume (2026-07-09 end — OFFICIAL SPEC respec verified, r70/r72)

The sim now models the official 2026-07 survey: outdoor GRASS field,
3 m cells, WHITE 10 cm satin lines, 0.4 m marker sheets on interior
vertices, 4 unique IDs from 0..49. Two parameterized ASSUMPTIONS to
confirm against the rules: arena 30x21 m (grid_width/depth) and
DICT_4X4_50 (aruco_dict + world aruco.py --dict; "0..49" exactly
matches the 50-marker dictionaries). Side camera mount 22 -> 26 deg
(3 m adjacent row sat on the old band edge). r70 verified the whole
candidate pipeline on the new spec (all 4 records exact, GOTO chain,
bonus far-band candidate at 6.3 m range); r72 is the full-serpentine
baseline. Honest A/B for seed 42: baseline WON (283.5 vs 321.0 sim s
search) because this layout's markers lie on the serpentine's natural
path while the lookahead paid a doubled-back candidate tour — see
[line_tracer](progress/line_tracer.md) for the analysis; the r64-era
26% win and the closed-loop corner-layout A/B show the opposite sign.
Candidate-directed search is LAYOUT-DEPENDENT; visit-policy
optimization is the new M-D item. 192 tests green.

The 2026-07-08/09 platform root-cause fixes remain load-bearing:
yawrate_sp NED sign reversal (every earlier mission silently flew at
yaw ~ pi), mod-pi yaw-lock blindness, ArUco quiet zone on marker
textures. Operational contracts unchanged:
[docker](progress/docker.md) zombie-FC teardown + WSLg GUI plumbing —
read before running anything. Host RTF degrades over WSL uptime
(0.45 -> 0.13 observed); all timing metrics here are SIM seconds.

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
- [ ] M-D: search time. Mechanism landed and verified (lookahead
      camera, row-skip, candidates, short-circuit; r64 cut 26% on the
      4 m grid) but on the official 3 m grid the benefit is
      LAYOUT-DEPENDENT (r70 vs r72: baseline won seed 42 by ~35 s).
      Next: candidate visit-policy optimization — cheapest-detour
      insertion into the remaining sweep instead of the row-end flush
      tour; skip the tour when a future sweep leg's downward corridor
      covers the candidate.
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
