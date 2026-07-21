# UAV26 Progress

Index. Each topic file holds the actual work log. Entries go newest-first inside each topic.

## Topics

- [world](progress/world.md) — Gazebo Harmonic sim package (drone model, arena, bridge, launch)
- [line_tracer](progress/line_tracer.md) — vision, dead-reckoning, FSM, planning
- [fc_core](progress/fc_core.md) — STM32 firmware port as a pure C ROS2 package
- [fc_sim](progress/fc_sim.md) — ROS2 node that runs fc_core inside Gazebo, plus hover/flight demos
- [tools](progress/tools.md) — standalone scripts (gain_sweep, etc.)
- [docker](progress/docker.md) — build environment, compose layering, image rebuilds

Planning docs (checklists, not logs):

- [SIM_TO_REAL](SIM_TO_REAL.md) — hardware transition: estimator sources, loop placement, perception deltas, firmware re-tests

## Open

### Bug sweep against real runs (2026-07-21, `fix/intersection-axis-change`)

Eight defects found by flying seed 42 and reading the logs, not by
inspection. The flight one: a turn swapped which Hough family counted
as the crossing and handed the role to the line the drone was parked
on, so any turn taken without a settle counted a node that was never
flown — after a marker confirm the mission believed row y=9 while
re-flying y=6. Fixed in the detector by disarming on a travel-axis
change. Then the launch file's attitude gains, which halved
`fc_sim_node`'s defaults on a premise (ground contact at takeoff) this
world no longer has: paired runs put cross-track RMS at 0.145 m against
0.324 m, so the launch defaults now track the node. The rest were
silent tooling and metadata breaks — `plot_mission.py` parsed nothing
from any current log, `dev.sh` printed an empty final pose, 82 % of a
run log was one repeated line, marker confirms did not name the id that
opened them, and four descriptions documented a retired arena.

Post-fix full mission on this host: INIT -> FINISHED in 490 s, 4/4
records at 0.00 m, ID-order rescue, 0.17 m landing miss, zero gz
aborts. 371 tests green. Details in the topic files; the run logs are
`build/sweep_logs/mission/{full01,gainA,gainB,gainBfull}_*.log`.

Not fixed, needs a decision: a grass phantom opened a marker confirm at
(27.0, 15.1) with no marker within 6 m. The vote rejected it, so the
record path held, but the stop was paid — this is M-E reproducing on
the standard texture.

### Skeleton mission architecture: verified end-to-end (2026-07-17)

`feat/mission-skeleton-interface` (pushed through the r80 log; later
commits local): the mission layer now follows the team skeleton —
MissionManager.step -> McuCommand -> fc_core outer loop (STM32-shared),
node-based navigation with DR snaps, [dx, dy, flag] line contract.
Contract in [MISSION_INTERFACE](MISSION_INTERFACE.md). r83 flew the
FULL mission to FINISHED: 4/4 exact records, ID-order rescue, landing.
343 tests green. Cruise default 1.0 m/s: speed_scale deceleration
scheduling (transits/final legs/marker approaches slow, straights
full) plus front-camera hints (IMX219 45 deg down, hints-only) made
1.0 fly clean end-to-end — search -31 % vs 0.5. Markers now record at
their own projected node (confirm-overshoot-proof). Remaining:
lost-line recovery and the FAILSAFE stubs. Legacy FSM stays selectable
(mission_backend:=legacy). Full history:
[line_tracer](progress/line_tracer.md) r77-r83 entries.

### Comment refactor branch (2026-07-14, merged into the above)

`refactor/human-friendly-comments` (pushed): comments-only pass over
fc_core and line_tracer vision, KNOWN BUG markers at three
firmware-parity traps. Not yet refactored: line_tracer_node,
state_machine, planner, dead_reckoning, grid (same recipe applies).

### Where to resume (2026-07-09 end — visit policy verified, search -39 %)

M-D is met. The candidate visit policy replaced the unconditional
row-end flush with two rules: never tour a candidate that a not-yet-flown
sweep leg passes over, and collect the rest at the transit whose detour
is smallest. Seed 42, search phase: r75 194.5 sim s / 105.9 m against
its own control r76 (policy off) at 320.5 s / 163.7 m, and against the
full-serpentine baseline r72 at 282.5 s. r76 reproduces r70 to 0.5 s, so
polarity does not confound and variance is negligible. Table and the
per-waste breakdown in [line_tracer](progress/line_tracer.md).
The r70-era "candidate-directed search is layout-dependent" conclusion
was a scheduling defect, not a property of the layout.

BLOCKER before those numbers are final. Marker polarity is now a plain
ArUco with the code flush to the 0.4 m sheet — the rules' "(바탕) 검정색,
(마커) 하얀색" describes a standard marker, not a negated one. An
intermediate commit negated it, and that negation incidentally
suppressed the r73 grass-quad phantom by lifting grass above the
detector's threshold. A standard marker forbids negation, so the phantom
hazard is LIVE again: OpenCV builds candidates only from dark quads with
a dark border ring, grass qualifies, and a white grid line clipping a
grass quad is DICT_4X4_50's id 17 exactly.

Next, in order: (1) gate the downward record path — multi-frame vote on
(id, node) plus a marker-size gate at `fx * marker_size / altitude`,
since the AUTHORITATIVE path is the only unguarded one; (2) re-run the
r75/r76 A/B on the correct texture. The r75/r76 numbers above were
measured on the negated texture: the comparison is internally valid, the
absolute times will shift. See [line_tracer](progress/line_tracer.md)
and [world](progress/world.md). 209 tests green.

Both runs used `scripts/dev.sh mission rNN 1150 [params_file:=...]`.
The control file is `build/params_policy_off.yaml`.

### Prior state (2026-07-09 — OFFICIAL SPEC respec verified, r70/r72)

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
`view` (rqt_image_view on the detection overlay), `mission rNN 1200`
(headless + FSM summary), `build`. It wraps the X11 bridge, container
recreation, and the zombie-sweep contract.

#### Next milestones

- [x] M-B: 4-marker sweep + recorded XY (r57/r60; sim-side accuracy
      is snap-exact because DR is truth-injected — hardware DR drift
      is the real M-B risk, revisit with the LIDAR/flow estimator).
- [ ] M-C: retrieval order verification output (40 pt) + per-WP Z
      (20 pt).
- [x] M-D: search time, ON THE LEGACY BACKEND. Lookahead camera,
      row-skip, candidates, short-circuit, and the visit policy
      (coverage filter + cheapest-transit flush). r75 vs r76: search
      -39.3 % against its own control, -31.1 % against the full
      serpentine. None of it is reachable from the default skeleton
      backend: `sweep_row_step` is read only by `state_machine.py`, and
      the lookahead gate tests a legacy FSM the skeleton never ticks, so
      a 2026-07-21 full run logged zero candidates. The skeleton sweeps
      every row and searched in 286 s. Porting the row-skip and the
      visit policy to `ExplorationPlanner` is the open item.
- [ ] M-E: robustness. The downward record path takes a single frame's
      ArUco id with no vote and no geometric check, while the side
      camera's hint path needs 3 votes per node — the authoritative path
      is the unguarded one. Marker polarity removed the r73 phantom that
      exploited this, but the asymmetry stands: add a multi-frame vote
      on (id, node) + a marker-size gate from fx * marker_size /
      altitude. Then: mask ArUco quads before the Hough line detection
      (marker edges hijacked psi_err and spun the drone in r61),
      lost-line recovery. Already landed: per-(id,node) voting on the
      side camera, yaw-lock override.
- [ ] Lookahead v2 (deferred by design): IPM rectification for the
      +6 m opportunistic band (already fired once in r70 at 6.3 m),
      full-res (1280x800) sensor if RTF allows, hardware bracket at
      26 deg + intrinsics/extrinsics calibration for the real OV9281.
- [ ] Test-reality gap (audit findings): run_mission test driver
      calls the real node methods instead of reimplementing them;
      MockDrone gets attitude lag + drag=0; one launch_testing
      headless-gz smoke tier.

#### Deferred firmware items (needs paired hardware re-test)

- [ ] Mixer `Allocation()` swaps `a`=1/(4·dx) and `b`=1/(4·dy) between roll and pitch terms — ~9 % asymmetry.
- [ ] `quat_to_euler` returns `eul.y = -asinf(sinp)` (sign-flipped); sim shim compensates explicitly.
- [ ] `GetAngle2Vec()` (linalg.c) assigns all three results to `res.x`, returning uninitialized y/z. Dead code (no callers in this repo); fix before first use.

## Conventions

- Add new entries to the **top** of the relevant topic file. Each bullet ships in the same commit as the code change it documents.
- Keep this index under ~50 lines. Move detail into a topic file.
- Split a topic file when it crosses ~500 lines.
- Conventional Commits: `feat`, `fix`, `refactor`, `perf`, `docs`, `test`, `chore`. Subject ≤ 72 chars, imperative.
- No `Co-authored-by: Claude` trailer in commit messages (per `CLAUDE.md`).

## Re-entry

1. `git log --oneline -25` — recent commits + their scope.
2. Read this index, then open the topic file(s) listed in **Open**.
3. `scripts/dev.sh gui` (or `mission rNN 1200`) — handles container up,
   rebuild-if-needed, X11 bridge, and teardown. `dev.sh view` shows
   the detection overlay. Container recreation drops
   `/workspace/install`; dev.sh rebuilds automatically.
4. Smoke test: `colcon test --packages-select fc_core line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs` should give 371 passing tests (49 fc_core gtest + 322 line_tracer pytest).
5. Analysis tools: `scripts/plot_mission.py <tracer.log>` (trajectory
   PNG + recorded-vs-GT diff; pass `--layout` with the runtime
   aruco_layout.yaml), `src/line_tracer/scripts/record_debug_video.py`
   (record the camera overlay during a run, encode an N-x video on the
   sim timeline). Host scripts under `scripts/` use `uv` via PEP 723
   inline metadata — no installs needed.
