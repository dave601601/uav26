# line_tracer

Vision-driven companion: downward camera -> Hough line + ArUco -> dead reckoning + FSM -> setpoint to FC.

## Intersection pulse detector (2026-07-14)

New IntersectionDetector in perception.py (additive): fires exactly one
pulse per physical grid crossing via an enter/exit hysteresis band on
the crossing line's center offset (40/90 px at 640x400, ~2 m alt), and
reports forward/left/right/backward branch flags from the Hough
segments at the firing frame. Re-arms only when the crossing is SEEN
beyond the exit band, so a one-frame Hough miss cannot double-count.
Branch flags are labeled for positive-axis travel; the mission layer
owns the X_NEG/Y_NEG flip. 30 synthetic-image tests; suite at 232.

## Node-based mission core landed (2026-07-14)

line_tracer/mission.py: pure-Python (stdlib-only) mission layer per
docs/MISSION_INTERFACE.md — fixed-value enums, metric dataclasses,
preallocated 11x8 GridMap with node_world/nearest_node, self-contained
BFS PathPlanner, boustrophedon ExplorationPlanner that never leaves the
grid and re-sweeps on exhaustion, and MissionManager with injected
now/logger, DR snap at grid entry and at marker confirm (off-by-one fix
+ counting-drift re-zero), and node+meters+DR in every state log line.
Skeleton names preserved for team diffability. 20 new tests; suite at
252. Deviations from the sketch are listed in the spec section 10 plus:
initial direction is planner-chosen at grid entry (an edge snap would
otherwise walk out of bounds on the first move).

## Mission skeleton architecture adopted (2026-07-14)

Team skeleton (docs/mission_skeleton.py) becomes the target interface:
MissionManager.step(SensorData, PerceptionData) -> McuCommand, with the
MCU (fc_core/STM32) running the outer control loops on raw vision
errors. User decisions: node-based (intersection-counting) navigation
is the main algorithm with world meters logged alongside; fc_core gets
the outer loop (real STM32 is the deployment target); intersection
detection gets a real implementation; the side/lookahead camera is
deferred. Grid confirmed 30 x 21 m (11 x 8 nodes), not the skeleton's
24 x 15. Full contract in docs/MISSION_INTERFACE.md, including the
two DR snap points (grid entry, marker confirm) that re-zero counting
drift, and the deviations list. Legacy Setpoint path stays behind a
node parameter for A/B.

## Vision comments rewritten for outside readers (2026-07-14)

Comments-only pass over perception.py and side_camera.py (geom.py was
already clean); the docstring-stripped AST is identical before and
after, and the full test suite is green. The aruco_white_on_black
comment now states the constraints without the run-history: standard
marker so no negation; grass patches bounded by white grid lines can
decode as an exact codeword; the mitigation belongs in the record path
(multi-frame vote + size gate). The Korean rule quote is translated.
The use_ipm comment's "+4 m band" was stale pre-respec wording and now
reads "adjacent-row band (+3 m on the official grid)". Noticed but not
applied (code change, out of scope for this pass): `detect_aruco` uses
default `DetectorParameters()` while the side camera tunes its own — a
shared detector-params helper would keep the two paths consistent.
Follow-up on review feedback: mid-code comments are capped at 2-3
lines; the grass false-positive hazard moved from the field comment
into the perception.py module docstring.

## Lookahead overlay publishes in every FSM state (2026-07-09)

`/line_tracer/lookahead_debug_image` already existed, but `_on_lookahead`
returned before publishing whenever the FSM sat outside the three search
states (or before camera_info / altitude / grid arrived). The window was
therefore black through the whole of TAKEOFF — exactly when you open it
— and through the entire retrieval tour.

Detection and publishing are now separate. Detection plus voting stays
gated to the search states: it is the expensive half, candidates cannot
change the mission once retrieval starts, and TAKEOFF/LAND attitudes are
outside the projection's small-angle comfort zone. The overlay is drawn
and published every frame regardless, annotated `detection paused
(<reason>)` in amber when the detector did not run. Without that note an
empty frame would read as "the side camera saw nothing", which is a
different claim from "nothing looked".

Verified live: the FIRST lookahead frame a subscriber receives carries
2221 amber pixels, i.e. it is a paused frame that the old code would
have dropped. Rates measured against a running sim: downward 15.5 Hz,
lookahead 6.2 Hz (10 sim-Hz sensor at RTF ~0.6).

`dev.sh gui` now brings up Gazebo, the tracer, and both overlay windows;
`dev.sh view [down|side]` picks one. rqt binds its topic argument
against what is advertised at start, so the viewers come up 6 s after
the tracer. Both were smoke-tested offscreen against a live sim.

## Current state (2026-07-09 — visit policy verified, search -39 % against its own control)

The candidate visit policy (M-D) is landed and measured. Two rules
replace the unconditional row-end flush, both in `state_machine.py`,
both switchable so the old scheduling stays reproducible
(`candidate_coverage_radius: 0.0` + `defer_flush_to_cheapest: false`):

- Coverage. A candidate within `candidate_coverage_radius` of a sweep
  leg the drone has not flown yet is never toured — the downward camera
  records it for free when that leg runs. r70/r76 flew a dedicated tour
  to id17 at (21, 15) and then swept the row-15 leg straight over it.
- Cheapest insertion. A tour can only splice into a transit (splicing
  into a leg truncates that leg's coverage, and with row-skip nothing
  else ever looks at that ground), and the row-end flush points are
  exactly the transits. So flush here only if no later transit collects
  the same set for a smaller detour. The row-6 pair costs a 42.7 m
  detour at the row-3 east end and 10.0 m at the row-9 -> row-15
  transit; r70/r76 paid the former, r75 waits and pays the latter.

Deferral cannot strand a candidate. The last flush point has no future
transit to compare against, so it always fires; `_on_sweep_exhausted`
collects anything still outstanding; the short-circuit is never delayed
because candidates count toward `max_records` exactly as records do; and
a wrong candidate is still dropped after `candidate_wait_seconds`.
Coverage applies only at flush points — short-circuit and exhaustion
abandon the sweep, so "a future leg" does not exist for them.

### A/B, seed 42, search phase (LINE_FOLLOW -> ARRANGE_BY_ID)

r76 is r75 with only the two policy knobs off, so the comparison is
clean. r70 is the same scheduling as r76 on a different texture — they
agree to 0.5 sim s and 0.4 m across the whole search, which both shows
texture polarity barely moves the timing and pins run-to-run variance
far below the effect.

| run | visit policy | texture | search | search track | records |
|---|---|---|---|---|---|
| r70 | eager flush | white sheet, standard code | 321.0 s | 164.1 m | 4/4 exact |
| r76 | eager flush | black sheet, negated code | 320.5 s | 163.7 m | 4/4 exact |
| r72 | none (full 6-row serpentine) | white sheet, standard code | 282.5 s | 148.1 m | 4/4 exact |
| r75 | coverage + cheapest | black sheet, negated code | **194.5 s** | **105.9 m** | 4/4 exact |

Neither texture is the rules-correct one; see the caveat below.

Against its own control the policy cuts search 126.0 s (-39.3 %) and
57.8 m (-35.3 %); total-to-LAND 493.5 -> 370.0 s. Against the
full-serpentine baseline that previously beat the lookahead, it cuts
88.0 s (-31.1 %). Retrieval track is 92.8 / 95.9 m — unchanged, as
expected: the policy only touches the search.

Both wastes show up as measured time. r76 reaches the row-9 west end at
t=200.5, r75 at t=107.5 — the 93 s row-3 double-back. Then r76 spends
t=199..237 touring id17 and returning, ground r75 covers anyway: it
records id17 in passing at t=177 with no detour at all.

Layout-dependence is gone in the sense that mattered. The r70 finding
("candidate-directed search is layout-dependent, the baseline won seed
42") was really a scheduling defect, not a property of the layout.

Both runs: 0 gz aborts, 0 candidate drops, all records at exact cells,
landing 0.41 / 0.35 m from spawn.

CAVEAT, pending a re-run. r75 and r76 both flew the NEGATED texture (the
misread of the rules), whose detector inversion incidentally suppressed
the grass-quad phantom. The comparison between them is internally valid
— same textures, one variable — but the absolute times will not
reproduce on the rules-correct texture, whose 33 % coarser modules
promote candidates earlier. Re-run once the record path is gated;
running before that risks another silently corrupted record.

### r73 and the phantom that blocked this measurement

r73 ran this policy on a white sheet carrying a standard code. Its
border ring was black, so the detector saw the same polarity it sees on
today's texture — this failure is NOT historical.

Both rules executed exactly as designed: no flush at the row-3 east end,
tour at the row-9 west end, id17 left to the sweep. But at t=156 the
downward camera decoded a phantom id=17 at the bare grid crossing
(9, 15), 12 m from the real marker, and the FSM recorded it. That single
frame poisoned the record (12.00 m error), blinded the drone to the real
id17 when it overflew (21, 15) at t~184 (the id was already in
`records`), and left the sweep exhausted at 3 records — which triggered
the one-shot fallback sweep over rows 6/12/18. The fallback recovered
the mission (4 records, landed) but cost ~115 s, so r73's search read
321.0 s against a ~205 s trajectory.

r75 avoided it only because it flew the negated texture, whose detector
inversion made grass too bright to form a candidate quad. That texture
was a misreading of the rules and is gone. Nothing in the corrected
world prevents r73 from happening again.

### The phantom id=17 — the hazard is structural and still open

Root cause, not bad luck.

OpenCV builds ArUco candidate quads out of DARK regions only (it
adaptive-thresholds with `THRESH_BINARY_INV` and contours the result),
then rejects any candidate whose border ring is not dark. On a green
grass field, GRASS SATISFIES BOTH. A patch of grass bounded by white
grid lines is a dark quad with a dark border — a legal candidate. The
detector then samples its interior on a 4x4 grid.

In DICT_4X4_50, id 17's codeword is a white 2x2 block against one edge
and black everywhere else:

```
0 1 1 0
0 1 1 0
0 0 0 0
0 0 0 0
```

which is exactly what a white grid line clipping the edge of a dark quad
produces. And the match had to be exact: OpenCV allows
`int(maxCorrectionBits * errorCorrectionRate)` = `int(1 * 0.6)` = 0
corrected bits for this dictionary (verified — a 1-bit-flipped id 17
renders undetectable at both 0.6 and 0.8; only errorCorrectionRate 1.0
recovers it). An all-black quad decodes to nothing. It takes the white
band to become a marker.

A correction to what this file said earlier today. Negating the
grayscale before detection does make grass bright, which removes the
whole candidate class — but negation is only legal for a marker whose
border ring is white. The rules describe a STANDARD ArUco, so the world
now carries one (see [world](world.md)) and negation is off. THE HAZARD
IS THEREFORE LIVE. r73 ran on a standard-polarity texture and recorded a
phantom id 17 twelve metres from the real marker; nothing about the
texture fix prevents that recurring.

The mitigation must live in the record path, and the asymmetry there is
the real defect: the side camera's HINT path is gated by 3 votes on one
intersection (`CandidateTracker`), while the downward camera's
AUTHORITATIVE record path takes the first frame's id and snaps it to the
nearest intersection with no vote and no geometric check. Two cheap,
independent gates, neither yet implemented:

- Multi-frame vote on (id, node), mirroring `CandidateTracker`. A
  single-frame fluke never reaches `records`. WAYPOINT_VISIT already
  hovers 3 s, so the latency is free.
- Marker-size gate. A real marker's apparent side is
  `fx * marker_size / altitude` — 93 px at 2 m for the 0.4 m sheet. A
  quad formed from grass and grid lines has no reason to match it.

Random-pattern false-accept ceilings, for the flagged `aruco_dict`
assumption (the rules give IDs 0..49 but name no dictionary). `eff_t` is
the correctable bits actually used at errorCorrectionRate 0.6, not the
dictionary's nominal `maxCorrectionBits`:

| dict | modules | maxCorrectionBits | eff_t | false-accept |
|---|---|---|---|---|
| DICT_4X4_50 | 4x4 | 1 | 0 | 0.305 % |
| DICT_5X5_50 | 5x5 | 3 | 1 | 0.016 % |
| DICT_6X6_50 | 6x6 | 6 | 3 | 0.002 % |

The phantom was not drawn from that random distribution — it was a
structured, exact match — so a bigger dictionary mitigates rather than
prevents. It also costs lookahead range. Close the record path first: it
is dictionary-independent.

## Superseded state (2026-07-09 — official-spec respec, r70/r72 eager-flush A/B)

The stack now models the official 2026-07 survey spec end to end:
3 m grass grid with white satin lines, 0.4 m marker sheets, 4 unique
IDs from 0..49 on interior vertices. Two assumptions are parameterized
and flagged until the rules confirm them: arena 30x21 m
(grid_width/depth) and DICT_4X4_50 (aruco_dict — "IDs 0..49" exactly
matches the 50-marker dictionaries). Side camera mount deepened 22 ->
26 deg: on 3 m cells the adjacent row (depression 33.3 deg) sat
exactly ON the old band edge; the new band [2.64, 7.56] m gives it a
4.0 deg margin and keeps the +6 m row as a bonus. 192 tests green.

r70 (lookahead, seed 42: id17@(21,15), id15@(6,6), id14@(3,6),
id8@(24,18)) verified the candidate pipeline on the new spec first
try: both row-6 markers promoted at range 3.6 (the design point),
id17 promoted from the 6 m FAR band (range 6.3 — better than the
2.5 px/module estimate, thanks to 4X4's larger modules), id8
triggered the short-circuit, all four recorded at exact cells via
the GOTO chain, landing 0.39 m from spawn.

Honest A/B (sim seconds, search / total to LAND, same seed):

| run | search | total | notes |
|---|---|---|---|
| r70 lookahead (rows 3,9,15) | 321.0 | 495.5 | 4 GOTO visits |
| r72 baseline (all 6 rows) | 283.5 | 461.0 | markers en-route |

The baseline WON by ~35 s on this layout: seed 42's markers happen to
lie on the serpentine's natural path (row-6 pair near the west end of
its westward leg; id8 4 m into the row-18 leg), while r70 paid ~50 m
of doubled-back travel — the row-3 flush tour runs from the row's
east end back to the west-end candidates and then returns east to the
transit waypoint. The 4 m-grid r64 result (search -26%) and the
closed-loop corner-layout A/B (one marker per skipped row -> flush +
short-circuit both fire, >20 s margin) show the opposite sign. Net:
candidate-directed search is LAYOUT-DEPENDENT on the 3 m grid; wins
come from skipping empty rows and early short-circuit, losses from
tour double-backs when candidates sit behind the sweep direction.

Follow-up (new M-D item): candidate visit-policy optimization —
insert candidate nodes into the remaining sweep route (cheapest
detour, TSP-style) instead of the unconditional row-end flush tour,
and skip the tour entirely when the candidate lies within the
downward camera's corridor of a future sweep leg. The mechanism
(tracker, dedup, GOTO, fallback) is verified and unchanged.

## Superseded state (2026-07-09 — r64 on the pre-respec 4 m grid)

r64 (seed 42) is the first mission where the sideways lookahead
camera drives the search end to end: row-4 sweep records id2/id0 with
the downward camera, the row-12 leg promotes candidates for both
row-16 markers from 4 m away (side camera, vote threshold 3), the
short-circuit abandons the sweep the moment records+candidates cover
all four, GOTO_CANDIDATE tours the two candidates, and the downward
camera records them at their exact GT cells. Rows 8 and 16 are never
flown; row 12 is abandoned at x~4. No fallback, no gz aborts, yaw
held at 0.00 the whole flight, all four records err 0.00 m.

Search-phase A/B (sim seconds, measured by counting the 0.5 s
throttled status lines between the first LINE_FOLLOW and
ARRANGE_BY_ID):

| run | search | total to LAND | notes |
|---|---|---|---|
| r60 | 221.0 | 486.5 | full 4-row serpentine (pre-lookahead baseline) |
| r61 | 235.0 | 500.5 | camera blind (yaw flipped 180) -> skipped-row fallback |
| r64 | 163.5 | 389.0 | candidate-directed — search -26%, total -20% |

(r64 total also gains on the tour: the search ends on top of the
last candidate, which is already a tour node.) Landing 0.32 m from
spawn; max |yaw| over the whole flight 0.000 rad — the yaw actuation
sign fix plus lock override hold the heading exactly where every
prior run silently wandered to 180 deg.

The saving generalizes better than seed 42 shows: this layout puts
markers only on the first/last rows (near worst case — the candidate
tour re-flies row 16 that the serpentine would have swept anyway).
Layouts with markers on middle rows skip more of the arena.

r64 event timeline: RECORD id2 t=5.5, id0 t=46.0 (row 4);
CANDIDATE id3 t=75.5, id1 t=113.0 (row-12 leg, range 5.4 m);
short-circuit + GOTO same tick as the second candidate; RECORD id1
t=121.5, id3 t=161.5; ARRANGE t=164.5.

Three platform defects had to fall to get here — each invisible to
the position-controlled downward-only missions and exposed by making
yaw direction and oblique rendering load-bearing (entries below):
yawrate NED sign reversal, mod-pi yaw-lock blindness, and the
missing ArUco quiet zone on the marker textures.

## Superseded (2026-07-08 — lookahead build-out logs, pre-respec 4 m grid)

### Yaw actuation sign was REVERSED end-to-end (r62 -> measured fix)

r62 (first run with the mod-pi lock override) still flew flipped and
still saturated wz all flight — the override math was right but could
not converge because the PLANT sign is inverted: a direct experiment
(hover the sim, command yawrate_sp=+0.3 for 12 s) turned the drone
-1.68 rad, i.e. +command = CW. The Setpoint contract is the
firmware's NED body frame (+yawrate = yaw right; flight_demo_pub.py
documents it, fc_sim passes it through, fc_core treats yaw as NED
throughout per the fc_sim IMU-path comments) — but line_tracer fed
its REP-103 FLU wz (+CCW) in unconverted. Under the reversed sign
the yaw lock's fixed point at 0 is a REPELLER and the line-aligned
orientations at 90/180 deg are attractors: every long run drifted to
yaw ~ pi and stayed (r60/r61 full-flight wz saturation), invisible
until the side camera made yaw direction load-bearing. The r15-era
"kp_yaw too small to fully cancel drift" observation was this bug.

Fix: `body_vel_to_atti_thr` emits `yawrate_sp = -vel.wz` (the same
place the pitch/roll sign bookkeeping lives); MockDrone now models
the NED plant (`yaw -= yawrate_sp*dt`) so the closed-loop tests
exercise the true double negation; sign pinned by a dedicated test.
The 1 Hz status log now prints yaw. Roll/pitch were already
empirically shimmed (2026-05-25); yaw rate was the last unconverted
field.

### r61 + yaw-flip root cause: perception's psi_err is mod-pi

r61 (first lookahead mission, seed 42) recorded all four markers at
exact cells and landed — but via the skipped-row FALLBACK sweep, not
candidates: search 235 sim s vs r60's 221 s. The side camera stared at
the EXPLORED side the whole flight. Root cause chain, from the log:

- At t~40 s the drone crossed marker 2 at (4,4): the marker's own
  edges (random orientation) hijacked the Hough vertical line,
  psi_err escalated -0.05 -> -0.44 chasing it, the drone spun, and
  after ~90 deg the PERPENDICULAR grid line read as "the" vertical
  line (line alignment is mod-pi) — the new heading is
  self-consistent to the line follower. It eventually settled at
  yaw ~ pi and stayed there for the rest of the flight (odom
  quaternion confirmed z=0.9999).
- Position control is yaw-agnostic (world_to_body), so r57/r60 flew
  correct missions in this state — invisible until the side camera
  made the yaw DIRECTION load-bearing. r60's log shows the same
  wz=+-2.5 saturation for the entire flight: the yaw lock stuck at
  the wrapped-angle antipode, where wrap(start-yaw) alternates sign
  every tick and a P controller dithers instead of unwinding.
- Proof the projection itself is right: from (6.4, 12.1) on the
  row-12 leg, the flipped camera (world -Y) saw marker 2 eight
  metres away and the attitude-compensated ray-cast voted EXACTLY
  node (4,4) — using the flipped DR yaw. `>> CANDIDATE id=2 ...
  range=8.6` was the smoking gun (already-recorded ids are filtered,
  so the mission was unaffected).

Fix: `dead_reckoning.resolve_locked_yaw_error` — perception psi_err
may only fine-trim while |wrap(start_yaw - yaw)| <= 0.6 rad (the
vertical-band half-width pi/6 plus margin; a legitimate nearest-line
error can never exceed it). Beyond that the absolute lock error
drives the unwind, and errors within 0.1 rad of -pi fold to +pi so
the unwind direction is deterministic. 8 new unit tests; the
marker-edge Hough hijack itself is an M-E follow-up (mask ArUco
quads before line detection).

### Tests: 144 -> 183 (side camera + candidate FSM + closed loop)

- `test_side_camera.py` (20): projection pinned against the downward
  cam's hardcoded map; band edges; the roll identity Rx(r)*Rz(pi/2) =
  Rz(pi/2)*Ry(-r) (roll = shallower mount); body pitch swings the hit
  to exactly x = -h*tan(p) — the initial "boresight is +Y so pitch is
  invariant" assumption was WRONG (the depressed ray has a -Z
  component that Ry rotates into -X); horizon/range gates; oblique
  6 px/module detection on a warped synthetic marker; tracker votes.
- `test_state_machine.py` (+13): row-skip planning + non-ascending
  guard, tick-top dedup, GOTO flows (record target / record different
  id en route / not-found drop / timeout / vote re-election),
  row-finish flush, short-circuit tour, fallback-once exhaustion.
- `test_mission_closed_loop.py` (+3): `SideCam` synthetic band model;
  seed-42-like corner layout completes with GOTO_CANDIDATE and exact
  cells; lookahead run beats the full sweep by >20 sim-seconds
  (measured ~130 s saving); side-blind marker on a skipped row is
  recovered by the fallback sweep.

Two design fixes surfaced by writing the tests (not tuning):

- Fallback sweep entry: `_plan_sweep(rows_override=...)` put the FAR x
  endpoint first, so a drone finishing the skip sweep at x_min would
  cut a diagonal to (x_max, row) and pass row markers outside the
  downward camera's 1.5 m radius. The fallback now routes to the near
  endpoint of the first row before traversing (`from_x`); the primary
  plan is unchanged (spawn sits at the inset by construction).
- Recorded-beats-candidate had a one-tick lag: the tick-top dedup
  filter runs before WAYPOINT_VISIT records, so a freshly recorded id
  stayed visible in `ctx.candidates` until the next tick. The record
  path now purges the id from candidates + queue in the same tick.

### FSM: GOTO_CANDIDATE + row-skip sweep + short-circuit

The search now consumes lookahead candidates (fed per tick as
`tick(candidates=tracker.snapshot(...))`, default None keeps every
legacy caller source-compatible):

- `sweep_row_step=2` (param; forced 1 when lookahead is disabled):
  `_plan_sweep` flies every other interior row — {4, 12} for our grid
  — and stashes the complement. Skip requires the distance-sorted
  rows to ASCEND (camera faces +Y = unexplored only then); otherwise
  reverts to step 1.
- Candidate dedup is FSM-owned and runs at tick top: recorded ids and
  dropped ids are filtered from every snapshot (the tracker keeps
  votes forever and knows nothing of the mission). Primary dedup key
  is the ArUco id; the per-(id, node) majority vote in the tracker
  guards against long-range ID misreads.
- New state GOTO_CANDIDATE (ARRANGE-style world-target navigation):
  flies to `candidates[queue[0]].xy` (live lookup — the target moves
  if votes re-elect a node). Any unrecorded downward sighting en
  route — target or not — goes through the unchanged WAYPOINT_VISIT
  snap-record; the queue entry stays until resolved, so the same
  target resumes after the interrupt. Arrival + candidate_wait (4 s)
  without a downward sighting drops the id permanently
  (`>> CANDIDATE-DROP`); goto_timeout 60 s is the stall guard.
- Visit scheduling: row-finish flush (candidates spotted from the
  finished row are visited during the transit — they sit on the row
  in between) and short-circuit (records + candidates >= 4 abandons
  the remaining sweep immediately). Nearest-neighbor chaining orders
  the queue.
- `_on_sweep_exhausted` (now also checked when GOTO returns to an
  already-exhausted sweep — previously unreachable, would have hit
  the gridless cruise-into-wall fallback): pending candidates first,
  then a ONE-SHOT fallback sweep over the skipped rows (safety net
  for missed side detections), then retrieval with what exists.

### side_camera.py — lookahead perception + candidate tracker

Pure module (no rclpy) for the new sideways OV9281+6mm camera
(world.md entry has the mount geometry and why sideways beats
forward under the +X yaw lock):

- `MountExtrinsics.rotation_body_optical()` = Rz(yaw)*Ry(pitch)*R_so,
  the same extrinsic-rpy composition as the SDF sensor pose. The
  downward camera is the (0, pi/2) degenerate case and reproduces the
  node's hardcoded `xb=-yc, yb=-xc` optical->body map — pinned by
  test.
- `project_pixel_to_ground`: attitude-compensated ray-cast onto z=0.
  Not the downward camera's depth=altitude shortcut — an oblique
  ray's ground hit moves ~h/sin^2(depression) per rad of attitude
  (~10 m/rad at the near band), so live roll/pitch (new: extracted
  from the /odom_truth quaternion) and yaw enter the rotation.
  Near-horizon rays (down-component < 0.02) and hits beyond
  lookahead_max_range are refused.
- `detect_aruco_side`: DetectorParameters tuned for the ~6 px/module
  foreshortened far quads (win step 4, perimeter rate 0.02, polygon
  accuracy 0.05, SUBPIX corners, errorCorrectionRate 0.8).
- `CandidateTracker`: votes per (id, nearest-intersection) pair;
  majority node wins, ties to most recent. Vote-on-node absorbs
  projection error the same way snap_to_intersection does (2 m
  tolerance). Deliberately ignorant of records/drops — that
  filtering is FSM-owned.
- Node wiring is passive this commit: subscribe, detect, project,
  vote, publish `/line_tracer/lookahead_debug_image` + cyan
  `aruco_candidate` spheres, log `>> CANDIDATE` on promotion. No
  flight-behavior change; detections do NOT enter PerceptionResult
  (the FSM treats `perception.aruco` as "marker below" — merging
  would snap-record 5 m off).

## Superseded state (2026-07-08 end — r57/r60 full 4-marker mission, video-verified)

r57 and r60 fly the complete competition flow (seed 42, ~12 min):
serpentine sweep over the interior grid rows finds and records ALL
FOUR markers at exactly their ground-truth cells (id2 (4,4), id0
(24,4), id3 (24,16), id1 (4,16) — err 0.00 m each), ARRANGE_BY_ID
tours the corners in ID order, and the drone homes to the start and
parks ~0.5 m from spawn. No DartSim aborts. r60's downward camera was
recorded for the whole mission (16768 frames) and encoded to a 10x
video with the detection overlay — each marker's yellow box + id is
visible during its 3 s hover. 162 tests green (144 pytest + 18
gtest).

What M-B added on top of the r54 platform:

- `_plan_sweep`: lawnmower waypoints over every interior grid row
  (markers only sit on intersections), starting from the row nearest
  the start, x endpoints inset 2 m from the border. LINE_FOLLOW
  follows the sweep; a WAYPOINT_VISIT interrupt resumes where it
  left off; sweep exhaustion with partial records still retrieves
  what was found.
- mission_max_records back to 4 (the rules' value).
- max_vxy 0.2 -> 0.5: the 0.2 cap dated from the open-loop era when
  commanded velocity was really acceleration. With the velocity loop
  the drone tracks 0.5 m/s and the camera still gets ~5 s of dwell
  per marker. The ~250 m mission needs the speed.
- arrange_timeout 420 s (r57 measured the nominal tour at ~330 sim
  seconds; the 300 s guard cut the final homing leg — harmless, it
  falls through to RETURN_PATH, but a backstop shouldn't fire on the
  nominal path).
- `scripts/run_mission.sh` takes a duration argument; the full
  mission needs ~900 wall seconds (sim RTF runs a bit under 1).

Marker hover raised to 3 s (waypoint_hover_seconds) so the detection
is visually verifiable: the ArUco overlay (yellow corner box + id)
publishes on /line_tracer/debug_image at ~16 Hz; `scripts/dev.sh
view` opens rqt_image_view on it alongside `dev.sh gui`/`mission`.

Remaining toward M-C/M-D/M-E: per-WP Z handling, mission time (the
sweep revisits empty rows — a smarter search or higher cruise speed
cuts minutes), robustness items (multi-frame ID voting, lost-line
recovery), and the test-reality gaps from the audit (driver calling
real node methods, MockDrone attitude lag, gz smoke tier).

## Superseded state (2026-07-08 — r54/r55 single-marker mission)

Two consecutive clean headless runs on the 4S power train: TAKEOFF ->
LINE_FOLLOW -> WAYPOINT_VISIT (marker 2 recorded at exactly (4,4),
err 0.00 m) -> ARRANGE_BY_ID -> RETURN_PATH -> LAND. Cruise holds
1.98 m for target 2.0; touchdown 0.5 m from the start point; the
disarmed drone stays parked (no post-landing drift). No DartSim
aborts. 159 tests green (141 pytest + 18 gtest).

What changed (root-cause fixes from the four-agent audit, not tuning):

| Root cause | Fix |
|---|---|
| `pitch_sp = vx/g` is open loop — with zero drag in gz it commands acceleration, speed integrates unbounded (r39 exited the arena, slid 200 m after touchdown) | Body-velocity loop: `pitch_sp = kp_vel*(vx_cmd - vx_meas)` with vx_meas from the /odom_truth xy derivative (message-stamp dt, alpha-0.5 smoothing) rotated into body frame. Open-loop mapping remains only as the no-measurement fallback. |
| Altitude loop is P-only, so a feed-forward mis-trim becomes a permanent offset: hover_ff 0.38 (measured on a zombie-contaminated run) vs clean plant 0.334 put every setpoint +0.27 m high — cruise 2.27, LAND parked at 0.27 m forever (r52) | hover_thrust_norm unified at 0.33 (param == dataclass default), and LAND descends on a velocity command (land_descent_vz -0.3 m/s) instead of the P law, so touchdown is trim-independent. Touchdown cutoff (<0.12 m) zeroes thrust and disarms. |
| Retrieval path collapses to length 1 when marker node == start node == current node (start (2,4) ties between x-nodes 0 and 4); `if idx >= len: LAND` pre-empted the `elif -> RETURN_PATH` branch — r52 skipped the whole return leg | ARRANGE exhaustion now always goes to RETURN_PATH, which homes on the EXACT start_xy (not its nearest node) with the existing 30 s timeout; obsolete path/idx guards removed from `_tick_return`. Plus: arrange_timeout 60 s stall guard, mission-phase timer resets in `_plan_retrieval`. |
| Takeoff drift walked the cruise off the marker row (r42 missed detection 1 m off-line); ground-phase position hold flung the drone off the sphere contact (r43) | TAKEOFF holds start_xy only above 0.5 m; LINE_FOLLOW uses a moving 2 m lookahead target so cross-track error steers ~27 deg instead of decaying over tens of metres. |
| FSM captured start_xy from the DR default (0,0) before the first odom fix (r41 returned to the wrong corner) | Sim builds gate the whole DR tick on the first /odom_truth pose. |

Operational prerequisites for reproducing (see docker.md): zombie-free
container (`scripts/run_mission.sh` sweep contract), ROS_DOMAIN_ID
pinned in compose.

Remaining for M-B/M-C: mission_max_records back to 4 + grid sweep for
off-axis corners; XY accuracy target <0.5 m per marker; retrieval
order + per-WP Z.

## Superseded state (2026-05-25 end-of-session)

### Algorithm: ✅ done

124/124 unit tests pass — closed-loop synthetic mission walks TAKEOFF → LINE_FOLLOW → WAYPOINT_VISIT (×4) → ARRANGE_BY_ID → RETURN_PATH → LAND, records 4 markers within 2 m of ground truth, lands within 3 m of spawn. See commits `de77a04` `1fd79a5` `f970604` `5266c6f` `5880c3e` `228f4b4`.

Pure-function decomposition that lets the algorithm be tested without rclpy:
- `dead_reckoning.compute_body_velocity()` — P on body offsets.
- `dead_reckoning.snap_to_intersection()` — absolute fix on marker sighting.
- `dead_reckoning.world_to_body()` — rotation for retrieval phase targets.
- `dead_reckoning.body_vel_to_atti_thr()` — body vel + alt + vz → AttiThrCmd.
- `state_machine.StateMachine.tick()` — automaton + MissionContext.
- `planner.arrange_by_id()` — BFS retrieval path.

### Integration: ✅ happy path lands (2026-05-25 r23)

`sweep_logs/mission/r23` is the first clean end-to-end run. Drone
takes off (1.78 -> 2.08 m), holds altitude for 65 s at 2.08 ±0.01 m,
flies +X, sees marker 2, hovers, records it, returns to LINE_FOLLOW.
PNG: `r23_tracer.png`.

What landed across `a71eae9`, `ed587ce`, `0d838f2`:

| Symptom | Fix |
|---|---|
| Drone parks at alt=0.05 m; thrust=0.70 doesn't break ground contact | Takeoff burst (alt < 0.30 + alt_err > 0.5 + vz_truth < 0.2 -> thrust=0.85). Mirrors `hover_pub.py:86`. The vz guard is one-sided so the burst fires while falling. |
| `kd_alt_thrust * vz_truth` blows up on DartSim garbage frames | `_on_odom_truth` rejects |z| > 50 m frames |
| `twist.linear.z` from gz OdometryPublisher had a frame mismatch with the PD damping sign (alt was decreasing while linear.z > 0) | vz is now finite-differenced from altitude itself, world frame by construction |
| Sanity-gate release at non-identity Gazebo yaw made cruise_vx fly the drone off-axis | Behavior.lock_yaw_to_initial drives psi_err = start_yaw - dr_state.yaw when perception doesn't have a fresh psi_err |
| Yaw lock had no signal in sim (DR.yaw was 0; Gazebo drone yaw was unobserved) | /odom_truth orientation -> DR.yaw at the top of `_on_dr_tick`. Sim-only path, gated by use_odom_truth_altitude. |
| Yaw lock during TAKEOFF saturated wz, interacted with sphere ground contact, drone spun in place without lifting | TAKEOFF behavior lock_yaw_to_initial=False. LINE_FOLLOW and later still lock. |
| Drone occasionally hits ground before sanity gate releases motors | Spawn altitude 1.5 -> 3.0 m. Buys ~0.5 s more free-fall to stabilize. |

Still open in r23 (not blocking for the visual demo):

| Symptom | Likely cause | Where to look |
|---|---|---|
| RECORD coord came out (+4.00, +0.00) when GT marker 2 is at (4.00, 4.00); the snap is off by one grid cell | DR.x/y isn't synced from /odom_truth pose — DR integrates body velocity from (0, 0). At WAYPOINT_VISIT the FSM snaps DR's drifted (x,y) to the wrong intersection. | `line_tracer_node._on_dr_tick` — extend the existing odom-yaw inject to also write `self._dr.state.x = msg.pose.pose.position.x` (sim-only) |
| Long trajectory curls ~200 m off-axis over 60 s — yaw lock has steady-state error ~0.07 rad against firmware drift | kp_yaw too small to fully cancel drift | bump `kp_yaw` or add an integral term |
| Garbage xy frames (|x| ~ 4700 m) pass the existing |z|>50 / |vz|>30 filter | `_on_odom_truth` only checks altitude/vz | add `|x|>200 \|\| |y|>200` reject in the same guard |

### Operational pitfall — zombie containers

r19/r20/r21/r22 all looked like algorithm regressions but the real
cause was host CPU starvation. Each prior `docker compose run --rm`
left a container alive: the inner `pkill -f 'gz sim'` killed the
*processes inside the container*, but the bash wrapper that owned the
`ros2 launch` PID was still alive, so the container itself never
exited and `--rm` never reaped it. After ~10 runs the host had ~9
parallel `gz sim` instances + `line_tracer_node` instances, load
average 56. A fresh tracer launched into that environment couldn't
get its 40 Hz timer to fire — log lines came out every 10 s instead
of every 0.5 s.

After `docker stop $(docker ps -q --filter name=uav)` + container
prune, load fell to ~20 and r23 (same code as r22) ran cleanly.

Future runs need either `pkill -9 -f 'ros2 launch'` in the cleanup
chain or `docker stop $(name)` from the host once the inner kill is
in flight.

### Resume guide

1. Run the existing test suite to confirm nothing rotted:
   ```
   docker compose run --rm -T uav-aruco bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && colcon test --packages-select line_tracer --packages-ignore realsense2_camera realsense2_camera_msgs && colcon test-result --test-result-base build/line_tracer"
   ```
   Expect 124/124.
2. Reproduce the sim issue:
   ```
   docker compose run --rm -T -v /home/dongha/uav26/sweep_logs:/sweep_logs uav-aruco bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ( ros2 launch world sim.launch.py headless:=true marker_seed:=42 > /sweep_logs/mission/rN_sim.log 2>&1 & ) && sleep 5 && stdbuf -oL -eL ros2 launch line_tracer line_tracer.launch.py > /sweep_logs/mission/rN_tracer.log 2>&1 & TRACER_PID=\$!; sleep 90; kill -INT \$TRACER_PID; wait; echo done"
   ```
3. Visualize: `scripts/plot_mission.py sweep_logs/mission/rN_tracer.log`. PNG appears next to the log.
4. Pick one of the three integration failure modes above and try a fix. The fixture in `test_mission_closed_loop.py` lets you A/B-test the algorithm-level change before re-running the sim.

## Planned (Open)

- **M-A demo polish (still rough).** End-to-end headless run goes
  TAKEOFF -> LINE_FOLLOW cleanly with the post-2026-05-25 fixes, but
  the drone often drifts at takeoff (drone sits on the sphere body
  collision during the few-second window before line_tracer engages,
  small contact perturbations push it laterally before the takeoff
  burst reaches 1.8 m). DartSim's ODE collision detector also still
  aborts occasionally with the firmware-native 0.40 / 0.80 gains —
  the demo currently runs on the halved 0.20 / 0.40 sim.launch
  defaults to dodge it.
- ArUco detection-during-flight not yet verified in end-to-end mode.
  The FSM transition path (LINE_FOLLOW -> WAYPOINT_VISIT -> ...) is
  unit-tested but has not yet caught a marker during a real run.
- M-B (XY accuracy < 0.5 m) + M-C (retrieval order + Z) + M-D (time)
  + M-E (robustness) remain as listed in the M-A plan; none are
  blocking for the smoke-test demo.

## Done

### Takeoff burst + odom-truth garbage gate (2026-05-25, r14 -> r15)

Two minimum-change fixes that unstuck the M-A integration:

- `body_vel_to_atti_thr` gained an open-loop takeoff branch
  (`takeoff_thrust_norm=0.85`) for the {alt < 0.30, |vz| < 0.2, alt_err
  > 0.5} corner. Mirrors the proven pattern in `hover_pub.py:86`.
  Plain PD with thrust_max=0.70 (calibrated for in-flight hold) wasn't
  enough to break the sphere/ground contact in DartSim — even though
  the implied thrust force (15 N) safely exceeds the drone's weight
  (11.6 N).
- `_on_odom_truth` rejects frames with |z| > 50 m or |vz| > 30 m/s.
  DartSim's ODE collision detector occasionally publishes nonsense
  odom contact frames (alt = -2.6e6, vz = ±5000). Before the gate,
  those poisoned `kd_alt_thrust * vz_truth` and the thrust setpoint
  oscillated between thrust_min and thrust_max, slamming the motors
  and feeding back into more DartSim instability.

Four new tests in `TestTakeoffBurst` pin the new branch (burst when
on ground, no burst when airborne / already rising / target below
self). Plus one existing PD-clamp test was nudged above the burst
threshold so it actually exercises the clamp path. Total: 128/128
pytest pass.

Why both fixes together: the failure modes were a cascade
(ground-stick -> sanity gate cycles -> motor slam -> ODE garbage ->
vz spikes -> thrust oscillation -> more ground impact), so neither
fix alone would land a clean takeoff.

What r15 shows: drone reaches 2.26 m and the FSM walks through
TAKEOFF -> LINE_FOLLOW -> WAYPOINT_VISIT -> LINE_FOLLOW. The
remaining yaw-drift + altitude-droop issues are scoped above under
the new Open section.

### Test rewrite — closed-loop + edge cases (commits `5266c6f` `5880c3e` `228f4b4`, 2026-05-25)

After r10 / r14 visualizations showed the 104 existing tests passing while the actual sim run left the drone 90 m outside the mission area, rewrote the test suite. Total: 96 → 124 tests (+28).

What the old tests checked: "FSM transitions correctly given synthetic dr_state / perception / altitude inputs." What they missed: "the drone actually follows the intent the FSM expressed." The pitch-sign bug that crashed r09 lived in `LineTracerNode._build_setpoint` for two days because it was a Node method — no pure-function test.

Refactor: extracted the sign / clamp / thrust-PD logic from `_build_setpoint` into `dead_reckoning.body_vel_to_atti_thr(vel, target_alt, altitude, vz_truth, gains) -> AttiThrCmd`. The Node method is now a thin rclpy wrapper.

New tests:

- **`TestAttiThrSign` (6 tests)**: `+vx -> +pitch_sp`, `+vy -> -roll_sp`, clamp behaviour, hover-thrust on target, descending-vz increases thrust. The pitch sign bug would have failed `test_positive_vx_yields_positive_pitch` immediately.
- **`TestMissionClosedLoop` (6 tests)**: `MockDrone` (point-mass + linear drag) + `SyntheticCam` (radius-based ArUco visibility) + a 40 Hz mock control loop. Drives one synthetic mission (4 markers on the y=4 line) end-to-end, asserts FSM reaches LAND, all 4 markers recorded within `snap_max_err`, state sequence is in order, drone lands near spawn.
- **`TestMissionTickEdgeCases` (8 tests)**: altitude streak reset on dip, snap refusal beyond `max_err`, same-id repeat during WAYPOINT_VISIT not resetting the timer, perception drop mid-hover not breaking the timer, `records_full` with `grid=None` staying safe, `target_xy_world` only in retrieval phases, LAND terminal, `set_state` override + tick re-application.

The closed-loop tests still don't replace a real sim run — they assume instantaneous attitude tracking and don't simulate gz/DartSim's contact constraint. But "FSM produces a self-consistent set of intents that converge on a sim drone" is now a unit test, not a hope.

### M-A end-to-end FSM scaffolding (commits `de77a04` `1fd79a5` `f970604` `9876c18` `c7385e7` `b17a25d` `f161c42` `810ef36`, 2026-05-25)

Single-shot launch (`world/sim.launch.py` then `line_tracer.launch.py`)
now drives the FSM from TAKEOFF through LAND without manual
`set_state` calls. Pieces:

- **`dead_reckoning.snap_to_intersection(state, grid, max_err)`** —
  pure helper. Every ArUco sighting is an absolute XY fix because the
  rules place markers on grid intersections; the FSM snaps the DR
  pose to the nearest intersection during WAYPOINT_VISIT.
- **`dead_reckoning.world_to_body(err_x, err_y, yaw)`** — inverse of
  `integrate()`'s rotation, used during ARRANGE_BY_ID / RETURN_PATH
  so the node can consume the FSM's world-frame target.
- **`state_machine.MissionContext` + `tick(now, dr_state, perception,
  altitude)` + `TickResult`** — drives the automaton
  `TAKEOFF -> LINE_FOLLOW <-> WAYPOINT_VISIT -> ... -> ARRANGE_BY_ID
  -> RETURN_PATH -> LAND`. `set_state` stays as a manual override
  hook but `tick` overwrites it next call. 104/104 unit tests pass.
- **`line_tracer_node` rewiring** — calls `_fsm.tick(...)` every
  control tick, logs `>> FSM:` / `>> RECORD` events so headless runs
  can be grep'd. When `tick` emits `target_xy_world`, the node
  bypasses perception and feeds world-to-body deltas to DR. Truth-
  frame xy/alt/vz from `/odom_truth` are logged 1 Hz for downstream
  plotting.
- **Altitude controller hardening** — `/odom_truth` subscriber under
  `use_odom_truth_altitude=true` is the sim stand-in for the
  proposal's LIDAR + 2-state KF (item I1). Thrust formula gains a
  `kd_alt_thrust` term, a `[0.42, 0.70]` clamp, and a depth-camera
  short-circuit when truth is the source. `dr_dt` 50 ms -> 25 ms so
  the publisher stays well inside fc_core's freshness window.
- **`fc_core/COMP_STALE_MS` 50 -> 200** — a 20 Hz companion racing
  with the old 50 ms threshold was triggering periodic
  fall-back-to-descent ticks at the FC. 200 ms tolerates the same
  publish rate with realistic jitter.
- **LINE_FOLLOW behaviour** — `use_forward_error=False`, dv is the
  grid line *crossed* on every cell, not a target to snap back to;
  cruise_vx 0.4 -> 0.5 to scan faster.
- **Pitch / roll sign** — `pitch_sp = +vx/g`, `roll_sp = -vy/g`
  (matches the empirical sim convention after the 2026-05-25
  pitch-shim fix). Previous `-vx/g` made vx body commands map to -X
  world; visible in `r10_tracer.log` where the drone consistently
  flew backwards.
- **`sim.launch.py` gain defaults reverted** to half (0.20 rate /
  0.40 atti) — the firmware-native 0.40 / 0.80 destabilised DartSim
  at takeoff. C++ defaults stay at 0.40 / 0.80 for the eventual
  hardware path.
- **`scripts/plot_mission.py`** — reads a headless mission log,
  produces a 3-panel PNG (top-down trajectory coloured by state,
  altitude vs time with state bands, state strip) plus a text diff
  of recorded markers vs `aruco_layout.yaml` ground truth.

Open follow-ups documented above; M-B / M-C / M-D / M-E milestones
defined in the M-A plan still apply.



### Planning step 1 — grid + BFS (2026-04-30, committed in the housekeeping pass)

- `line_tracer/grid.py` — Grid dataclass over the 30x20 arena, 4-connectivity edges on (i, j) intersection nodes, `marker_node()` lookup by ArUco id against an externally-supplied layout.
- `line_tracer/planner.py` — BFS `shortest_path`, `visit_in_order`, `arrange_by_id`, `path_length_m`. Pure functions, no rclpy.
- `test/test_grid.py` + `test/test_planner.py` — boundary/neighbor expansion, BFS correctness, ordered-waypoint chaining, id-sorted marker traversal.
- Decision: BFS over Dijkstra because edges are uniform-cost. Marker layout is passed in as a dict, not loaded from yaml — yaml loading stays a node-layer concern.

### Steps 1-8 (commit `9cf03bf`, 2026-04-29)

#### `line_tracer_msgs/` (commit `b57e20f`, `c216f08`)
- ament_cmake 패키지
- `srv/SetState.srv` — FSM 상태 전이용 (string state -> bool success + string message)

#### `line_tracer/` package skeleton (commit `b57e20f`, `c216f08`)
- ament_python 패키지; deps rclpy/sensor_msgs/geometry_msgs/nav_msgs/std_msgs/cv_bridge/tf2_ros/line_tracer_msgs.
- entry point: `line_tracer_node = line_tracer.line_tracer_node:main`.

#### Step 2 — `geom.py` (commit `d2bcdbe`)
- `CameraIntrinsics` dataclass + `.from_camera_info()` factory.
- `pixel_offset_to_meters(du, dv, depth, intr)` — Δx = du·d/fx, Δy = dv·d/fy.

#### Step 3 — `dead_reckoning.py` (commit `349e255`)
- `Gains`, `State`, `BodyVelocity` dataclasses + `wrap_angle` / `clamp` helpers.
- `compute_body_velocity(dx_body, dy_body, psi_err, z_hat, gains)` — P control + clamp.
- `integrate(state, vel, dt)` — yaw rotation then forward-Euler.
- 22 tests; frame: body FLU (REP-103), world ENU. Camera->body rotation lives in the caller.

#### Step 4 — `perception.py`
- `PerceptionConfig`, `PerceptionResult`, `ArucoDetection` dataclasses.
- Canny + HoughLinesP line detection, vert/horiz classification, nearest-line pick, `compute_pixel_errors` -> (du, dv, psi_err).
- ArucoDetector with legacy fallback; DICT_6X6_250.
- Sign convention: du>0 -> line on image right -> -y_body; dv>0 -> line below -> -x_body; psi>0 -> +wz.
- 22 perception tests.

#### Step 5 — `state_machine.py`
- `StateName` enum + `Behavior` dataclass + `StateMachine` container.
- TAKEOFF (climb only), LINE_FOLLOW (lateral+heading+forward+cruise), LAND (target_alt=0). WAYPOINT_VISIT / ARRANGE_BY_ID / RETURN_PATH currently behave as LINE_FOLLOW stubs.
- 17 sm tests.

#### Step 6 — `line_tracer_node.py`
- Subscribes color/depth/camera_info + `/line_tracer/pixel_error` override.
- Publishes `/cmd_vel`, `/odom_dr`, `/waypoints/aruco`, `/line_tracer/debug_image`.
- `/line_tracer/set_state` service (line_tracer_msgs/SetState).
- 20 Hz DR timer: pixel -> body conversion -> P control -> cmd_vel + odom_dr.
- Depth unit auto-handling: 16UC1 [mm] (RealSense) or 32FC1 [m] (sim).

#### Step 7 — launch / config / README
- `launch/line_tracer.launch.py` (sim:=true / false 분기, realsense2_camera include).
- `config/params.yaml` (Kp들, Hough/Canny, dr_dt 등).
- README — 토픽/파라미터 표 + frame 약속 + FC 핸드오프 가이드.

#### Step 8 — build / test / integration
- `colcon build --packages-select line_tracer_msgs line_tracer world` 통과.
- `pytest test/` 71/71 통과 (geom 10 + dr 22 + perception 22 + sm 17).
- world sim + line_tracer 동시 실행 (headless): TAKEOFF -> alt=2.0 m climb, hover. LINE_FOLLOW + sustained `/line_tracer/pixel_error` 80, -100, 0.1 -> `vx=+0.34 vy=-0.28 vz=-0.00 wz=+0.10`.

## Decisions

- FSM transitions are manual via `/line_tracer/set_state` on purpose. We rejected "altitude-reached -> LINE_FOLLOW" auto-transitions because they couple FSM to one mission flow and would fight WAYPOINT_VISIT / ARRANGE / RETURN later. If manual calls become annoying during dev, write a 5-line helper script — don't bake auto-transitions into the FSM.
- `/cmd_vel` frame is FLU (REP-103). When the Setpoint refactor lands, the small-angle velocity -> attitude map happens inside line_tracer_node so the FSM stays velocity-level.
- Camera mount yaw offset = 0 (drone forward = image v-up).
- Planning step 1 keeps grid/planner as pure functions (no rclpy, no I/O); they're the foundation of the executor's reasoning layer.
