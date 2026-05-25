# line_tracer

Vision-driven companion: downward camera -> Hough line + ArUco -> dead reckoning + FSM -> setpoint to FC.

## Current state (2026-05-25 end-of-session)

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
