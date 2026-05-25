# line_tracer

Vision-driven companion: downward camera -> Hough line + ArUco -> dead reckoning + FSM -> setpoint to FC.

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
