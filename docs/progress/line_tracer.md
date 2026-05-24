# line_tracer

Vision-driven companion: downward camera -> Hough line + ArUco -> dead reckoning + FSM -> setpoint to FC.

## Planned (Open)

- Replace the Twist publisher (`/cmd_vel`) with a `fc_sim_msgs/Setpoint` publisher on `/fc/setpoint`. Map the existing body-velocity command into roll/pitch setpoints via the small-angle approximation (`pitch_sp = -vx/g`, `roll_sp = +vy/g`, clamped ±0.2 rad). Add an altitude-hold P controller for `thrust_norm` against the `/uav26_quad/range` topic.
- Update Twist-asserting unit tests to assert on Setpoint fields instead.
- FSM transitions stay manual via `/line_tracer/set_state`. The Setpoint plumbing does not change the existing state machine.
- The 3-layer planning stack continues from here: `path_executor` (step 2) consuming the planned node list and emitting per-tick intersection actions on top of LINE_FOLLOW. ArUco-PnP localization will land alongside the executor.

## Done

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
