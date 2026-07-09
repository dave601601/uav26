# line_tracer

ROS 2 (Jazzy) package implementing the competition stack for the
outdoor missing-person search task: grid search over a grass field
with white satin grid lines (official 2026-07 spec: 3 m cells, 10 cm
lines, 0.4 m ArUco marker sheets on interior intersections, 4 unique
IDs from 0..49), using two cameras:

- a downward RealSense D435 (line tracking + marker recording), and
- a sideways OV9281 + 6 mm "lookahead" camera (boresight body +Y,
  depressed 26 deg) that observes the adjacent grid row while the
  serpentine sweeps the current one, so the sweep can skip rows and
  fly directly to believed marker positions (candidates).

The node speaks `fc_sim_msgs/Setpoint` to the flight controller
(fc_sim_node running the fc_core firmware in sim; the STM32 over
USART2 on hardware). Arena dims (30 x 21 m) and the ArUco dictionary
(DICT_4X4_50) are parameterized assumptions until the rules confirm
them.

See `docs/PROGRESS.md` for the roadmap and
`docs/progress/line_tracer.md` for the work log.

---

## Architecture

```
 downward camera (color+depth+info)        lookahead camera (mono+info)
          │                                          │
          ▼                                          ▼
  ┌────────────────────┐                   ┌──────────────────────────┐
  │ perception.py      │                   │ side_camera.py           │
  │ (Hough lines +     │                   │ (oblique ArUco detect +  │
  │  ArUco below)      │                   │  attitude-compensated    │
  └────┬───────────────┘                   │  ground ray-cast +       │
       │ du, dv, psi_err, aruco            │  CandidateTracker votes) │
       ▼                                   └────────────┬─────────────┘
  ┌─────────────────────────────────────┐   candidates  │
  │ state_machine.py                    │ ◄─────────────┘
  │ TAKEOFF → LINE_FOLLOW ↔ GOTO_       │
  │ CANDIDATE ↔ WAYPOINT_VISIT →        │
  │ ARRANGE_BY_ID → RETURN_PATH → LAND  │
  └────┬────────────────────────────────┘
       │ Behavior + target_xy_world
       ▼
  ┌──────────────────────────┐
  │ dead_reckoning.py        │  velocity loop + yaw lock +
  │ body_vel_to_atti_thr     │  NED yawrate flip + thrust PD
  └────┬─────────────────────┘
       ▼
  /fc/setpoint (fc_sim_msgs/Setpoint, firmware NED semantics)
```

Every layer below the node is pure Python (no rclpy / ROS msg deps)
so unit tests run out of tree: 192 pytest in `test/`.

| Module | Purpose |
|---|---|
| `geom.py` | Camera intrinsics + pixel-to-meter projection |
| `perception.py` | Canny+Hough grid lines, downward ArUco, dictionary registry (`aruco_dict` param) |
| `side_camera.py` | Lookahead detection, mount-parametric ground ray-cast, per-(id, node) vote tracker |
| `dead_reckoning.py` | Velocity/yaw/thrust mapping, `resolve_locked_yaw_error`, intersection snap |
| `grid.py` / `planner.py` | Grid graph + BFS retrieval routing |
| `state_machine.py` | Mission FSM incl. row-skip sweep, candidates, GOTO_CANDIDATE, fallback |
| `line_tracer_node.py` | rclpy node — wires the layers above |

Search strategy: with `sweep_row_step: 2` the serpentine flies every
other interior row; the lookahead camera covers each skipped row from
3 m away (one cell). Side detections become candidates after
`lookahead_vote_threshold` sightings agree on a grid node; candidates
are visited (GOTO_CANDIDATE) and recorded by the DOWNWARD camera's
intersection snap — the side camera never records. A one-shot
fallback sweep over the skipped rows backstops missed side
detections. Recorded/dropped IDs are filtered from candidates every
tick (FSM-owned dedup; the ArUco ID is the dedup key, the vote
majority guards long-range misreads).

---

## Topics

### Subscribed
| Topic | Type | Notes |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | downward, BGR8 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 32FC1 (sim, m) or 16UC1 (RealSense, mm) — auto-detected |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| `/camera/lookahead/image_raw` | `sensor_msgs/Image` | sideways OV9281 model, mono8 |
| `/camera/lookahead/camera_info` | `sensor_msgs/CameraInfo` | lookahead intrinsics |
| `/odom_truth` | `nav_msgs/Odometry` | sim-only truth inject (pose, yaw, roll/pitch for the side ray-cast) |
| `/line_tracer/pixel_error` | `geometry_msgs/Vector3` | `(du, dv, psi_err)` external override / test handle |

### Published
| Topic | Type | Notes |
|---|---|---|
| `/fc/setpoint` | `fc_sim_msgs/Setpoint` | ATTITHR mode; fields use the FIRMWARE's NED semantics (`yawrate_sp` + = yaw right/CW — the node negates its FLU wz) |
| `/odom_dr` | `nav_msgs/Odometry` | world ENU DR estimate |
| `/waypoints/aruco` | `visualization_msgs/MarkerArray` | yellow = downward detections, cyan ns `aruco_candidate` = lookahead candidates |
| `/line_tracer/debug_image` | `sensor_msgs/Image` | downward overlay (lines + ArUco) |
| `/line_tracer/lookahead_debug_image` | `sensor_msgs/Image` | lookahead overlay (detections + projected xy) |

(`/cmd_vel` Twist remains only as a fallback when `fc_sim_msgs` is
not built.)

### Service
| Name | Type | Notes |
|---|---|---|
| `/line_tracer/set_state` | `line_tracer_msgs/SetState` | manual override; `tick()` may overwrite on the next cycle |

Grep-able mission events in the node log: `>> FSM:` (transitions),
`>> RECORD` (marker recorded at snapped cell), `>> CANDIDATE` /
`>> CANDIDATE-DROP` (lookahead promotions / give-ups).

---

## Key parameters (`config/params.yaml` + node defaults)

| Key | Default | Meaning |
|---|---|---|
| `target_altitude` | 2.0 | [m] cruise altitude (side-camera band geometry assumes this) |
| `grid_width` / `grid_depth` / `grid_cell` | 30.0 / 21.0 / 3.0 | official 3 m cells; 30x21 arena is an assumption |
| `marker_size` | 0.4 | [m] official sheet size |
| `aruco_dict` | `"4X4_50"` | dictionary assumption for "IDs 0..49"; swap when the rules confirm (regenerate world textures too) |
| `mission_max_records` | 4 | markers to record before retrieval |
| `sweep_row_step` | 2 | fly every 2nd interior row (forced 1 when `lookahead_enable` false) |
| `lookahead_enable` | true | side camera + candidate pipeline |
| `lookahead_mount_yaw/pitch` | pi/2 / 0.4538 | mount extrinsics — MUST mirror the `lookahead` sensor pose in `world/models/uav26_quad/model.sdf` (26 deg depression) |
| `lookahead_vote_threshold` | 3 | sightings on one node before candidate |
| `lookahead_max_range` | 9.0 | [m] slant gate (band far edge ~7.8 m) |
| `lookahead_snap_max_err` | 1.5 | [m] vote acceptance = half cell |
| `candidate_wait_seconds` | 4.0 | hover on a voted node before dropping the id |
| `snap_max_err` | 2.0 | [m] downward record snap tolerance |
| `max_vxy` / `max_wz` | 0.5 / 2.5 | velocity clamps |
| `dr_dt` | 0.025 | [s] control period (40 Hz) |
| `use_odom_truth_altitude` | true | sim truth inject; false on hardware |
| `canny_low/high`, `hough_*` | 60/180, 60/40/20 | line detection (polarity-agnostic; white-on-grass verified) |

---

## Sign / frame conventions

| Frame | Convention |
|---|---|
| World | ENU (REP-103). x east, y north, z up |
| Body | FLU (REP-103). +x forward, +y left, +z up |
| Downward camera | optical +Z = -Z_body (straight down), yaw offset 0 |
| Lookahead camera | boresight = +Y_body depressed 26 deg (`Rz(pi/2)·Ry(0.4538)`); ground band lateral 2.64..7.56 m at 2 m altitude |
| Setpoint | FIRMWARE NED: `yawrate_sp` positive = yaw right (CW). The node emits `-wz`; measured 2026-07-09 (+0.3 rad/s command turned the gz drone -1.68 rad) |

Pixel-error sign convention (perception → node):
* `du > 0` ⇒ line right of center ⇒ `-y_body` motion.
* `dv > 0` ⇒ line below center ⇒ `-x_body` motion.
* `psi_err > 0` ⇒ +`wz` (CCW) — but the yaw lock overrides perception
  whenever |start_yaw − yaw| > 0.6 rad (line alignment is mod-pi and
  blind to 90/180-deg flips; see `resolve_locked_yaw_error`).

---

## Run

Daily driver (host, repo root — wraps container, build, X11, teardown):
```bash
scripts/dev.sh gui [seed]       # Gazebo window + tracer
scripts/dev.sh view             # rqt_image_view on the downward overlay
scripts/dev.sh mission rNN 1200 # headless mission + FSM event summary
scripts/dev.sh build
```

Manual (inside the container):
```bash
ros2 launch world sim.launch.py headless:=true marker_seed:=42
ros2 launch line_tracer line_tracer.launch.py     # ~8 s after the sim
```

External pixel-error override (no perception):
```bash
ros2 topic pub --once /line_tracer/pixel_error geometry_msgs/Vector3 \
    "{x: 50.0, y: 0.0, z: 0.0}"
```

On real hardware: `sim:=false use_sim_time:=false` pulls in
`realsense2_camera`; the OV9281 driver, mount bracket (26 deg) and
intrinsic/extrinsic calibration are open hardware items.

---

## Open follow-ups

- Candidate visit-policy optimization (M-D): cheapest-detour insertion
  into the remaining sweep instead of the unconditional row-end tour —
  the r70/r72 A/B showed the tour can double back ~50 m on layouts
  whose markers already lie on the serpentine path.
- M-E robustness: mask ArUco quads before the Hough line detection
  (marker edges hijacked psi_err in r61), multi-frame ID voting on the
  downward record path, lost-line recovery.
- Hardware: OV9281 bracket + calibration, LIDAR/flow estimator to
  replace the sim truth inject, satin-ribbon specularity unknowns.
