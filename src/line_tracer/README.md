# line_tracer

ROS 2 (Jazzy) package implementing the **competition stack** for the
indoor missing-person search task: line tracing on a 30 × 20 m painted
grid using a single downward-facing RealSense D435, with a
dead-reckoning motion stub that publishes `/cmd_vel` for an external
flight controller (Gazebo's MulticopterVelocityControl in sim, a TBD
custom STM32 FC on hardware).

See `260428.md` for the original task spec and `PROGRESS.md` for the
overall roadmap.

---

## Architecture

```
   camera (color, depth, info)
          │
          ▼
  ┌────────────────────┐            external test/debug
  │  perception        │   /line_tracer/pixel_error  (geometry_msgs/Vector3)
  │  (Hough + ArUco)   │           ───────────► ┐
  └────┬───────────────┘                         │
       │  (du, dv, psi_err)                      │
       ▼                                         ▼
  ┌────────────────────┐         ┌──────────────────────────┐
  │  state_machine     │ ──────► │  dead_reckoning  (P)     │
  │  (FSM Behavior)    │         │  vx vy vz wz             │
  └────────────────────┘         └────┬─────────────────────┘
                                      │
                                      ▼
                                 /cmd_vel    (geometry_msgs/Twist, body FLU)
                                 /odom_dr    (nav_msgs/Odometry,    world ENU)
                                 /waypoints/aruco   (visualization_msgs/MarkerArray)
                                 /line_tracer/debug_image  (sensor_msgs/Image)
```

Every layer below the node is **pure-Python** (no rclpy / ROS msg deps)
so unit-tests run out-of-tree. Tests live in `test/` and are exercised
via `pytest`.

| Module | Purpose | Tests |
|---|---|---|
| `geom.py` | Camera intrinsics + pixel↔meter projection | `test_geom.py` (10) |
| `dead_reckoning.py` | P-controller + Euler integrator | `test_dead_reckoning.py` (22) |
| `perception.py` | Canny+Hough grid lines, ArUco, debug overlay | `test_perception.py` (22) |
| `state_machine.py` | TAKEOFF / LINE_FOLLOW / LAND FSM (+ stubs) | `test_state_machine.py` (17) |
| `line_tracer_node.py` | rclpy node — wires the layers above | (integration) |

---

## Topics

### Subscribed
| Topic | Type | Notes |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | BGR8 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 32FC1 (sim, m) or 16UC1 (RealSense, mm) — auto-detected |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | required to project pixels |
| `/line_tracer/pixel_error` | `geometry_msgs/Vector3` | `(du, dv, psi_err)` — external override / unit-test handle |

### Published
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | body FLU; `linear.x/y/z` + `angular.z` |
| `/odom_dr` | `nav_msgs/Odometry` | world ENU; pure DR estimate (no IMU/GPS) |
| `/waypoints/aruco` | `visualization_msgs/MarkerArray` | one yellow sphere per detected marker, in `world` frame |
| `/line_tracer/debug_image` | `sensor_msgs/Image` | grid+ArUco overlay |

### Service
| Name | Type | Notes |
|---|---|---|
| `/line_tracer/set_state` | `line_tracer_msgs/SetState` | request: `string state` ∈ {TAKEOFF, LINE_FOLLOW, WAYPOINT_VISIT, ARRANGE_BY_ID, RETURN_PATH, LAND} (case-insensitive). Stub states behave as LINE_FOLLOW until their planners ship. |

---

## Parameters (`config/params.yaml`)

| Key | Default | Meaning |
|---|---|---|
| `target_altitude` | 2.0 | [m] cruise altitude |
| `kp_xy` | 0.8 | P gain on body-x/-y position error |
| `kp_yaw` | 1.0 | P gain on heading error |
| `kp_z` | 0.6 | P gain on altitude error |
| `max_vxy` | 1.0 | [m/s] linear velocity clamp |
| `max_wz` | 1.0 | [rad/s] yaw rate clamp |
| `dr_dt` | 0.05 | [s] DR integration period (20 Hz) |
| `external_override_ttl` | 0.5 | [s] freshness window for `/line_tracer/pixel_error` override |
| `publish_debug_image` | true | toggle `/line_tracer/debug_image` |
| `canny_low` / `canny_high` | 60 / 180 | Canny thresholds |
| `hough_threshold` | 60 | Hough vote threshold |
| `hough_min_line_length` | 40 | min Hough segment length [px] |
| `hough_max_line_gap` | 20 | max gap inside a Hough segment [px] |
| `marker_size` | 0.5 | [m]; reserved for PnP pose later |

---

## Sign / frame conventions

| Frame | Convention |
|---|---|
| World | ENU (REP-103). x east, y north, z up |
| Body | FLU (REP-103). +x forward, +y left, +z up |
| Camera mount | base_link bottom, optical +Z = -Z_body (수직 하향), yaw offset = 0 |

Pixel-error sign convention (set by `perception.py`, consumed by node):
* `du > 0` ⇒ line is to the right of image center ⇒ drone is left of the
  line ⇒ node yields `-y_body` movement (`vy < 0`).
* `dv > 0` ⇒ line is below image center ⇒ drone is past the line ⇒ node
  yields `-x_body` movement (`vx < 0`).
* `psi_err > 0` ⇒ apply +`wz` (CCW around +z_body) to align body forward
  with the line.

The pixel→body mapping inside the node:
```
dx_body = -dv * altitude / fy
dy_body = -du * altitude / fx
psi      passes through
```

---

## Run

### In the Gazebo sim (this repo)
```bash
# Terminal 1 — world (publishes /camera/.../*, accepts /cmd_vel)
ros2 launch world sim.launch.py

# Terminal 2 — line_tracer
ros2 launch line_tracer line_tracer.launch.py use_sim_time:=true

# Once airborne, switch into line tracking:
ros2 service call /line_tracer/set_state line_tracer_msgs/srv/SetState \
    "{state: 'LINE_FOLLOW'}"
```

### Manual override (no perception)
```bash
ros2 topic pub --once /line_tracer/pixel_error geometry_msgs/Vector3 \
    "{x: 50.0, y: 0.0, z: 0.0}"   # 50 px lateral offset → vy < 0
```
The override is honored for `external_override_ttl` seconds, then the
node falls back to perception output.

### On real hardware (when STM32 FC + RealSense are wired)
```bash
ros2 launch line_tracer line_tracer.launch.py sim:=false use_sim_time:=false
```
This pulls in `realsense2_camera` and skips the world include. Whichever
node speaks to the FC needs to subscribe to `/cmd_vel` and emit motor
commands; the line_tracer side is unchanged.

---

## Future work hooks

- **Custom flight controller**: replace whatever consumes `/cmd_vel`.
  Topic name and frame are fixed (FLU body Twist) so the FC adapter can
  swap independently.
- **Waypoint planner**: implement real behaviors for `WAYPOINT_VISIT`,
  `ARRANGE_BY_ID`, `RETURN_PATH` in `state_machine.py`. Today they are
  stubs that fall through to LINE_FOLLOW so the loop runs end-to-end.
- **IMU / EKF fusion**: today `/odom_dr` is pure dead-reckoning. When
  IMU arrives, fuse and drop the cruise-altitude `_altitude_m` patch
  inside `_on_dr_tick`.
