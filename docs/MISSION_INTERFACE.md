# Mission Interface Spec (skeleton architecture)

Target architecture for the `feat/mission-skeleton-interface` branch.
Reference implementation sketch: `docs/mission_skeleton.py` (verbatim
from the team; deviations listed at the bottom). Three layers:

```
Jetson (Python, line_tracer)                MCU (C, fc_core / STM32)
┌──────────────────────────────┐            ┌──────────────────────────┐
│ perception -> PerceptionData │  McuCommand│ mission_ctrl (outer loop)│
│ sensors    -> SensorData     │ ──────────>│   -> COMP setpoints      │
│ MissionManager.step()        │  (serial / │ Control() attitude/rate  │
│   node-based navigation      │   ROS sim) │   cascade (unchanged)    │
└──────────────────────────────┘            └──────────────────────────┘
```

The Jetson decides WHAT to do (mission state, mode, direction, raw
vision errors). The MCU decides HOW to fly it (error -> velocity ->
attitude -> mixer). `Control()` and everything below it is untouched;
`mission_ctrl` writes the same `COMP` struct the old setpoint path
wrote.

Scope of this branch: node-based navigation is the main algorithm,
world-meter positions are logged alongside (never used for control
decisions except the two snap points defined below). The side
(lookahead) camera is out of scope. The legacy Setpoint path stays
selectable via a node parameter for A/B.

## 1. Enums (fixed values, shared Jetson/MCU)

MissionState (skeleton values, unchanged):
INIT=0 TAKEOFF=1 LOCALIZE=2 ENTER_GRID=3 EXPLORE=4 MARKER_CONFIRM=5
PLAN_RESCUE_PATH=6 FOLLOW_RESCUE_PATH=7 RETURN_HOME=8 LAND=9
FINISHED=10 FAILSAFE=11

ControlMode (replaces the skeleton's mode strings — a serial link
cannot carry strings; log `.name` for readability):
HOLD=0 FOLLOW_LINE=1 ALIGN_MARKER=2 SEARCH_LINE=3 MOVE_TO_LANDMARK=4
LAND_ON_MARKER=5 STOP=6 EMERGENCY_LAND=7

MoveDirection (skeleton values, unchanged):
X_POS=0 X_NEG=1 Y_POS=2 Y_NEG=3   (body FLU: +x forward, +y left)

## 2. Grid and frames

- Arena 30 x 21 m, cell 3 m -> 11 x 8 intersection nodes (user-confirmed;
  skeleton's 24 x 15 default is replaced). All three are parameters.
- Node (0,0) = world (0,0) (arena SW corner, same as `world`/`grid.py`);
  node (i,j) = world (i*cell, j*cell). Grid axes are aligned with body
  axes because yaw is locked to the initial heading (existing yaw-lock
  behavior).
- Body frame is REP-103 FLU. The FLU -> NED yawrate negation lives in
  mission_ctrl's setpoint mapping, exactly where `body_vel_to_atti_thr`
  has it today.

## 3. Navigation (node-based, meters logged)

- Position = integer node + MoveDirection, advanced by intersection
  events (one node per detected crossing), exactly as the skeleton does.
- Meters appear in two places only:
  1. Logging: every node update / state change logs
     `node=(i,j) -> nominal (x,y) m, dr=(x,y) m` using the DR estimate.
  2. Snap corrections:
     - ENTER_GRID -> EXPLORE: `home_node = current_node =` the grid-entry
       node (one-time initialization): on the travel axis the last line
       already crossed (floor of the DR coordinate for positive travel,
       ceil for negative), on the perpendicular axis the nearest line.
       Snapping to the nearest node on the travel axis indexed one cell
       ahead whenever the drone had not yet crossed the nearest line, and
       every later node update inherited the offset (r77).
     - MARKER_CONFIRM completion: the drone has spent 3 s centering on
       a marker that sits ON an intersection, so
       `current_node =` node nearest to the DR position, and the marker
       is recorded there. This both fixes the skeleton's off-by-one
       (it records at the last counted node even when the marker was
       spotted mid-edge) and re-zeros intersection-count drift at every
       marker.
- Double-count guard: MissionManager only accepts an intersection event
  if the previous one was released (perception sends pulses, section 5)
  — no time-based cooldown needed at this layer.

## 4. Exploration (serpentine, side camera deferred)

`ExplorationPlanner.choose_direction()` implements a boustrophedon
sweep by direction rules (no world-coordinate path):

- Sweep rows j = 0..max in ascending order; traverse x ascending on
  even visit, descending on odd (classic lawnmower).
- At a row end (next node in the travel direction is out of bounds):
  command Y_POS once, then reverse the x direction.
- Mission-complete condition unchanged (4 markers recorded). If the
  sweep exhausts the grid with markers missing, restart the sweep from
  the current node (loop until found; timeout/FAILSAFE is future work).

## 5. IntersectionDetection (new perception capability)

Contract (consumed by MissionManager):

- `detected` is a PULSE: true for exactly one result per physical
  crossing. Hysteresis: the crossing line (the grid line perpendicular
  to travel) must enter the image-center band (|offset| < enter_px)
  to fire, and leave the band (|offset| > exit_px, exit > enter) before
  the detector may fire again.
- `forward/left/right/backward` (relative to the current travel axis in
  the image) report whether line segments extend from the crossing in
  each direction, from the existing Hough segments.
- Implementation lives in `perception.py` (pure OpenCV/numpy, no
  rclpy), unit-tested on synthetic grid images, and validated against
  sim ground truth (counted crossings == truth crossings on a straight
  leg).

## 6. McuCommand

Dataclass fields (Python) and wire fields (C) are 1:1. Units are
metric and radians; the Jetson converts pixels to meters with
`altitude / f` before sending (the MCU knows no camera intrinsics).

| field | type | units / range | notes |
|---|---|---|---|
| mode | u8 | ControlMode + arm bit 0x80 | mirrors existing mode byte |
| mission_state | u8 | MissionState | telemetry/logging |
| seq | u8 | | wraps |
| node_x, node_y | i8 | grid index | telemetry/logging |
| move_direction | u8 | MoveDirection | selects travel axis |
| target_altitude | u16 | cm, 0..10 m | |
| line_dx | i16 Q14 | m, clamp +/-2.0 | vertical-line offset: signed body +y position of the nearest vertical grid line relative to the drone. Jetson mapping: -du*alt/fx. 0 when absent (see flags bit0). |
| line_dy | i16 Q14 | m, clamp +/-2.0 | horizontal-line offset: signed body +x position of the nearest horizontal grid line. Jetson mapping: -dv*alt/fy. 0 when absent (see flags bit1). |
| line_angle_error | i16 Q14 | rad, FLU +CCW | followed-line direction vs travel axis (Jetson selects the line by travel axis; angle only) |
| marker_error_x | i16 Q14 | m, body +x | -(v-cy)*alt/fy |
| marker_error_y | i16 Q14 | m, body +y | -(u-cx)*alt/fx |
| marker_yaw_error | i16 Q14 | rad | 0 for now (centering only) |
| vx_est, vy_est | i16 Q14 | m/s body | companion DR velocity, closes the MCU velocity loop; validity flag below |
| marker_id | i8 | -1 = none | |
| line_confidence | u8 | 0..255 | |
| marker_confidence | u8 | 0..255 | |
| flags | u8 | bit0 vertical_line, bit1 horizontal_line, bit2 intersection_detected, bit3 fwd, bit4 left, bit5 right, bit6 back, bit7 marker_detected | line presence per axis (the [dx, dy, flag] contract) |
| flags2 | u8 | bit0 vel_est_valid, bit1 emergency | rest reserved |
| speed_scale | u8 | percent 0..100 (values above 100 clamp to 100) | scales the MCU cruise for this command; 100 = full cruise. The mission slows legs that end in a stop or turn (section 7a). |

Line information is deliberately sent as the full [dx, dy, flag]
triple — both grid-line offsets plus per-line presence bits — rather
than a single travel-axis-selected lateral error. The MCU picks the
error for its FOLLOW_LINE law by move_direction (dx for +/-x travel,
dy for +/-y travel) and checks the matching presence bit; the Jetson
does no axis selection for offsets (angle_error is the one
travel-selected value, since it comes from the followed line's
orientation in the image).

Wire framing follows the existing protocol style: magic `0xA6`,
version, payload above, CRC16-CCITT trailer, little-endian, fixed
length. Exact byte offsets are defined once in `protocol.h` and are
the source of truth. The existing 24-byte setpoint downlink stays
(legacy path). Encode/decode symmetric in C; the Python side packs the
same layout (struct module) for the future serial link, and a ROS
message `fc_sim_msgs/McuCommand` mirrors the fields 1:1 for the sim.

## 7. mission_ctrl (fc_core, C — the MCU outer loop)

New `mission_ctrl.{h,c}`: pure C, no ROS, compiled into fc_core and
later the STM32 build. Per control tick:

```
fc_mission_tick(cmd,            /* latest decoded McuCommand        */
                meas,           /* altitude m, vz m/s, (vx,vy) body if valid */
                dt)             /* s */
    -> writes COMP (roll_sp, pitch_sp, yawrate_sp, thrust_norm, arm)
```

It ports the two Python laws verbatim (same gains and defaults as
`dead_reckoning.py`, except `kp_xy`, see below):

1. `compute_body_velocity`: v_along = cruise (MCU param) in the
   commanded MoveDirection axis; v_perp = kp_xy * line_lateral_error
   (toward the line); wz = kp_yaw * line_angle_error; clamps max_vxy,
   max_wz. ALIGN/LAND modes use kp_xy * marker_error_{x,y} instead.
2. `body_vel_to_atti_thr`: velocity-error P to roll/pitch when the
   velocity estimate is valid (else the open-loop v/g mapping),
   attitude clamp, altitude PD around hover_thrust_norm with the
   takeoff burst below takeoff_z_threshold, LAND descent on
   land_descent_vz with land_cutoff_alt thrust cut,
   yawrate_sp = -wz (FLU -> NED, the sign the whole fleet depends on).

`kp_xy` defaults to 0.2 on the MCU, not the Jetson's 0.8. The legacy
waypoint law pushed kp_xy * (distance to a waypoint ~3 m away) through
the max_vxy vector clamp, which scaled the lateral component down to an
effective gain of ~0.13-0.2; FOLLOW_LINE's constant cruise removes that
squeeze, so a raw 0.8 is a 4-6x stiffer lateral loop than the legacy
path ever flew and diverges against the attitude loop's response lag
(r77 weave). 0.2 restores the flown effective stiffness.

Mode behavior table:

| ControlMode | xy source | z target | notes |
|---|---|---|---|
| HOLD | zero velocity | target_altitude | |
| FOLLOW_LINE | cruise + lateral + angle | target_altitude | lateral error = line_dx for +/-x travel, line_dy for +/-y travel; missing presence bit for that axis -> HOLD behavior |
| ALIGN_MARKER | marker errors | target_altitude | marker_detected=0 -> HOLD (covers TAKEOFF climb) |
| SEARCH_LINE / MOVE_TO_LANDMARK | slow cruise in move_direction | target_altitude | |
| LAND_ON_MARKER | marker errors | descend (land law) | cutoff -> disarm |
| STOP | — | thrust 0, disarm | |
| EMERGENCY_LAND | zero velocity | descend (land law) | ignores errors |

Stale-command fallback: reuse the existing COMP_STALE_MS contract —
mission_ctrl only writes COMP when a fresh McuCommand exists; the
Control() stale fallback stays the safety net.

## 7a. Speed scheduling (Jetson side) and the front camera

The MCU applies effective_cruise = cruise * speed_scale / 100 in
FOLLOW_LINE / SEARCH_LINE / MOVE_TO_LANDMARK. The mission sets
speed_scale per leg (constructor-tunable defaults):

- transit legs (Y moves between rows) and the first leg after any
  settle: 40
- the final leg before a known row end (next node in the travel
  direction is a boundary node): 50
- a front-camera marker hint projected within hint_slow_range_m
  (default 4.0) ahead on the current row: 50
- otherwise: 100

Rationale: braking authority (attitude clamp + cascade lag) needs
~a full 3 m cell from 1.3 m/s, so speed must already be low wherever
a stop or turn can occur; straights carry the full cruise.

Front camera (IMX219 8MP 120 deg wide, front-mounted, 45 deg down;
sim: /front_camera/image + /front_camera/camera_info, 1024x768,
fx 295.6): HINTS ONLY, never records — the downward camera stays the
authoritative record path. Pipeline reuses the mount-agnostic
machinery in side_camera.py (MountExtrinsics yaw=0, pitch=pi/4,
tx=+0.08, tz=-0.03; project_pixel_to_ground; CandidateTracker voting
onto grid nodes). The node feeds the mission a per-tick hint
(marker id, node, ground distance ahead); the mission uses it only
for speed_scale and to anticipate MARKER_CONFIRM braking.

## 8. Sim wiring

- `fc_sim_msgs/McuCommand.msg` mirrors section 6.
- `fc_sim_node`: subscribe `/fc/mcu_command`, encode->decode through
  the new protocol frame (wire parity, as `onSetpoint` does), run
  `fc_mission_tick` each control tick with gz-truth altitude/vz/body
  velocity, write COMP.
- `line_tracer_node`: parameter `mission_backend` = `skeleton`
  (default) | `legacy`. Skeleton path builds PerceptionData +
  SensorData (battery/imu/lidar/rc stubbed healthy in sim), calls
  `MissionManager.step()`, publishes McuCommand. Legacy path is the
  existing FSM + Setpoint publisher, unchanged.

## 9. Testing

- Python mission layer: pure-unit (no ROS): grid, serpentine coverage,
  state transitions incl. FAILSAFE, marker confirm vote, off-by-one
  snap, BFS rescue path, double-count guard.
- Perception: synthetic-image tests for the intersection pulse +
  branch flags.
- fc_core: gtest roundtrip + CRC tamper for the new frame;
  mission_ctrl unit tests (mode table, takeoff burst, land cutoff,
  velocity-loop vs open-loop path, NED negation sign).
- End-to-end: `dev.sh mission` flying the skeleton backend in Gazebo.

## 10. Deviations from docs/mission_skeleton.py (documented on purpose)

1. `McuCommand.mode` string -> ControlMode IntEnum (serial link).
2. Grid default 9x6 (24x15 m) -> 11x8 (30x21 m), user-confirmed.
3. Marker recorded at the DR-snapped node, not blindly at
   `current_node`; current_node re-zeroed there (off-by-one fix).
4. ExplorationPlanner: keep-direction placeholder -> serpentine
   (the placeholder walks off the grid and add_edge raises).
5. Battery/RC failsafe thresholds kept, values stubbed healthy in sim.
6. `time.time()` -> injected `now` (sim time; testability).
7. print() -> injected logger.
8. Line info is the [dx, dy, flag] triple (both line offsets + per-line
   presence bits, user requirement) instead of the skeleton's single
   visible/lateral_error pair; the MCU selects by move_direction.
