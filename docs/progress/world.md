# world

Gazebo Harmonic simulation package: arena, drone model, sensor plugins, bridge, launch.

## Done (recent)

### 4S motor model + four-feet contact geometry (2026-07-08)

- Motor plugins: maxRotVelocity 800 -> 1050 rad/s, motorConstant
  8.55e-06 -> 8.0e-06. k_f x w_max^2 = 8.82 N ~= 900 gf per motor —
  the 2212-920KV / 4S / 9450-prop operating point. Must stay in sync
  with fc_core `max_thrust_g_per_motor` and fc_sim defaults.
- Contact: the single 5 cm CoM sphere made the drone a ball — one
  contact point, no support polygon (tipped over landing motors-off,
  r49), zero rolling resistance (disarmed drone rolled 6 m after a
  clean touchdown, r53). Replaced with four r=0.02 sphere feet at the
  arm tips with explicit mu=1.0; contact height unchanged (-0.05 m).
  r54/r55: drone stays parked within 2 cm after touchdown.
- Spawn stays airborne at 3.0 m and the comment now records the r45-r49
  experiments as constraints: resting spawn explodes DartSim (rotor
  spin-up in ground contact -> "ODE INTERNAL ERROR 1 ... aabbBound"),
  short-hop spawn lands motors-off before the IMU sanity gate opens
  and tips over, and the sim->tracer launch gap must stay ~8 s or the
  FSM engages mid-air above takeoff_alt_threshold and skips TAKEOFF.

## Planned (Open)

- Replace `MulticopterVelocityControl` (fake FC accepting `Twist`) with `fc_sim_node` driven by Setpoint + actuator outputs. Drop the `enable_fc` TimerAction.
- Update `model.sdf` to firmware geometry: mass 1.5 -> 1.182 kg, rotor poses ±0.14 -> (±0.183, ±0.168), inertia rescaled.
- Add a downward single-beam range sensor (`gpu_ray` or `altimeter`) publishing `/uav26_quad/range` for the firmware's ESKFz.
- `bridge.yaml`: drop `/cmd_vel`, add `/uav26_quad/command/motor_speed` (actuator_msgs/Actuators) and `/uav26_quad/range`.
- `marker_randomize.py`: pre-launch script picks 4 unique grid intersections in the 30x20 arena, writes `models/world_assets/markers_runtime.sdf` (gitignored) and overwrites `config/aruco_layout.yaml`. Reproducible via `--seed`.
- `competition.sdf`: replace the inline 9-marker block with `<include>` of the runtime SDF.

## Done

### Drone model, arena, bridge, launch (commit `08239dd`, 2026-04-29)

산출물:
- `src/world/script/grid_stl.py` — argparse 기반 격자 STL 생성기 (later replaced by `floor_tex.py` PNG-bake)
- `src/world/mesh/grid_30x20_t0.10_cell4.stl` — 30 × 20 m, 셀 4 m, 라인 폭 10 cm (since removed in favor of the floor-tex bake)
- `src/world/script/aruco.py` — DICT_6X6_250 PNG 일괄 생성기
- `src/world/textures/aruco_{0..8}.png` — 9 개 마커 (512 px)
- `src/world/config/aruco_layout.yaml` — 9 개 마커 좌표
- `src/world/worlds/competition.sdf` — physics + 조명 + 격자 + 마커 9 + 드론 include
- `src/world/models/uav26_quad/{model.sdf,model.config}` — X-frame quad, 하향 D435 (color + depth) sensor, 4× motor model + MulticopterVelocityControl
- `src/world/models/world_assets/{model.sdf,model.config}` — `model://` 해석용
- `src/world/config/bridge.yaml` — 카메라/cmd_vel/IMU/odom/clock 매핑
- `src/world/launch/sim.launch.py` — gz_sim include + parameter_bridge + auto-enable
- `src/world/{package.xml,CMakeLists.txt}` — ament_cmake 패키지 메타

검증:
- `colcon build --packages-select world line_tracer line_tracer_msgs` 통과
- headless `ros2 launch world sim.launch.py headless:=true` -> `/clock` 500 Hz, `/camera/.../color/image_raw` 30 Hz, `camera_info` fx/fy=465.6 cx/cy=320/240, `/cmd_vel` -> 드론 z 상승 (가짜 FC 동작 확인)

Detail report: `report_260429_world.md` (in repo root).

### Floor refactor (uncommitted as of 2026-04-30 session)

Replaced the `grid_30x20` STL mesh with a baked floor PNG (rough concrete + grid lines + 50×50 cm gaps at marker positions). New `script/floor_tex.py` generator + `textures/floor.png` (3000×2000, 100 px/m). Removed `grid_stl.py` and old STL meshes. Drone spawn moved from `(2, 2, 0.15)` -> `(2, 4, 0.15)` so body +X aligns with the y=4 grid line. (Note: this work is mentioned in the auto-memory but may have already landed in a later commit — check `git log` for `floor_tex` before re-applying.)

## Decisions

- Floor texture is baked single PNG (not a separate STL mesh) — fewer draw calls, no z-fight, marker exclusion zones blanked of grid lines in the same pass.
- Camera optical frame composed as `pose=(0,0,-0.05) rpy=(0, π/2, 0)` so -Z body is the look direction. D435 mesh visual uses `rpy=(π, 0, π/2)` to align the part visually.
- Inertia rescaled proportionally to mass when geometry parameters change; revisit if attitude oscillation appears in Tier B.
