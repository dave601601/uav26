# world

Gazebo Harmonic simulation package: arena, drone model, sensor plugins, bridge, launch.

## Done (recent)

### Marker polarity corrected to the official spec (2026-07-09)

The survey notice specifies "색상 : (바탕) 검정색, (마커) 하얀색" — the
0.4 m sheet is BLACK and the marker is WHITE. The world had it inverted
(white sheet, black code). `aruco.py` now negates the code and paints it
on a black canvas; all 50 textures regenerated.

This is not cosmetic. OpenCV forms ArUco candidate quads only from dark
regions with a dark border ring, so a white-on-black marker is invisible
to the default detector (verified: 0 detections). Both cameras now
negate the grayscale once before detection (`aruco_white_on_black`),
which restores a standard marker AND lifts the grass above the
threshold. Grass was previously a legal candidate quad — dark region,
dark border — and that is how r73 recorded a phantom id=17 on a bare
grid crossing. See [line_tracer](line_tracer.md).

The 1-module quiet zone stays, with its polarity flipped: the black
sheet is now the dark ring isolating the white code, sitting on the
white grid lines. That is a better arrangement than the white-sheet era,
where the sheet's white margin merged with the white ribbons.

### Official survey spec respec (2026-07-09)

The 2026-07 survey notice re-specifies the mission area; the world
package now models it:

- Floor: green GRASS texture (color floor_tex.py pipeline; gz PBR
  renders the green albedo darker than its luma — texture noise kept
  visible via higher contrast, residual brightness gap noted as a
  cosmetic-only delta) with WHITE 10 cm satin-ribbon lines on 3 m
  cells. Lines are baked continuous (no marker exclusion gaps — the
  opaque marker sheets cover the ribbons, matching reality).
- Arena 30 x 21 m (ASSUMPTION until the rules confirm; 3 m-divisible,
  closest to the previous scale). Floor plane, pose and spawn moved;
  spawn (2, 3, 3) = first interior row, preserving the ascending-sweep
  "+Y unexplored" invariant for the side camera.
- Markers: 0.4 m sheets (BLACK bg / white marker per the spec, 1-module quiet zone —
  code 0.3 m), placed on interior vertices only; marker_randomize.py
  samples 4 unique IDs from 0..49 (rules) as well as positions.
  aruco.py gains --dict (default 4X4_50 — "IDs 0..49" exactly matches
  the 50-marker dictionaries; ASSUMPTION until the rules name one) and
  all 50 textures are committed.
- Lookahead sensor mount deepened 22 -> 26 deg (0.4538): on 3 m cells
  the adjacent row (depression 33.3 deg) sat exactly ON the old VFOV
  band edge; the new band covers lateral 2.64..7.56 m with ~4 deg
  margins to both the +3 m and +6 m rows.
- Verified: held-pose render test 10/10 side detections of a marker
  one cell over (pixel row matched the 26-deg prediction); r70 full
  mission ran the whole candidate pipeline on the new spec (see
  line_tracer.md for the honest r70/r72 A/B).

### Marker textures gain the ArUco quiet zone (2026-07-09)

- aruco.py generated the code flush to the texture edge — no white
  quiet zone. A marker on a grid intersection therefore FUSES with
  the black grid cross into one blob and the detector cannot form a
  candidate quad. The nadir camera survived by scale luck (adaptive
  threshold separates the thin lines at its resolution); every
  oblique side-camera detection in r61/r63 died this way — verified
  by a held-pose render test: 0/10 detections without the margin,
  10/10 with it (id 1 from lateral 4.5 m, the row-skip design point).
- Textures are now a 1-module white margin around the code (plate
  stays 0.5 m; code 0.4 m — how a real 0.5 m sheet is printed; the
  detector's documented requirement is a light quiet zone). Nadir
  detection keeps ~12 px/module and also benefits from the margin.

### Sideways lookahead camera — OV9281 + 6 mm model (2026-07-08)

- New `lookahead` camera sensor on `uav26_quad`: boresight = +Y_body
  depressed 22 deg (`pose 0 0.05 -0.03 0 0.384 1.5708`; SDF rpy
  composes Rz(yaw)*Ry(pitch), same convention the downward cam uses
  for straight-down). HFOV 0.6196 rad = 35.5 deg — the real OV9281
  (3 um px) behind a 6 mm lens. Purpose: while the serpentine sweeps
  row y, this camera observes every intersection of row y+4 (lateral
  4 m, depression 26.6 deg), so the sweep can skip every other row
  (M-D). Sideways beats forward because yaw is locked to +X: on -X
  legs the drone flies backward and a forward camera would stare at
  already-swept ground, while +Y always faces the unexplored side of
  an ascending sweep.
- Resolution 640x400 (f = 1000 px), HALF the real sensor, for RTF:
  full 1280x800@15 halved host RTF (0.674 -> 0.366 measured on the
  idle-spawn scene); 640x400@10 lands at 0.533. The load-bearing
  +4 m row keeps ~6.3 px/module (detects fine); only the
  opportunistic +8 m band is lost. All nodes run on sim time, so RTF
  affects wall-clock only. L8 bridges end-to-end as mono8 with
  fx = 999.7 in camera_info.
- `bridge.yaml`: `/camera/lookahead/image_raw` + `/camera/lookahead/camera_info`.
- Verified headless (seed 42): 640x400 mono8 frames at the scaled
  rate, floor + grid line visible at the expected oblique angle.

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

- Add a downward single-beam range sensor (`gpu_ray` or `altimeter`)
  publishing `/uav26_quad/range` for the firmware's ESKFz (the sim
  currently substitutes /odom_truth for altitude).
- Confirm arena dims + ArUco dictionary against the rules; both are
  parameterized (competition.sdf sizes / floor_tex args; aruco.py
  --dict + line_tracer aruco_dict) so the swap is mechanical.
- Satin-ribbon specularity is not modeled (white albedo only) —
  hardware-day risk, not a sim item.

(The 2026-04-era list — fc_sim_node replacing the fake FC, firmware
geometry in model.sdf, motor_speed bridge, marker_randomize.py,
runtime-SDF injection — all landed; see Done entries.)

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
