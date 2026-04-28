# UAV26 Progress

마지막 업데이트: 2026-04-29

## Done

### `world/` 패키지 — Gazebo Harmonic 시뮬 (instruction `260429_world.md`)
산출물 (이번 세션):
- `src/world/script/grid_stl.py` — argparse 기반 격자 STL 생성기
- `src/world/mesh/grid_30x20_t0.10_cell4.stl` — 30 × 20 m, 셀 4 m, 라인 폭 10 cm
- `src/world/script/aruco.py` — DICT_6X6_250 PNG 일괄 생성기
- `src/world/textures/aruco_{0..8}.png` — 9 개 마커 (512px)
- `src/world/config/aruco_layout.yaml` — 9 개 마커 좌표
- `src/world/worlds/competition.sdf` — physics + 조명 + 격자 + 마커 9 + 드론 include
- `src/world/models/uav26_quad/{model.sdf,model.config}` — X-frame quad,
  하향 D435 (color + depth) sensor, 4×motor model + multicopter velocity control
- `src/world/models/world_assets/{model.sdf,model.config}` — `model://` 해석용
- `src/world/config/bridge.yaml` — 카메라/cmd_vel/IMU/odom/clock 매핑
- `src/world/launch/sim.launch.py` — gz_sim include + parameter_bridge + auto-enable
- `src/world/{package.xml,CMakeLists.txt}` — ament_cmake 패키지 메타

검증:
- `colcon build --packages-select world line_tracer line_tracer_msgs` 통과
- `colcon test --packages-select line_tracer` 32/32 OK
- headless `ros2 launch world sim.launch.py headless:=true` →
  `/clock` 500 Hz, `/camera/.../color/image_raw` 30 Hz,
  `camera_info` fx/fy=465.6 cx/cy=320/240 frame_id=`camera_color_optical_frame`,
  `/cmd_vel` → 드론 z 상승 (가짜 FC 동작 확인)

상세 보고: `report_260429_world.md`



### `line_tracer_msgs/` (commit `b57e20f`, `c216f08`)
- ament_cmake 패키지
- `srv/SetState.srv` — FSM 상태 전이용 (string state → bool success + string message)

### `line_tracer/` 패키지 골격 (commit `b57e20f`, `c216f08`)
- ament_python 패키지
- `package.xml` deps: rclpy, sensor_msgs, geometry_msgs, nav_msgs, visualization_msgs, std_msgs, cv_bridge, tf2_ros, line_tracer_msgs
- entry point: `line_tracer_node = line_tracer.line_tracer_node:main`
- 디렉토리: `line_tracer/`, `launch/`, `config/`, `resource/`, `test/`

### Step 2 — `line_tracer/geom.py` (commit `d2bcdbe`)
- `CameraIntrinsics` dataclass + `.from_camera_info()` factory
- `pixel_offset_to_meters(du, dv, depth, intr)` — `Δx = du·d/fx`, `Δy = dv·d/fy`
- `pixel_to_principal_offset(u, v, intr)`
- 테스트 10개 통과

### Step 3 — `line_tracer/dead_reckoning.py` (commit `349e255`)
- `Gains`, `State`, `BodyVelocity` dataclasses
- `wrap_angle`, `clamp` 헬퍼
- `compute_body_velocity(dx_body, dy_body, psi_err, z_hat, gains)` — P 제어 + 클램프
- `integrate(state, vel, dt)` — yaw 회전 후 forward-Euler
- `DeadReckoning` orchestrator (state 보유, `step()`/`reset()`)
- 테스트 22개 통과 (총 32개)
- Frame: body=FLU(REP-103), world=ENU
- 카메라→body 회전은 모듈 외부 (호출자) 책임

### Docker / 빌드 환경
- `Dockerfile` numpy 1.x↔2.x ABI 충돌 해결 (`pip install --ignore-installed numpy`)
- 컨테이너 이름: `uav-aruco`
- `colcon build --packages-select line_tracer_msgs line_tracer` 통과
- `ros2 interface show line_tracer_msgs/srv/SetState` OK
- `ros2 pkg executables line_tracer` → `line_tracer_node` 등록됨

## Pending in `line_tracer/`

| Step | 산출물 | 비고 |
|---|---|---|
| 4 | `perception.py` | Canny+HoughLinesP 격자선, ArUco DICT_6X6_250, 디버그 오버레이. **카메라→body 회전 정의 필요 (world 패키지와 합의 후)** |
| 5 | `state_machine.py` | TAKEOFF/LINE_FOLLOW/LAND 실작동, WAYPOINT_VISIT/ARRANGE_BY_ID/RETURN_PATH는 stub |
| 6 | `line_tracer_node.py` | rclpy 노드, image cb + DR timer + set_state 서비스 |
| 7 | `launch/line_tracer.launch.py` + `config/params.yaml` + README | realsense2_camera include |
| 8 | docker 안에서 통합 빌드/테스트 | colcon test 전체 통과 |

## Future Work (line_tracer 외부)

### `world/` 패키지 — Gazebo 시뮬 환경
**왜 필요**: 실물 드론 하드웨어 없음 → 폐루프 검증 유일 경로가 Gazebo. 상세 instruction은 `260429_world.md` 참조.

핵심 deliverable:
- 30×20m 격자 STL/SDF (현 `grid_stl.py` 확장)
- ArUco 마커 SDF/텍스처 배치 (현 `aruco.py` 확장)
- `world.sdf` (조명, 바닥, 마커)
- 드론 모델 (X-frame, 하향 D435 sensor 플러그인)
- `MulticopterVelocityControl` 또는 동등 — `/cmd_vel` 받아 thrust 적용
- `ros_gz_bridge` 런치 (gz camera → ROS Image)
- `world/launch/sim.launch.py`

### 자작 비행제어기 (별도 트랙, 본 repo 외)
- STM32 (또는 동급 MCU), C
- 자세 PID, IMU 융합, 모터 PWM
- companion ↔ MCU 프로토콜 미정 (MAVLink vs 자체)
- companion 컴퓨터 하드웨어 미정 (RPi5 / Jetson / 노트북)

### `fc_adapter/` 패키지 (사양 결정 후)
- ROS2 노드: `/cmd_vel` 구독 → 시리얼 프레임 송신
- 가짜 FC 모드: Gazebo 시뮬에서 `MulticopterVelocityControl` 로 대체 가능 (별도 노드 불필요할 수도)
- 실제 FC 모드: UART 프레임 송수신

## 미결 의사결정

| # | 결정 항목 | 막힌 이유 |
|---|---|---|
| 1 | `set_state` srv 타입 | `line_tracer_msgs/SetState`로 결정 ✓ |
| 2 | line_tracer 1차 범위 | TAKEOFF/LINE_FOLLOW/LAND 실작동, 나머지 stub ✓ |
| 3 | `/cmd_vel` 프레임 | FLU(REP-103) ✓ — 변경 시 `dead_reckoning.py` 부호 검토 필요 |
| 4 | 카메라 마운트 yaw offset | world 패키지에서 드론 모델 확정 후 결정 |
| 5 | companion 컴퓨터 | 미정 (Jetson/RPi/노트북) |
| 6 | companion ↔ STM32 프로토콜 | 미정 (MAVLink/자체) |
| 7 | Gazebo world가 본 repo에 포함? | 예 (계획) — `src/world/` 확장 |
| 8 | 가짜 FC: Gazebo 플러그인 vs 별도 ROS 노드 | `MulticopterVelocityControl` 플러그인 권장 (코드 0줄) |

## Re-entry guide

다음 세션에서 이 작업을 이어받으려면:
1. `git log --oneline` 으로 step1~3 commit 확인
2. `260428.md` — 본 task 사양
3. `260429_world.md` — Gazebo world 구축 instruction
4. `PROGRESS.md` — 본 파일
5. `docker compose up -d && docker exec uav-aruco bash -lc "cd /workspace && colcon test --packages-select line_tracer"` 로 환경 확인
