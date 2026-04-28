# 작업 보고: world 패키지 (260429_world.md 구현)

세션 시작: 2026-04-29 새벽 (`349e255` line_tracer step3 commit 직후)
대상 instruction: `260429_world.md`

## 결론
260429_world.md 의 산출물 1~7 + 검증 1~5 모두 완료.
검증 6 (line_tracer 와의 폐루프) 는 **미수행** — line_tracer step 4~8
(perception, FSM, node, launch, build) 가 아직 안 되어 있어서 노드를 띄울 수 없다.
폐루프 검증은 line_tracer 마무리 후로 자연 미뤄짐.

## 산출 파일 (모두 git 미커밋)

```
src/world/
├── package.xml
├── CMakeLists.txt
├── script/
│   ├── grid_stl.py        # argparse 화 + grid 모드 정리
│   └── aruco.py           # 일괄 생성 + YAML 인자 지원
├── mesh/
│   ├── grid_30x20_t0.10_cell4.stl   # 신규 본 사양
│   └── grid_4x4_t0.1_cell0.5_beam0.05.stl  # 기존 디버그용 그대로 유지
├── textures/aruco_{0..8}.png         # 9 개 ArUco DICT_6X6_250 (512 px)
├── config/
│   ├── aruco_layout.yaml             # 9 marker pose
│   └── bridge.yaml                   # ros_gz_bridge 매핑
├── worlds/competition.sdf            # physics + 격자 + 마커 + 드론 include
├── models/
│   ├── uav26_quad/{model.sdf, model.config}   # X-frame quad + 하향 D435 + 가짜 FC
│   └── world_assets/{model.sdf, model.config} # model:// 해석용 자산 컨테이너
└── launch/sim.launch.py
```

기존 파일 보존 (변경 없음): `Dockerfile`, `compose.yml`, `PROGRESS.md`(추가만),
`src/line_tracer*`.

## 좌표/단위 합의 (260429_world.md §"좌표계/단위 합의" 와 일치)

| 항목 | 값 |
|---|---|
| World | ENU (`world` frame) |
| Body  | FLU. base_link 가 드론 중심 |
| 카메라 마운트 | base_link 기준 `(0, 0, -0.05)` (5 cm 하단) |
| 카메라 자세 | `<pose>0 0 -0.05 0 1.5707963 0</pose>` → Gazebo cam +X(view) = -Z_body |
| Optical frame_id | `camera_color_optical_frame` / `camera_depth_optical_frame` |
| 카메라 yaw 오프셋 | 0 (드론 +X = 이미지 v 위쪽) |
| 단위 | m, rad |

## 토픽 매핑 (config/bridge.yaml)

| ROS | ↕ | Gazebo |
|---|---|---|
| `/camera/camera/color/image_raw`              | ← | `/uav26_quad/camera/color/image` |
| `/camera/camera/color/camera_info`            | ← | `/uav26_quad/camera/color/camera_info` |
| `/camera/camera/aligned_depth_to_color/image_raw`   | ← | `/uav26_quad/camera/depth/image` |
| `/camera/camera/aligned_depth_to_color/camera_info` | ← | `/uav26_quad/camera/depth/camera_info` |
| `/imu`         | ← | `/uav26_quad/imu` |
| `/odom_truth`  | ← | `/model/uav26_quad/odometry` |
| `/cmd_vel`     | → | `/uav26_quad/cmd_vel` (gz.msgs.Twist) |
| `/clock`       | ← | `/clock` |

## 검증 결과

### `ros2 launch world sim.launch.py headless:=true`
백그라운드 실행 후 호스트에서 `ros2 topic list`:

```
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/clock
/cmd_vel
/imu
/odom_truth
/parameter_events
/rosout
```

### 토픽 hz / 데이터 정합

- `/clock` 500 Hz (= max_step_size 0.002 s 의 역수, 정확)
- `/camera/camera/color/image_raw` 30 Hz (model.sdf `<update_rate>30</update_rate>` 일치)
- `/camera/camera/color/camera_info`:
  - `width=640, height=480` ✓
  - `frame_id="camera_color_optical_frame"` ✓ (드론 model.sdf `<gz_frame_id>` 가 SDF 스키마 외 element 라 경고는 뜨지만 동작)
  - `K = [465.60, 0, 320; 0, 465.60, 240; 0, 0, 1]`
    → fx = fy = 465.60 (= 320 / tan(34.5°), H-FOV 69° 일치), cx=320, cy=240
  - `distortion = [0,0,0,0,0]` (ideal pinhole)

### 가짜 FC (cmd_vel → 드론 운동) 확인
```
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0.5}, angular: {z: 0}}'
```
4 초 동안 반복 publish 시 `/odom_truth` position.z 가 0.03 → 11.04 로 상승.
즉 cmd_vel ROS→GZ 브리지 + MulticopterVelocityControl 자세 PID + MotorModel 추력
체인이 모두 살아 있다. 검증 5 (드론이 cmd_vel 따라 이동) 통과.

### colcon build / test
- `colcon build --packages-select world` 0 errors
- `colcon build --packages-select line_tracer line_tracer_msgs world` 모두 통과
  (단, line_tracer_msgs 는 numpy 버전 변경 여파로 build cache 무효화 → 재빌드 필요했음)
- `colcon test --packages-select line_tracer` 32/32 통과

## 설계상 결정

### `<gz_frame_id>` 사용
SDF 표준 element 가 아니라 gz-sim 의 sensor 구현이 별도 읽는 필드.
파싱 시 "XML Element[gz_frame_id], child of element[sensor], not defined in SDF"
경고가 뜨지만 카메라 메시지의 `header.frame_id` 에 정상 반영됨 (위 camera_info echo 결과로 확인).

### `MulticopterVelocityControl` 의 `<robotNamespace>uav26_quad</robotNamespace>`
이 plugin 은 robotNamespace 가 비어있으면 `Please specify a robotNamespace.`
에러로 죽음. `MulticopterMotorModel` 은 entity name 으로 fallback (warn 만 뜸).

### `enable=true` 자동 publish
`MulticopterVelocityControl` 는 `<enableSubTopic>` 으로 Boolean=true 를
받기 전에는 비행을 시작하지 않음. `sim.launch.py` 가 시뮬 시작 3 초 뒤
`gz topic -t /uav26_quad/enable -p 'data: true'` 를 1 회 publish 하도록
TimerAction + ExecuteProcess 추가.

### Camera info 토픽 충돌 회피
처음 작성한 `<topic>uav26_quad/camera/color</topic>` 는 camera_info 가
parent dir 기준이라 color/depth 둘 다 `/uav26_quad/camera/camera_info` 로 충돌함.
`<topic>uav26_quad/camera/color/image</topic>` (one extra level) 로 바꿔
`/uav26_quad/camera/{color,depth}/camera_info` 로 분리.

### Grid mesh collision 제거
dartsim 은 SDF 의 `<mesh>` collision 을 미지원 (debug log: "Mesh construction
from an SDF has not been implemented yet for dartsim"). 격자선은 5 mm 두께
painted line 이라 충돌이 의미가 없으므로 visual 만 두고 collision 은 삭제.

### `model://world_assets/...` 패턴
설치 후 `share/world/{mesh,textures}/...` 에 둔 자산을 SDF 가 참조하려면
GZ_SIM_RESOURCE_PATH 가 필요. CMakeLists 가 mesh/STL 와 textures/PNG 를
`share/world/models/world_assets/{meshes,textures}/` 로 복사하고,
sim.launch.py 가 `GZ_SIM_RESOURCE_PATH += share/world/models` 를 set.
→ `model://world_assets/meshes/grid_30x20_t0.10_cell4.stl` 식으로 해석됨.

## 주의/한계

1. **컨테이너 numpy 상태 불일치 (사이드이펙트)**
   - `Dockerfile` 의 M (uncommitted) 변경분 `pip install --ignore-installed numpy` 가
     /usr/local 에 numpy 2.x 를 덮어 쓰는데, apt 에서 깔린 scipy / trimesh 는
     numpy 1.x 빌드라 ABI 충돌이 남.
   - 이번 세션에서 STL/PNG 생성을 위해 컨테이너 안에서
     `pip install --ignore-installed 'numpy<2'` 를 실행해 numpy 1.26.4 로 다운그레이드
     → trimesh/scipy 정상화 → STL/PNG 생성 성공.
   - 컨테이너 재빌드 시 다시 numpy 2.x 가 깔리며 trimesh 가 깨짐. STL/PNG 는
     이미 git 에 들어가므로 런타임 (ros2 launch) 영향 없지만, 자산 재생성이
     필요할 때마다 위 다운그레이드를 다시 해야 함.
   - 권장 영구 수정 (이번 세션에서는 안 함):
     ```dockerfile
     RUN pip install --break-system-packages --no-cache-dir --ignore-installed \
         'numpy<2' 'opencv-contrib-python<4.13' trimesh transforms3d
     ```
2. **GUI Gazebo 미검증**
   - 호스트 X 가 안 떠 있어 `headless:=true` 로만 검증함.
   - `ros2 launch world sim.launch.py` (기본 GUI) 는 X11 / NVIDIA 드라이버 OK 인
     환경에서 별도 확인 필요. SDF 자체는 GUI 모드에서도 동일하게 로드되므로
     큰 차이 없음.
3. **검증 4 / 6 미수행**
   - 4: `rqt_image_view` 로 격자 영상 확인 — GUI 미검증과 같은 이유.
   - 6: line_tracer 와 폐루프 — line_tracer step 4~8 완료 전이라 불가.
4. **마커 9 개 (vs md 의 7×5=35 표현)**
   - md 본문 "예: 7×5 = 35 교차점 전체 또는 5~10개 선택" 에서 후자 선택.
   - 30×20 m 셀 4 m 격자에서 7×5 정확히 35 가 안 떨어지는 점도 있어
     (interior intersection 7×4 = 28) 5~10 의 권장 범위만 충족.
   - 추가 필요 시 `config/aruco_layout.yaml` 에 항목 추가하고
     `python3 script/aruco.py --ids-from config/aruco_layout.yaml --out-dir textures`
     재실행 + `worlds/competition.sdf` 에 `<model name="aruco_N">` 블록 복붙하면 됨.
5. **Depth encoding**
   - Gazebo depth_camera → gz.msgs.Image (R_FLOAT32, m). bridge 가 32FC1 [m] 로 노출.
   - 실 RealSense 는 16UC1 [mm]. line_tracer perception step (Step 4) 에서
     이 차이를 감안해 m/mm 모두 핸들링하거나 use_sim_time 여부로 분기 필요.

## 다음 단계 권고

1. `260428.md` 의 line_tracer Step 4 (perception.py) 부터 진행.
   - 카메라→body 회전 = `Rz(0) ∘ Ry(π/2)` (yaw=0, pitch=+90° 하향) — 위에서 확정.
   - depth encoding 은 시뮬 32FC1 m / 실 16UC1 mm 모두 받게.
2. line_tracer Step 7 launch 에서 본 패키지 sim.launch.py 를 `IncludeLaunchDescription`
   으로 import → 한 번에 시뮬 + 라인추종 노드 + (선택) realsense2_camera 까지.
3. Step 8 통합 빌드 후 검증 6 (폐루프) 시도.
   - 시작 spawn pose 는 `(2, 2, 0.15, 0, 0, 0)` 로 박혀 있음 (격자 모서리 근처).
   - 처음 cmd_vel 은 `linear.z` 로 적당히 hover 시킨 후 line_tracer FSM TAKEOFF→LINE_FOLLOW.
4. `Dockerfile` numpy/opencv 핀 정정 (위 §주의 1 참고) — 재현성 위해.
