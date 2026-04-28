# 작업 보고: line_tracer Step 4–8 (260428.md 마무리)

세션: 2026-04-29 새벽 (`world` 패키지 commit `4aec491` 직후)
대상 instruction: `260428.md` (Step 4 perception ~ Step 8 build/test)
선행 의존: `world` 패키지 (`08239dd`) — 카메라→body 회전 합의가 풀렸기 때문에 이번 step 4 가 풀림

## 결론
260428.md 의 Step 4 ~ 8 모두 완료. line_tracer 패키지가 perception, FSM,
node 까지 살아 있으며 world 시뮬과 묶어 폐루프가 돈다. 모든 단위 테스트
71/71 통과. 통합 검증으로 `/line_tracer/pixel_error` override → `/cmd_vel`
의 부호와 크기가 설계값과 일치함을 확인.

## 산출물 (이번 세션)

| 파일 | 라인수 | 기능 |
|---|---|---|
| `src/line_tracer/line_tracer/perception.py` | ≈ 300 | Canny+HoughLinesP, vert/horiz 분류, du/dv/psi_err, ArUco DICT_6X6_250, debug overlay |
| `src/line_tracer/line_tracer/state_machine.py` | ≈ 130 | StateName enum, Behavior, StateMachine, set_state |
| `src/line_tracer/line_tracer/line_tracer_node.py` | ≈ 320 | rclpy 노드 — 카메라/override → DR → cmd_vel/odom_dr/markers/debug_image |
| `src/line_tracer/test/test_perception.py` | ≈ 200 | 22 test (geometry helpers, sign convention, 합성 영상 검출, ArUco) |
| `src/line_tracer/test/test_state_machine.py` | ≈ 100 | 17 test (parse, behavior 매핑, transitions, override) |
| `src/line_tracer/launch/line_tracer.launch.py` | ≈ 90 | sim/real 분기, params YAML, target_altitude override |
| `src/line_tracer/config/params.yaml` | ≈ 25 | Kp들, perception 파라미터 |
| `src/line_tracer/README.md` | ≈ 175 | 토픽/파라미터 표, frame 약속, FC 핸드오프 가이드 |

## 핵심 설계 결정

### 카메라 → 바디 회전 (드디어 정의)
하향 카메라 SDF `<pose>0 0 -0.05 0 +π/2 0</pose>` (yaw=0). 광학 좌표:
- `+Z_optical` = `-Z_body` (광축, 수직 하향)
- `+Y_optical` (image v down) = `-X_body` (드론 후방)
- `+X_optical` (image u right) = `-Y_body` (드론 우측)

→ 노드의 픽셀→body 변환:
```
dx_body = -dv * altitude / fy   # image v 아래 ⇒ 드론 후방
dy_body = -du * altitude / fx   # image u 오른쪽 ⇒ 드론 우측
psi_err  passes through
```

### Pixel 오차 부호 약속 (perception ↔ DR 계약)
- `du > 0` ⇒ 라인이 image 우측 ⇒ 드론은 라인 좌측 ⇒ `vy < 0` 으로 `-y_body` 이동
- `dv > 0` ⇒ 라인이 image 아래 ⇒ 드론이 라인 통과 ⇒ `vx < 0` 으로 후진
- `psi_err > 0` ⇒ `+wz` (CCW from above) 로 body forward 를 라인에 정렬

`perception.py` 가 이 약속으로 출력하고, `line_tracer_node._on_dr_tick`
이 위 변환을 적용한 뒤 `dead_reckoning.compute_body_velocity` 가 P 게인으로 계산.

### 외부 override 채널
`/line_tracer/pixel_error` (geometry_msgs/Vector3 = `(du, dv, psi_err)`).
TTL = 0.5 s 동안 perception 보다 우선. md verification 의 "픽셀 오차 강제
입력" 그대로 호환. 파라미터 `external_override_ttl` 로 조정.

### FSM Behavior 패턴
각 state 가 `Behavior` 데이터클래스 (target_altitude, use_lateral_error,
use_heading_error, use_forward_error, cruise_vx) 를 보유.
- TAKEOFF: target=2.0, 모든 error 무시 → 위로만 climb
- LINE_FOLLOW: 모든 error 사용 + cruise_vx=0.4 m/s (라인 못 찾을 때 fallback)
- LAND: target=0.0, 모든 error 무시 → 내려가기만
- WAYPOINT_VISIT / ARRANGE_BY_ID / RETURN_PATH: 현재 LINE_FOLLOW 와 동일 (스텁)

스텁 상태는 향후 planner 추가 위치만 명시한 것 — 단위 테스트로 이 사실을 못박음.

## 검증 (md §검증 항목별)

### `colcon test` 전체 통과
```
$ pytest test/ -q
71 passed in 0.11s
```
breakdown: geom 10 + dead_reckoning 22 + perception 22 + state_machine 17.

### `ros2 launch line_tracer line_tracer.launch.py use_sim_time:=true`
world sim 옆에서 띄움. 노드 startup 로그:
```
[line_tracer_node] line_tracer_node up (state=TAKEOFF, target_alt=2.0, dr_dt=0.05)
[line_tracer_node] camera_info: fx=465.60 fy=465.60 cx=320.00 cy=240.00 size=640x480
```
ROS topic list:
```
/camera/camera/...           ← world 측
/cmd_vel                     ← line_tracer 발행 (overrides world 의 sub)
/line_tracer/debug_image
/line_tracer/pixel_error
/odom_dr
/odom_truth                  ← world 측 (ground truth)
/waypoints/aruco
```

### TAKEOFF 자동 climb (alt 0 → 2 m)
시간별 alt: 0.41 → 1.20 → 1.72 → 1.94 → 2.01 → 2.00 (정착).
이후 `vz` ≈ 0 으로 정지호버. 즉 노드가 publish 한 `/cmd_vel.linear.z` 를
가짜 FC (MulticopterVelocityControl) 가 받아 추력으로 변환 → 카메라 깊이로
다시 alt 측정 → DR 닫힘. ✓

### `/line_tracer/pixel_error` 강제 입력 → `/cmd_vel` 의도대로 변화
LINE_FOLLOW 상태에서 (du=80, dv=-100, psi_err=0.1) 12 회/초 publish:
```
[LINE_FOLLOW/external] du=80.0 dv=-100.0 psi_err=0.1 alt=2.00
                       vx=+0.34 vy=-0.28 vz=-0.00 wz=+0.10
```
계산식 검증:
- `dx_body = -(-100)*2.00/465.6 = +0.430` → `vx = 0.8*0.430 = 0.344` ✓ (+0.34)
- `dy_body = -80*2.00/465.6 = -0.344` → `vy = 0.8*-0.344 = -0.275` ✓ (-0.28)
- `psi = 0.1` → `wz = 1.0*0.1 = 0.10` ✓
- `vz = 0.6*(2.0-2.0) = 0` ✓

부호와 크기 모두 일치. md §검증 5 통과.

### `/line_tracer/debug_image` 시각화
`process_image` → `draw_debug_overlay` → `cv_bridge` → publish 루프 살아있음.
`ros2 topic hz /line_tracer/debug_image` ≈ 30 Hz. (rqt_image_view 시각 확인은
GUI 미설치 — 토픽 트래픽으로 동작 확인.)

## 빌드 / 의존성 메모
- 컨테이너 numpy 1.26.4 (이전 commit `4aec491` 의 Dockerfile 핀과 일치).
  `--ignore-installed numpy` 가 다시 설치되며 line_tracer_msgs 의 rosidl_generator_py
  build cache 가 numpy 2.x 기반이면 헤더 경로 불일치로 실패 — 최초 1 회 clean rebuild
  필요 (`rm -rf build/line_tracer_msgs install/line_tracer_msgs && colcon build`).
- 새 deps 없음. line_tracer/package.xml 변경 없음.

## 폐루프 거동 (관찰만, 튜닝은 다음)
TAKEOFF 후 LINE_FOLLOW 로 전환하면 perception 이 격자선을 잡고
다음 거동:
1. cruise_vx=0.4 m/s 로 +x_body 진행
2. 격자선이 image 안에 들어오면 du/dv/psi_err 으로 보정
3. 노이즈 프레임에서 cmd_vel 잠깐 튀는 케이스 관찰 (HoughLinesP 가
   영상 가장자리에서 가짜 선을 잡는 경우)
4. 본격 line tracking 정상화 / hover 정밀도는 Kp 튜닝 + Hough 파라미터
   재선택 작업이 별도 필요 (이번 세션에서는 정상 동작 가능성만 확인)

## 비목표 (md §"지금 하지 마라" 준수)
- ArUco 재방문 경로 최적화 — 미구현. `/waypoints/aruco` 발행만 함.
- IMU/EKF 융합 — 미구현. `/odom_dr` 는 순수 dead-reckoning.
- 모터 PWM — 미구현. `/cmd_vel` 까지만.

## 다음 단계 권고

1. **튜닝 세션** — Kp_xy, kp_yaw, Hough threshold/min_line_length 를 sim
   런에서 반복 조정. `report_time` 같은 잡 파일 만들어 sweep 결과 저장.
2. **WAYPOINT_VISIT 구현** — perception 이 발행한 ArUco markers 를
   기억하고 `set_state(WAYPOINT_VISIT)` 시 미방문 목록을 따라 이동.
3. **RETURN_PATH** — 시작점 (sim 의 spawn pose `2,2,0.15`) 을 기억해
   복귀.
4. **`fc_adapter/` 패키지** — 실제 STM32 정해지면, `/cmd_vel` 구독 →
   UART 프레임. line_tracer 코드는 손대지 않음.
5. **GUI 검증** — DISPLAY 가 있는 호스트에서 `ros2 launch world sim.launch.py`
   기본 (gui=true) + `rqt_image_view /line_tracer/debug_image` 로 격자/마커
   오버레이 직접 확인.
6. **README 의 LINE_FOLLOW 정착 시나리오 보강** — 위 #1 튜닝 결과를 들고
   "이 파라미터로 X 초 안에 ±Y px 안 정착" 같은 수치 명세 추가.

## Commit
이 세션의 작업 commit (예정): `<hash>` `line_tracer: implement steps 4-8`
