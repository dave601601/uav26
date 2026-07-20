# uav26

Autonomous grid-survey drone for the 2026 competition. A Jetson-class
companion runs vision and the mission state machine; an STM32 flight
controller (shared source in `fc_core`) runs every control loop.

```
Jetson (Python, line_tracer)                MCU (C, fc_core / STM32)
┌──────────────────────────────┐            ┌──────────────────────────┐
│ cameras -> PerceptionData    │  McuCommand│ fc_proto_decode_mission  │
│ sensors -> SensorData        │ ──────────>│ fc_mission_tick (outer   │
│ MissionManager.step()        │  34-byte   │  loop) -> COMP setpoints │
│   node-based navigation      │  frame     │ Control() att/rate/mixer │
└──────────────────────────────┘            └──────────────────────────┘
```

Full contract (units, signs, byte offsets, mode table, deviations):
`docs/MISSION_INTERFACE.md`. Work log: `docs/PROGRESS.md`.

## The interface, in two files

| side | file | what it holds |
|---|---|---|
| Jetson | `src/line_tracer/line_tracer/mission_interface.py` | the whole contract: `MissionState` / `ControlMode` / `MoveDirection` enums (fixed values), `Node`, `PerceptionData` / `SensorData` inputs, `McuCommand` output, direction helpers, `pack_mcu_command()` serial packer |
| MCU | `src/fc_core/include/fc_core/protocol.h` + `mission_ctrl.h` | the 34-byte wire table (source of truth), decode/apply, the outer-loop entry `fc_mission_tick`, gains |

A golden frame is pinned byte-for-byte in `test_mission_interface.py`
(Python pack) and `test_protocol.cpp` (C decode) — change together.

## Frequently used (Jetson side)

| symbol | use |
|---|---|
| `MissionManager(grid_map, logger, ...)` | the mission state machine; all thresholds are constructor args |
| `MissionManager.step(now, sensors, perception) -> McuCommand` | call once per perception tick; runs the FSM and builds the command |
| `MissionManager.set_front_hint(id, node, dist_m)` | feed a front-camera marker hint (slows the approach); `None` id clears |
| `pack_mcu_command(cmd, seq=, arm=) -> bytes` | the exact 34-byte frame for the STM32 UART (Q14, CRC16-CCITT) |
| `SensorData(altitude, ..., dr_x, dr_y, vx_est, vy_est)` | DR position feeds the snap corrections and lost-line recovery; velocities close the MCU velocity loop |
| `mission_adapter.py` functions | pure pixel->metric conversions and front-hint selection (no ROS) |
| `perception.IntersectionDetector.update(...)` | one pulse per grid crossing + branch flags (node counting depends on it) |

Enums are `IntEnum` with wire-fixed values — log with `.name`, send as
`int`. `McuCommand` field units are SI (m, rad, m/s); the packer owns
all quantization (cm, Q14, u8 confidence).

## Frequently used (MCU side)

| symbol | use |
|---|---|
| `fc_proto_decode_mission(buf, &msg)` | validate magic/version/CRC of a received 34-byte frame |
| `fc_proto_apply_mission(&msg, now_ms)` | store into the `MISSION` global with a freshness stamp |
| `fc_mission_tick(&MISSION.cmd, &meas, dt)` | run the outer loop for one control tick; writes `COMP` |
| `fc_mission_meas_t` | what the MCU must measure: altitude (lidar), vz, body vx/vy if available (`vel_valid`) |
| `fc_mission_gains` / `fc_mission_gains_default()` | outer-loop gains; defaults are the sim-flight-proven values |
| `COMP` / `Control()` | unchanged pre-existing cascade; `mission_ctrl` only writes `COMP` |

## Wiring the STM32 (recipe)

1. UART RX (companion link): accumulate 34 bytes aligned on magic
   `0xA6`, call `fc_proto_decode_mission`; on success
   `fc_proto_apply_mission(&msg, now_ms)`. Bad CRC -> drop, resync.
2. In the existing control tick, before `Control()`: fill an
   `fc_mission_meas_t` (lidar altitude, filtered vz, body-velocity
   estimate if any) and, while `MISSION.last_ms` is fresh
   (`< 300 ms`), call `fc_mission_tick(&MISSION.cmd, &meas, dt)`.
   Stale link falls back to `Control()`'s existing COMP-stale descent.
3. Nothing below `COMP` changes: the same `Control()` attitude/rate
   cascade and mixer run as before.
4. On the Jetson: build `McuCommand` via `MissionManager.step()` and
   send `pack_mcu_command(cmd, seq=n)` over the UART at the
   perception rate (10-15 Hz). The uplink (MCU -> Jetson telemetry)
   still uses the pre-existing 40-byte frame; a mission-aware uplink
   is future work.

## Running the sim

- `scripts/dev.sh mission rNN 1500` — headless full mission (Gazebo,
  seed 42), FSM summary printed, logs in `build/sweep_logs/mission/`.
- `scripts/dev.sh gui` — Gazebo window + tracer + detection overlays.
- Gain experiments without recompiling: 5th arg, e.g.
  `scripts/dev.sh mission r99 900 "" "mission_cruise:=0.5"`.
- Tests: `colcon test --packages-select fc_core line_tracer` in the
  dev container (see `docs/progress/docker.md` for container traps).
