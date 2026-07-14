# fc_core

Pure C library carrying the STM32 firmware controller into ROS2 land.

## Done

### Comments rewritten for outside readers + known-bug warnings (2026-07-14)

Comments-only pass, no code changes (verified by stripping comments and
comparing byte-identical against the previous revision; full test suite
green). controller.c constant rationales lose the internal run numbers
and work-log pointers but keep the physics: the yaw-rate limit bump,
the deadband 1/3 rescale, and the F2PWM thrust-stand recalibration
warning. Three known traps that carried no in-code warning are now
marked KNOWN BUG at the function: `Allocation()`'s roll/pitch
arm-length swap (~9 % asymmetry), `quat_to_euler()`'s sign-flipped
pitch (both already tracked as deferred firmware items), and
`GetAngle2Vec()` returning uninitialized y/z — newly noticed in this
pass; dead code, no callers in this repo, must be fixed before use.
The other 14 fc_core files were already jargon-free and untouched.

### Thrust scale switched to the real power train: 900 g/motor, 2212-920KV on 4S (2026-07-08)

`Control()`'s thrust mapping assumed 600 g max thrust per motor (the
same motor on 3S). The real vehicle flies 2212 920KV motors on a 4S
pack (9450/1045-class props, bench range 800-1000 g/motor), so
thrust_norm=1.0 now commands 4 x 900 gf = 35.3 N; hover for the
1.182 kg frame sits at norm ~0.33 (r52 measured 0.334 in a clean
run). `COMP_STALE_THRUST_NORM` rescaled 0.40 -> 0.27 to stay just
below the new hover. Deviates from the verbatim-copy rule alongside
the earlier maxratecmd bump; both must be ported back to the embedded
repo, and `F2PWM`'s thrust-to-DSHOT curve is still calibrated for the
600 g train — needs a thrust-stand recalibration before hardware
flight.

### Firmware port + companion protocol + Tier A tests (commit `6eb7612`, 2026-05-25)

Teammate delivered the actual STM32G431 flight-controller firmware (`Core (1).zip`).
Goal: share the same controller source between sim (Gazebo) and real hardware,
so tuning, dynamics, and the companion link converge on one code path.

산출물:
- `src/fc_core/package.xml`, `CMakeLists.txt` — ament_cmake C library, no ROS deps.
- `include/fc_core/{linalg,controller,filter,planner,sbus,imu,lidar,protocol}.h` — HAL-less ports of the firmware headers (planner.h drops `stm32g4xx_hal.h`, filter.h drops `main.h`).
- `src/{linalg,controller,filter,planner}.c` — verbatim from firmware (`/tmp/uav26_core_fw/Core/lib/{src,inc}`).
- `src/{imu_parse,sbus_parse,lidar_parse}.c` — pure parse halves of the firmware drivers; HAL UART/DMA init dropped, caller supplies a pre-aligned frame buffer.
- `src/protocol.c` — companion <-> FC binary codec. 24-byte downlink (mode + arm + roll/pitch/yawrate/vz setpoints in Q14 + thrust_norm in Q15 + timestamp + flags + CRC16-CCITT), 40-byte uplink (state + RPY + body rates + alt + battery + CRC).
- `controller.c` — single deliberate patch on top of the firmware copy: an input-source mux at the top of `Control()`. SBUS ch4 (SWR) selects manual (SBUS sticks) vs autonomous (COMP struct). 50 ms companion stale-link detection falls back to level attitude + slight-below-hover thrust. Same patched file is intended to compile into the real STM32 build too.
- `test/test_{allocation,rate_pid,atti_pid,quat,protocol}.cpp` — gtest suite validating mixer hover symmetry, rate-PID step response, attitude cascade step convergence within 25%, quaternion-Euler roundtrip, and 200-frame protocol roundtrip + CRC tampering rejection.

검증 (Tier A):
- Standalone smoke harness (host gcc/g++): 35/35 checks pass — same logic as the gtest suite, runs without ROS/colcon while the dev container is unavailable.
- gtest suite designed to run under `colcon test --packages-select fc_core` once `uav-aruco:latest` is rebuilt.

## Decisions

- linalg/controller/filter copy verbatim from firmware, no behavioral edits. Single allowed change is the source-mux in `Control()`; the same edit will be ported back into the embedded firmware repo so sim and hardware stay bit-identical.
- IMU/SBUS/LiDAR drivers in firmware mix DMA setup with pure parsing. Only the parsing halves are extracted. Sim doesn't use them (it feeds the controller struct fields directly from ROS messages), but they exist for hardware-side reuse and HIL testing.
- Protocol uses Q14 (1/16384) for setpoints, Q15 (1/32767) for thrust_norm. Fixed point matches the firmware's existing int16 IMU style and works cleanly over a real UART at 921.6 kbaud. Float32 would also fit but adds endianness ambiguity on the STM32 toolchain.
- `M_PI` is not portable; `linalg.h` already defines `PI` and the port uses that.
