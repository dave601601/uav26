# fc_core

Pure C library carrying the STM32 firmware controller into ROS2 land.

## Done

### maxratecmd 1.0 -> 3.0 rad/s for autonomous yaw authority (commit `81918ba`, 2026-05-25)

The firmware's `maxratecmd = 1.0` rad/s was calibrated for SBUS stick
travel; in autonomous mode it clamped the companion's yawrate setpoint
below the algorithm-side `max_wz = 2.5`, so the yaw lock was always
clipped and drone yaw drifted ~0.2 rad/s uncorrected (line_tracer
r30..r34 cruise curled off-axis). Raised to 3.0 rad/s.
`fc_rate_deadband_factor` scaled 0.04 -> 0.0133 so the absolute rate
deadband stays at 0.04 rad/s — without the rescale `test_atti_pid`
fails because the proportional deadband widens with maxratecmd.
This deviates from the verbatim-copy rule (see Decisions); the change
must be ported back to the embedded firmware repo.

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
