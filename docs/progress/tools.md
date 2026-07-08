# tools

Standalone scripts under `scripts/` that help develop or evaluate the system.

## Done

### `scripts/dev.sh` + `scripts/run_mission.sh` — workflow drivers (2026-07-08)

`dev.sh` is the daily driver: `gui` (X11 bridge + container up +
rebuild-if-needed + Gazebo GUI + tracer, Ctrl+C teardown), `view`
(rqt_image_view on /line_tracer/debug_image), `mission rNN [dur]`
(headless run + FSM summary), `build`. `run_mission.sh` is the
underlying headless runner and carries the zombie-sweep contract
(sweep before start, SIGINT first, pkill fc_sim_node /
parameter_bridge by name — see docker.md for why).

### `src/line_tracer/scripts/record_debug_video.py` — camera overlay to N-x video (2026-07-08)

Runs in the container: `record` dumps every /line_tracer/debug_image
frame as JPEG + sim stamp; `encode --speedup 10 --fps 30` resamples
frames on the sim timeline (RTF-independent playback speed) and
writes an mp4 under build/debug_video/. Picks the longest monotonic
stamp segment first — a second sim session starting mid-recording
restamps from zero and would otherwise collapse the video span (r60).
Used for the r60 full-mission 10x detection video.

### `scripts/plot_waypoint.py` — visualise a waypoint_demo log (2026-05-25)

Reads a piped `ros2 launch fc_sim waypoint_demo.launch.py ... | grep -E 'WP|>>'` log, extracts waypoint definitions, the per-1 Hz pos/vel/dist samples, and the WP advance events, then writes a 3-row PNG: top-down trajectory with waypoint markers, position-vs-time with target overlay, and distance-to-current-WP. Also prints a one-line summary of per-WP arrival times and final positional error.

Runs on the host via uv (PEP 723 inline metadata pulls matplotlib + numpy). No docker required — it just reads a log file.

    scripts/plot_waypoint.py sweep_logs/manual/r10_wp_box.log
    scripts/plot_waypoint.py path/to/run.log --out path/to/run.png

Sample output: `sweep_logs/manual/r10_wp_box.png` — box pattern (3 m × 3 m square) shows clean WP0→WP1→WP2→WP3 traversal (~6 s per leg) and the limit-cycle around the return waypoint that the cascaded P-pos / P-vel controller can't fully damp without feedforward.

### `scripts/gain_sweep.py` — parallel attitude-gain sweep (commits `00276a8`–`0d8d049`, 2026-05-25)

Spawns N docker compose run --rm containers in parallel, each running `hover_demo.launch.py` headless with a different `(rate_kp, atti_kp, atti_kd)` triple and a unique `ROS_DOMAIN_ID`. Parses telemetry from each cell's log, scores by `RMS(z − target_alt) + 0.5 × worst-case` over the last 15 s, then medians across repeats per cell.

**Grids** (`--grid <name>`):
- `small` — 4 cells. ~30 s wall-clock. Smoke test the harness.
- `coarse` — 3×3×3 = 27 cells. ~5 min at 4 parallel, 1 repeat, 25 s.
- `fine` — 5×5×5 = 125 cells. ~25 min at 4 parallel.
- `overnight` — 11×11×7 = 847 cells. **~7.9 h** at 4 parallel × 3 repeats × 40 s. Designed to fill an 8 h sleep window.

All grids are **log-spaced** because gains are multiplicative in effect (kp 0.10 → 0.20 has the same dynamic impact as 0.20 → 0.40).

**Output**:
- `sweep_logs/results.csv` — one row per `(cell_idx, repeat_idx)` with all params, score, log10 score, sample count, elapsed seconds. Written incrementally with `fsync` per row so interrupts and reboots can't lose data.
- `sweep_logs/cell_<i>_rep<j>_*.log` — full launch log per cell. Useful for forensics on cells that scored badly.
- End-of-run summary: top-20 cells by median score across repeats, with the `log10(score)` bracket guide reprinted (`< -1.3` excellent, `-1.3..-0.7` good, `-0.7..-0.3` noisy, `> -0.3` broken).

**Resilience**:
- Ctrl-C handler kills outstanding docker containers cleanly so they don't leak into the host.
- `--resume` skips `(cell_idx, repeat_idx)` rows already in the CSV. Useful for crash recovery and for adding repeats after the fact.
- Each cell runs under `timeout <duration>` so a stuck gz simulation can't pin a container forever.

**Dependencies via uv**: PEP 723 inline metadata + `#!/usr/bin/env -S uv run --script` shebang. First invocation pulls `tqdm` into uv's cache (~8 ms); subsequent invocations are instant. No pip/apt install needed.

## Open

- Extend `gain_sweep.py` to also sweep `kp_alt` / `kd_alt` and score attitude-tracking error (qx, qy during PITCH_FORWARD / ROLL_RIGHT phases of `flight_demo`). Currently only hover altitude is scored.
- After the overnight run lands, write `scripts/analyze_sweep.py` for pandas plots / heatmap of the (kp, kd) plane.

## Decisions

- The sweep uses docker compose run --rm rather than `docker run` directly so it inherits everything from `compose.yml` + `compose.override.yml` (volume mounts, NVIDIA flags, etc.). Each cell gets its own ephemeral container.
- Scoring penalizes both RMS error and worst-case deviation in a single metric. Linear scale for the metric itself (interpretable as "metres of error"); log scale for ranking display (so "excellent vs good" stays visually distinct from "broken").
- 3 repeats per cell is the sweet spot for handling the DartSim ground-stuck flake without doubling wall-clock. Median across repeats is robust to a single broken run.
- uv is the preferred Python script tooling for this repo. PEP 668 blocks system pip; uv runs scripts in ephemeral envs with cached deps, no global state to manage. See the corresponding feedback memory entry.
