# tools

Standalone scripts under `scripts/` that help develop or evaluate the system.

## Done

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
