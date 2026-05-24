#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["tqdm"]
# ///
"""Parallel attitude-gain sweep for the fc_sim controller.

Spawns N docker compose run --rm containers in parallel, each running
hover_demo.launch.py headless with a different (rate_kp, atti_kp,
atti_kd) gain triple plus a unique ROS_DOMAIN_ID. Each cell repeats
`--repeats` times (median score reduces DartSim ground-contact noise).
Results are written incrementally to sweep_logs/results.csv so a
killed sweep leaves usable data.

Usage:
    # quick smoke test, ~3 min
    scripts/gain_sweep.py --grid small

    # overnight: ~7 hours at 4 parallel, 11x11x7 grid x 3 repeats
    scripts/gain_sweep.py --grid overnight --jobs 4 --repeats 3

    # custom: pick your own
    scripts/gain_sweep.py --grid coarse --jobs 6 --duration 30 --repeats 2
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

try:
    from tqdm import tqdm
except ImportError:                                            # pragma: no cover
    tqdm = None                                                # type: ignore[assignment]


REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = REPO_ROOT / "sweep_logs"
CSV_PATH = LOG_DIR / "results.csv"


def _logspace(lo: float, hi: float, n: int) -> list[float]:
    if n == 1:
        return [lo]
    r = (hi / lo) ** (1.0 / (n - 1))
    return [round(lo * (r ** i), 4) for i in range(n)]


# Each grid is a list of (rate_kp, atti_kp, atti_kd) triples.
def _grid_3d(rkps: list[float], akps: list[float], akds: list[float]):
    return [(rk, ak, ad) for rk in rkps for ak in akps for ad in akds]


GRIDS = {
    # ~4 cells * 1 repeat * 25 s / 4 jobs ~ 25 s wall
    "small": _grid_3d([0.20], [0.20, 0.40], [0.20, 0.40]),

    # 3*3*3 = 27 cells.  ~5 min at 4 parallel, 1 repeat, 25 s.
    "coarse": _grid_3d(
        _logspace(0.10, 0.40, 3),
        _logspace(0.15, 0.60, 3),
        _logspace(0.10, 0.40, 3),
    ),

    # 5*5*5 = 125 cells.  ~25 min at 4 parallel, 1 repeat, 30 s.
    "fine": _grid_3d(
        _logspace(0.08, 0.40, 5),
        _logspace(0.10, 0.80, 5),
        _logspace(0.08, 0.60, 5),
    ),

    # 11*11*7 = 847 cells. ~7 hours at 4 parallel, 3 repeats, 40 s.
    # Designed to fill an 8-hour overnight slot with headroom.
    "overnight": _grid_3d(
        _logspace(0.05, 0.80, 7),         # rate_kp_p/q
        _logspace(0.05, 1.00, 11),        # atti_kp_roll/pitch
        _logspace(0.05, 1.00, 11),        # atti_kd_roll/pitch
    ),
}


@dataclass
class Run:
    cell_idx: int
    repeat_idx: int
    rate_kp: float
    atti_kp: float
    atti_kd: float
    domain_id: int
    log_path: Path
    z_samples: list[float] = field(default_factory=list)
    score: float = float("inf")
    target_alt: float = 2.0
    elapsed_s: float = 0.0


def docker_cmd(run: Run, duration: float) -> list[str]:
    inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /workspace/install/setup.bash && "
        f"timeout {duration} ros2 launch fc_sim hover_demo.launch.py "
        f"headless:=true "
        f"rate_kp_p:={run.rate_kp} rate_kp_q:={run.rate_kp} "
        f"atti_kp_pitch:={run.atti_kp} atti_kp_roll:={run.atti_kp} "
        f"atti_kd_pitch:={run.atti_kd} atti_kd_roll:={run.atti_kd}"
    )
    return [
        "docker", "compose", "run", "--rm",
        "-e", f"ROS_DOMAIN_ID={run.domain_id}",
        "--name", f"uav-sweep-{run.cell_idx}-{run.repeat_idx}",
        "uav-aruco",
        "bash", "-lc", inner,
    ]


_ZRE = re.compile(r"hover_pub.*?z=([+-]?\d+\.\d+)")


def execute(run: Run, duration: float) -> Run:
    t0 = time.time()
    with run.log_path.open("w", encoding="utf-8") as fp:
        subprocess.run(
            docker_cmd(run, duration),
            stdout=fp,
            stderr=subprocess.STDOUT,
            cwd=REPO_ROOT,
            check=False,
        )
    run.elapsed_s = time.time() - t0
    text = run.log_path.read_text(encoding="utf-8", errors="replace")
    samples = [float(m) for m in _ZRE.findall(text)]
    run.z_samples = samples
    if samples:
        err = [z - run.target_alt for z in samples[-15:]]
        rms = math.sqrt(sum(e * e for e in err) / max(len(err), 1))
        worst = max(abs(e) for e in err)
        run.score = rms + 0.5 * worst
    return run


def fmt_log(s: float) -> str:
    return f"{math.log10(max(s, 1e-6)):+.2f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", choices=GRIDS.keys(), default="small")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--repeats", type=int, default=1,
                    help="Re-run each cell this many times; median wins.")
    ap.add_argument("--resume", action="store_true",
                    help="Skip cells whose CSV row already exists.")
    args = ap.parse_args()

    grid = GRIDS[args.grid]
    total_runs = len(grid) * args.repeats
    est_cell_s = args.duration + 5    # +5 s docker overhead per cell
    est_wall_s = est_cell_s * total_runs / max(args.jobs, 1)
    print(f"Grid: {args.grid} = {len(grid)} cells x {args.repeats} repeats "
          f"= {total_runs} runs")
    print(f"~{est_cell_s:.0f}s per run, {args.jobs} parallel "
          f"-> ~{est_wall_s / 60:.0f} min ({est_wall_s / 3600:.1f} h) wall-clock")
    print()

    if not args.resume and LOG_DIR.exists():
        shutil.rmtree(LOG_DIR)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    # Resume: load any rows already written, skip those (cell_idx,repeat_idx).
    done: set[tuple[int, int]] = set()
    if args.resume and CSV_PATH.exists():
        with CSV_PATH.open("r", encoding="utf-8") as fp:
            for row in csv.DictReader(fp):
                done.add((int(row["cell_idx"]), int(row["repeat_idx"])))
        print(f"Resume mode: {len(done)} runs already in {CSV_PATH}")

    write_header = not CSV_PATH.exists() or not args.resume
    csv_fp = CSV_PATH.open("a", encoding="utf-8", newline="")
    csv_w = csv.writer(csv_fp)
    if write_header:
        csv_w.writerow([
            "cell_idx", "repeat_idx",
            "rate_kp", "atti_kp", "atti_kd",
            "score", "log10_score", "samples", "elapsed_s",
        ])
        csv_fp.flush()

    runs: list[Run] = []
    for ci, (rk, ak, ad) in enumerate(grid):
        for ri in range(args.repeats):
            if (ci, ri) in done:
                continue
            runs.append(Run(
                cell_idx=ci,
                repeat_idx=ri,
                rate_kp=rk,
                atti_kp=ak,
                atti_kd=ad,
                domain_id=100 + (ci * args.repeats + ri) % 200,
                log_path=LOG_DIR / f"cell_{ci:04d}_rep{ri}_"
                                   f"rk{rk:.2f}_ak{ak:.2f}_ad{ad:.2f}.log",
            ))

    n_remaining = len(runs)
    print(f"To run: {n_remaining} (skipping {total_runs - n_remaining} already done)")
    print("Ctrl-C will kill running containers cleanly and preserve CSV + logs.")
    print()

    container_names = {f"uav-sweep-{r.cell_idx}-{r.repeat_idx}" for r in runs}

    def cleanup_containers():
        # Kill anything that's still running so we don't leave gz processes
        # eating CPU/RAM after a kill. `docker rm -f` is no-op for stopped
        # ones, so blasting them all is safe.
        for name in container_names:
            subprocess.run(
                ["docker", "rm", "-f", name],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    use_bar = tqdm is not None and sys.stderr.isatty()
    pbar = tqdm(total=n_remaining, unit="run", smoothing=0.1,
                bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} "
                           "[{elapsed}<{remaining}, {rate_fmt}]") if use_bar else None

    def emit(msg: str) -> None:
        if pbar is not None:
            pbar.write(msg)
        else:
            print(msg, flush=True)

    t_start = time.time()

    try:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = {pool.submit(execute, r, args.duration): r for r in runs}
            for fut in as_completed(futures):
                r = fut.result()
                csv_w.writerow([
                    r.cell_idx, r.repeat_idx,
                    f"{r.rate_kp:.4f}", f"{r.atti_kp:.4f}", f"{r.atti_kd:.4f}",
                    f"{r.score:.4f}", fmt_log(r.score),
                    len(r.z_samples), f"{r.elapsed_s:.1f}",
                ])
                csv_fp.flush()
                os.fsync(csv_fp.fileno())   # actually hit the disk

                emit(
                    f"cell={r.cell_idx:>3d} rep={r.repeat_idx} "
                    f"rk={r.rate_kp:.2f} ak={r.atti_kp:.2f} ad={r.atti_kd:.2f}  "
                    f"score={r.score:7.3f} log10={fmt_log(r.score)}"
                )
                if pbar is not None:
                    pbar.update(1)
    except KeyboardInterrupt:
        emit("\nInterrupted. Killing live containers... (CSV + logs preserved)")
        cleanup_containers()
        if pbar is not None:
            pbar.close()
        csv_fp.close()
        return 130
    finally:
        if pbar is not None and pbar.n > 0:
            pbar.close()
        csv_fp.close()
        cleanup_containers()

    # Summarize: median score per cell, sorted, top 20.
    rows: dict[int, list[Run]] = {}
    for r in runs:
        rows.setdefault(r.cell_idx, []).append(r)
    cells: list[tuple[int, float, float, float, float, int]] = []
    for ci, lst in rows.items():
        scores = [x.score for x in lst]
        med = statistics.median(scores)
        spread = max(scores) - min(scores) if len(scores) > 1 else 0.0
        cells.append((ci, lst[0].rate_kp, lst[0].atti_kp, lst[0].atti_kd, med, len(scores)))
        # Carry spread separately
    cells.sort(key=lambda c: c[4])

    print()
    print("== Top 20 cells (sorted by median score across repeats) ==")
    print("  log10 < -1.3 excellent | -1.3..-0.7 good (PICK HERE) | "
          "-0.7..-0.3 noisy | > -0.3 broken")
    print(f"\n{'rank':>4}  {'rate_kp':>8}  {'atti_kp':>8}  {'atti_kd':>8}  "
          f"{'med':>8}  {'log10':>6}  reps")
    for rank, (ci, rk, ak, ad, med, nreps) in enumerate(cells[:20], start=1):
        print(
            f"{rank:>4}  {rk:>8.3f}  {ak:>8.3f}  {ad:>8.3f}  "
            f"{med:>8.3f}  {fmt_log(med):>6}  {nreps:>4d}"
        )

    print(f"\nFull results: {CSV_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
