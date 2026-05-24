#!/usr/bin/env python3
"""Parallel attitude-gain sweep for the fc_sim controller.

Spawns N docker compose run --rm containers, each with a different
(kp_atti, kd_atti) combination and a unique ROS_DOMAIN_ID. Each runs
hover_demo.launch.py headless for `--duration` seconds and writes its
launch log to `sweep_logs/run_<i>.log`. Once all finish, parses each
log's hover_pub telemetry and scores by RMS altitude error + worst-case
deviation.

Usage:
    scripts/gain_sweep.py                # default grid, 4 parallel
    scripts/gain_sweep.py --jobs 6 --duration 25
    scripts/gain_sweep.py --grid coarse  # 9 combos
    scripts/gain_sweep.py --grid fine    # 25 combos
"""
from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = REPO_ROOT / "sweep_logs"


def _logspace(lo: float, hi: float, n: int) -> list[float]:
    """n points geometrically spaced between lo and hi (inclusive)."""
    if n == 1:
        return [lo]
    r = (hi / lo) ** (1.0 / (n - 1))
    return [round(lo * (r ** i), 3) for i in range(n)]


# Gain grids. Log-spaced (each step is the same multiplicative ratio)
# because attitude gains are multiplicative in effect — kp 0.10 -> 0.20
# matters the same as 0.20 -> 0.40, not as 0.20 -> 0.30.
GRIDS = {
    "small": [
        (0.20, 0.20),
        (0.30, 0.30),
        (0.40, 0.20),
        (0.40, 0.40),
    ],
    "coarse": [                       # 3x3 = 9 cells, span 4x
        (kp, kd)
        for kp in _logspace(0.15, 0.60, 3)
        for kd in _logspace(0.10, 0.40, 3)
    ],
    "fine": [                         # 5x5 = 25 cells, span 8x
        (kp, kd)
        for kp in _logspace(0.10, 0.80, 5)
        for kd in _logspace(0.08, 0.50, 5)
    ],
}


@dataclass
class Run:
    idx: int
    kp_atti: float
    kd_atti: float
    domain_id: int
    log_path: Path
    z_samples: list[float] = field(default_factory=list)
    score: float = float("inf")
    target_alt: float = 2.0


def docker_cmd(run: Run, duration: float) -> list[str]:
    """Build the docker compose run --rm command for one sweep cell."""
    inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /workspace/install/setup.bash && "
        f"timeout {duration} ros2 launch fc_sim hover_demo.launch.py "
        f"headless:=true "
        f"atti_kp_pitch:={run.kp_atti} atti_kp_roll:={run.kp_atti} "
        f"atti_kd_pitch:={run.kd_atti} atti_kd_roll:={run.kd_atti}"
    )
    return [
        "docker", "compose", "run", "--rm",
        "-e", f"ROS_DOMAIN_ID={run.domain_id}",
        "--name", f"uav-sweep-{run.idx}",
        "uav-aruco",
        "bash", "-lc", inner,
    ]


def execute(run: Run, duration: float) -> Run:
    """Run one sweep cell, capture output to log."""
    with run.log_path.open("w", encoding="utf-8") as fp:
        proc = subprocess.run(
            docker_cmd(run, duration),
            stdout=fp,
            stderr=subprocess.STDOUT,
            cwd=REPO_ROOT,
            check=False,
        )
    fp_check = run.log_path.read_text(encoding="utf-8", errors="replace")
    # Pull every "z=+1.23" sample from hover_pub log lines.
    pattern = re.compile(r"hover_pub.*?z=([+-]?\d+\.\d+)")
    samples = [float(m) for m in pattern.findall(fp_check)]
    run.z_samples = samples
    if samples:
        # Score: RMS error against target, plus worst-case deviation.
        err = [z - run.target_alt for z in samples[-15:]]   # last 15 s only
        rms = math.sqrt(sum(e * e for e in err) / max(len(err), 1))
        worst = max(abs(e) for e in err) if err else float("inf")
        run.score = rms + 0.5 * worst
    return run


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", choices=GRIDS.keys(), default="small")
    ap.add_argument("--jobs", type=int, default=4,
                    help="Parallel containers. Each gz sim ~1.5 GB RAM, 1 CPU.")
    ap.add_argument("--duration", type=float, default=25.0,
                    help="Seconds each cell runs before timeout.")
    args = ap.parse_args()

    grid = GRIDS[args.grid]
    print(f"Sweeping {len(grid)} cells, {args.jobs} parallel, {args.duration} s each")

    if LOG_DIR.exists():
        shutil.rmtree(LOG_DIR)
    LOG_DIR.mkdir(parents=True)

    runs: list[Run] = []
    for i, (kp, kd) in enumerate(grid):
        runs.append(Run(
            idx=i,
            kp_atti=kp,
            kd_atti=kd,
            domain_id=100 + i,
            log_path=LOG_DIR / f"run_{i:02d}_kp{kp:.2f}_kd{kd:.2f}.log",
        ))

    def fmt_log(s: float) -> str:
        return f"{math.log10(max(s, 1e-6)):+.2f}"

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(execute, r, args.duration): r for r in runs}
        for fut in as_completed(futures):
            r = fut.result()
            print(
                f"  [{r.idx:02d}] kp={r.kp_atti:.2f} kd={r.kd_atti:.2f} "
                f"samples={len(r.z_samples)} "
                f"score={r.score:.3f} log10={fmt_log(r.score)}"
            )

    elapsed = time.time() - t0
    print(f"\n== Results (sorted by log10(score), elapsed {elapsed:.0f} s) ==")
    print("   log10(score) bracket guide:")
    print("     < -1.3   (score < 0.05)  : excellent")
    print("     -1.3 to -0.7 (0.05-0.20) : good — pick from here")
    print("     -0.7 to -0.3 (0.20-0.50) : usable, noisy")
    print("     > -0.3   (score > 0.50)  : oscillating or broken")
    runs.sort(key=lambda r: r.score)
    print(f"\n{'rank':>4}  {'kp_atti':>8}  {'kd_atti':>8}  {'score':>8}  {'log10':>6}  log")
    for rank, r in enumerate(runs, start=1):
        print(
            f"{rank:>4}  {r.kp_atti:>8.2f}  {r.kd_atti:>8.2f}  "
            f"{r.score:>8.3f}  {fmt_log(r.score):>6}  "
            f"{r.log_path.relative_to(REPO_ROOT)}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
