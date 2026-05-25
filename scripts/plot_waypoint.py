#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Plot a waypoint_demo telemetry log.

Reads a file produced by piping the headless waypoint_demo launch output:

    timeout 60 ros2 launch fc_sim waypoint_demo.launch.py headless:=true \\
        pattern:=box 2>&1 | grep -E 'WP|>>' > run.log

Extracts the waypoint target list, the per-1 Hz pos/vel samples, and the
WP advance events, then writes a PNG with three subplots:

  1. Top-down (x, y) trajectory, waypoints labelled and connected.
  2. Position vs time, x / y / z each overlaid with their waypoint
     targets (held over the time interval each WP was active).
  3. Distance-to-current-waypoint vs time.

Usage:
    scripts/plot_waypoint.py sweep_logs/manual/r09_wp_fb_v3.log
    scripts/plot_waypoint.py path/to/run.log --out path/to/run.png
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


TIME_RE = re.compile(r"\[INFO\] \[(\d+\.\d+)\]")
WP_DEF_RE = re.compile(
    r"WP (\d+): \(([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+)\)"
)
WP_ADVANCE_RE = re.compile(
    r">> WP (\d+): \(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)"
)
WP_EVENT_RE = re.compile(r"WP (\d+) (reached|TIMEOUT)")
SAMPLE_RE = re.compile(
    r"WP(\d+) tgt=\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)"
    r" pos=\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)"
    r" v=\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)"
    r" dist=([\d\.]+)"
)


@dataclass
class WP:
    idx: int
    x: float
    y: float
    z: float


@dataclass
class Sample:
    t: float
    wp_idx: int
    pos: tuple[float, float, float]
    vel: tuple[float, float, float]
    dist: float


@dataclass
class Advance:
    t: float
    wp_idx: int
    kind: str       # "reached" or "TIMEOUT"


@dataclass
class Run:
    waypoints: list[WP] = field(default_factory=list)
    samples: list[Sample] = field(default_factory=list)
    advances: list[Advance] = field(default_factory=list)
    t0: float | None = None


def parse(path: Path) -> Run:
    run = Run()
    wp_defs_seen: dict[int, WP] = {}
    with path.open() as fp:
        for line in fp:
            tm = TIME_RE.search(line)
            t_abs = float(tm.group(1)) if tm else None
            if t_abs is not None and run.t0 is None and SAMPLE_RE.search(line):
                run.t0 = t_abs

            m = WP_DEF_RE.search(line)
            if m and ">>" not in line:
                idx = int(m.group(1))
                if idx not in wp_defs_seen:
                    wp_defs_seen[idx] = WP(idx, float(m.group(2)),
                                           float(m.group(3)), float(m.group(4)))
                continue

            m = WP_EVENT_RE.search(line)
            if m and t_abs is not None:
                run.advances.append(
                    Advance(t=t_abs, wp_idx=int(m.group(1)), kind=m.group(2))
                )
                continue

            m = SAMPLE_RE.search(line)
            if m and t_abs is not None:
                run.samples.append(Sample(
                    t=t_abs,
                    wp_idx=int(m.group(1)),
                    pos=(float(m.group(5)), float(m.group(6)), float(m.group(7))),
                    vel=(float(m.group(8)), float(m.group(9)), float(m.group(10))),
                    dist=float(m.group(11)),
                ))

    run.waypoints = [wp_defs_seen[i] for i in sorted(wp_defs_seen)]
    if run.t0 is None and run.samples:
        run.t0 = run.samples[0].t
    return run


def plot(run: Run, out: Path) -> None:
    if not run.samples:
        print("no samples parsed -- nothing to plot", file=sys.stderr)
        sys.exit(1)

    t0 = run.t0 or 0.0
    t = np.array([s.t - t0 for s in run.samples])
    pos = np.array([s.pos for s in run.samples])     # (N, 3)
    vel = np.array([s.vel for s in run.samples])
    dist = np.array([s.dist for s in run.samples])
    wp_idx = np.array([s.wp_idx for s in run.samples])

    wp_xyz = np.array([[w.x, w.y, w.z] for w in run.waypoints])

    fig, axes = plt.subplots(3, 1, figsize=(10, 11),
                             gridspec_kw={"height_ratios": [1.4, 1.6, 1.0]})

    # --- top-down xy trajectory ----------------------------------------
    ax = axes[0]
    ax.plot(pos[:, 0], pos[:, 1], "-", color="#2266cc", lw=1.5, label="drone path")
    ax.scatter(pos[0, 0], pos[0, 1], color="green", s=80, zorder=5, label="start")
    ax.scatter(pos[-1, 0], pos[-1, 1], color="red", s=80, marker="X",
               zorder=5, label="end")
    for w in run.waypoints:
        ax.scatter(w.x, w.y, color="black", marker="s", s=80, zorder=4)
        ax.annotate(f"WP{w.idx}", (w.x, w.y), xytext=(8, 8),
                    textcoords="offset points", fontsize=10, fontweight="bold")
    if len(run.waypoints) >= 2:
        ax.plot(wp_xyz[:, 0], wp_xyz[:, 1], "k--", lw=0.8, alpha=0.5,
                label="planned route")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("top-down trajectory")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # --- position vs time, with WP targets overlaid as step lines ------
    ax = axes[1]
    colors = {"x": "#cc3344", "y": "#22aa55", "z": "#5566cc"}
    for k, axis_i, label in [("x", 0, "x"), ("y", 1, "y"), ("z", 2, "z")]:
        ax.plot(t, pos[:, axis_i], color=colors[k], lw=1.4, label=f"{label} pos")

    # WP target as a step function: which WP was current at each sample time.
    if run.waypoints:
        wp_tx = wp_xyz[wp_idx, 0]
        wp_ty = wp_xyz[wp_idx, 1]
        wp_tz = wp_xyz[wp_idx, 2]
        ax.plot(t, wp_tx, color=colors["x"], lw=1.0, linestyle=":",
                alpha=0.7, label="x target")
        ax.plot(t, wp_ty, color=colors["y"], lw=1.0, linestyle=":",
                alpha=0.7, label="y target")
        ax.plot(t, wp_tz, color=colors["z"], lw=1.0, linestyle=":",
                alpha=0.7, label="z target")

    # Annotate WP advance events.
    for a in run.advances:
        ta = a.t - t0
        color = "green" if a.kind == "reached" else "orange"
        ax.axvline(ta, color=color, lw=0.7, alpha=0.6)
        ax.text(ta, ax.get_ylim()[1] * 0.95,
                f"WP{a.wp_idx} {a.kind}",
                rotation=90, va="top", ha="right", fontsize=8, color=color)

    ax.set_xlabel("t [s]")
    ax.set_ylabel("position [m]")
    ax.set_title("position vs time (solid = measured, dotted = target)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=3)

    # --- distance-to-current-WP vs time --------------------------------
    ax = axes[2]
    ax.plot(t, dist, color="#882288", lw=1.4, label="dist to current WP")
    for a in run.advances:
        ta = a.t - t0
        color = "green" if a.kind == "reached" else "orange"
        ax.axvline(ta, color=color, lw=0.7, alpha=0.6)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("dist [m]")
    ax.set_title("distance to current waypoint")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out} ({len(run.samples)} samples, "
          f"{len(run.waypoints)} waypoints, {len(run.advances)} events)")

    # Quick text summary.
    if run.waypoints and run.samples:
        final = run.samples[-1]
        last_wp = run.waypoints[final.wp_idx]
        final_err = math.sqrt(
            (final.pos[0] - last_wp.x) ** 2
            + (final.pos[1] - last_wp.y) ** 2
            + (final.pos[2] - last_wp.z) ** 2
        )
        print(f"final pos=({final.pos[0]:+.2f},{final.pos[1]:+.2f},"
              f"{final.pos[2]:+.2f}) -> WP{last_wp.idx} err={final_err:.2f} m")
        for a in run.advances:
            print(f"  {a.kind:>8s}  WP{a.wp_idx}  t={a.t - t0:.1f} s")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="waypoint_demo log file")
    ap.add_argument("--out", type=Path, default=None,
                    help="output PNG (default: <log>.png)")
    args = ap.parse_args()

    if not args.log.is_file():
        print(f"log not found: {args.log}", file=sys.stderr)
        return 1

    out = args.out or args.log.with_suffix(".png")
    run = parse(args.log)
    plot(run, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
