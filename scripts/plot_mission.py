#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy", "pyyaml"]
# ///
"""Plot a line_tracer mission log.

Reads the headless line_tracer log piped from:

    ros2 launch line_tracer line_tracer.launch.py > run.log 2>&1

Extracts:
  - FSM transitions (`>> FSM: A -> B (alt=...)`)
  - Marker records (`>> RECORD aruco id=N at (x, y)`)
  - 1 Hz status (`[STATE/source] xy=(x,y) yaw=Y alt=Z mode=M dir=D`)

Writes a 3-panel PNG:

  1. Top-down (x, y) trajectory coloured by FSM state. Recorded markers
     overlaid as squares; ground-truth markers from aruco_layout.yaml as
     open circles for reference. The +X / +Y axes are world frame.

  2. Altitude vs time, with the FSM state bands shaded behind so it's
     obvious which phase saw which altitude excursion.

  3. State timeline.

Optional --layout argument points to aruco_layout.yaml so the recorded
markers can be diff'd against ground truth. Defaults to looking in
$WORKSPACE/install/world/share/world/config/aruco_layout.yaml.

Usage:
    scripts/plot_mission.py sweep_logs/mission/r07_tracer.log
    scripts/plot_mission.py path/to/run.log --layout path/to/aruco_layout.yaml
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import yaml


TIME_RE = re.compile(r"\[INFO\] \[(\d+\.\d+)\]")
FSM_RE = re.compile(r">> FSM: (\w+) -> (\w+) \(alt=([+-]?\d+\.\d+)\)")
RECORD_RE = re.compile(
    r">> RECORD aruco id=(\d+) at \(([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+)\)"
)
# Tolerant of fields inserted between xy and alt (the skeleton backend added
# yaw there) and of the trailing field changing (vz_truth -> mode/dir).
SAMPLE_RE = re.compile(
    r"\[(\w+)/\w+\] xy=\(([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+)\)"
    r"[^=]*?(?:\w+=\S+ )*?alt=([+-]?\d+\.\d+)"
)
VZ_RE = re.compile(r"vz_truth=([+-]?\d+\.\d+)")


# Both backends: legacy FSM names and the skeleton's MissionState names.
STATE_COLORS = {
    "TAKEOFF": "#7c8cff",
    "LINE_FOLLOW": "#33aa55",
    "WAYPOINT_VISIT": "#cc8822",
    "ARRANGE_BY_ID": "#aa44aa",
    "RETURN_PATH": "#a05050",
    "LAND": "#444444",
    "LOCALIZE": "#6699cc",
    "ENTER_GRID": "#44bbaa",
    "EXPLORE": "#33aa55",
    "MARKER_CONFIRM": "#cc8822",
    "PLAN_RESCUE_PATH": "#aa44aa",
    "FOLLOW_RESCUE_PATH": "#a05050",
    "RETURN_HOME": "#996633",
    "FINISHED": "#222222",
    "FAILSAFE": "#cc2222",
}


@dataclass
class Sample:
    t: float
    state: str
    x: float
    y: float
    alt: float
    vz_truth: float


@dataclass
class FsmEvent:
    t: float
    prev: str
    new: str
    alt: float


@dataclass
class RecordEvent:
    t: float
    marker_id: int
    x: float
    y: float


@dataclass
class Run:
    samples: list[Sample] = field(default_factory=list)
    fsm_events: list[FsmEvent] = field(default_factory=list)
    records: list[RecordEvent] = field(default_factory=list)
    t0: float | None = None


def parse(path: Path) -> Run:
    run = Run()
    with path.open() as fp:
        for line in fp:
            tm = TIME_RE.search(line)
            t = float(tm.group(1)) if tm else None

            m = FSM_RE.search(line)
            if m and t is not None:
                run.fsm_events.append(FsmEvent(t, m.group(1), m.group(2), float(m.group(3))))
                if run.t0 is None:
                    run.t0 = t

            m = RECORD_RE.search(line)
            if m and t is not None:
                run.records.append(
                    RecordEvent(t, int(m.group(1)), float(m.group(2)), float(m.group(3)))
                )

            m = SAMPLE_RE.search(line)
            if m and t is not None:
                x = float(m.group(2))
                y = float(m.group(3))
                alt = float(m.group(4))
                # DartSim occasionally explodes mid-run and /odom_truth
                # spews multi-thousand-metre values for a few frames.
                # Drop those so they don't dominate the plot.
                if abs(x) > 200 or abs(y) > 200 or alt < -10 or alt > 50:
                    continue
                vz = VZ_RE.search(line)
                run.samples.append(Sample(
                    t=t, state=m.group(1), x=x, y=y, alt=alt,
                    vz_truth=float(vz.group(1)) if vz else 0.0,
                ))
                if run.t0 is None:
                    run.t0 = t

    if run.t0 is None:
        run.t0 = 0.0
    return run


def load_layout(path: Path | None) -> dict[int, tuple[float, float]]:
    if path is None:
        # Default location after `colcon build world`.
        cands = [
            Path("/workspace/install/world/share/world/config/aruco_layout.yaml"),
            Path.cwd() / "install/world/share/world/config/aruco_layout.yaml",
            Path.cwd() / "src/world/config/aruco_layout.yaml",
        ]
        for c in cands:
            if c.is_file():
                path = c
                break
    if path is None or not path.is_file():
        return {}
    with open(path) as fp:
        data = yaml.safe_load(fp)
    out: dict[int, tuple[float, float]] = {}
    # marker_randomize.py emits one of two shapes; tolerate both:
    #   - {"markers": [{"id": N, "pose": [x, y, z, rx, ry, rz]}, ...]}
    #   - {"markers": [{"id": N, "x": X, "y": Y}, ...]}
    for entry in data.get("markers", []):
        if "pose" in entry:
            pose = entry["pose"]
            out[int(entry["id"])] = (float(pose[0]), float(pose[1]))
        else:
            out[int(entry["id"])] = (float(entry["x"]), float(entry["y"]))
    return out


def plot(run: Run, layout: dict[int, tuple[float, float]], out: Path) -> None:
    if not run.samples:
        print("no samples parsed -- nothing to plot", file=sys.stderr)
        sys.exit(1)

    t0 = run.t0
    t = np.array([s.t - t0 for s in run.samples])
    x = np.array([s.x for s in run.samples])
    y = np.array([s.y for s in run.samples])
    alt = np.array([s.alt for s in run.samples])
    states = np.array([s.state for s in run.samples])

    fig, axes = plt.subplots(3, 1, figsize=(10, 11),
                             gridspec_kw={"height_ratios": [1.6, 1.0, 0.5]})

    # --- top-down trajectory, coloured by state ---
    ax = axes[0]
    seen_states: set[str] = set()
    for i in range(len(t) - 1):
        c = STATE_COLORS.get(states[i], "#888888")
        ax.plot([x[i], x[i+1]], [y[i], y[i+1]], color=c, lw=1.6,
                label=states[i] if states[i] not in seen_states else None)
        seen_states.add(states[i])
    if len(t) > 0:
        ax.scatter(x[0], y[0], color="green", marker="*", s=180, zorder=6,
                   edgecolors="black", linewidths=0.6, label="start")
        ax.scatter(x[-1], y[-1], color="red", marker="X", s=140, zorder=6,
                   edgecolors="black", linewidths=0.6, label="end")
    for rec in run.records:
        ax.scatter(rec.x, rec.y, color="black", marker="s", s=130, zorder=5)
        ax.annotate(f"rec {rec.marker_id}", (rec.x, rec.y),
                    xytext=(8, 8), textcoords="offset points",
                    fontsize=9, fontweight="bold")
    for mid, (mx, my) in layout.items():
        ax.scatter(mx, my, facecolors="none", edgecolors="#cc2222",
                   marker="o", s=200, lw=2.0, zorder=4)
        ax.annotate(f"gt {mid}", (mx, my), xytext=(-26, -16),
                    textcoords="offset points", fontsize=8, color="#cc2222")

    # Grid lines: the 2026-07 official spec is a 30 x 21 m arena on 3 m cells.
    for gx in range(0, 31, 3):
        ax.axvline(gx, color="#dddddd", lw=0.6, zorder=1)
    for gy in range(0, 22, 3):
        ax.axhline(gy, color="#dddddd", lw=0.6, zorder=1)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("trajectory (colour = FSM state); ◻ recorded marker, ○ ground truth")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best", fontsize=8)

    # --- altitude vs time, FSM bands ---
    ax = axes[1]
    _shade_state_bands(ax, run, t0)
    ax.plot(t, alt, color="black", lw=1.4, label="alt (m)")
    ax.axhline(2.0, color="gray", lw=0.8, linestyle="--", alpha=0.6, label="target_alt")
    for rec in run.records:
        ax.axvline(rec.t - t0, color="orange", lw=0.7, alpha=0.7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("altitude [m]")
    ax.set_title("altitude vs time (state bands shaded)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)

    # --- state timeline ---
    ax = axes[2]
    _state_strip(ax, run, t0)
    ax.set_xlabel("t [s]")
    ax.set_yticks([])
    ax.set_title("FSM state")

    fig.tight_layout()
    fig.savefig(out, dpi=130)

    # text summary
    print(f"wrote {out} ({len(run.samples)} samples, "
          f"{len(run.fsm_events)} fsm events, {len(run.records)} records)")
    if run.records:
        print("recorded markers:")
        for rec in run.records:
            err: str
            if rec.marker_id in layout:
                gt = layout[rec.marker_id]
                d = ((gt[0] - rec.x) ** 2 + (gt[1] - rec.y) ** 2) ** 0.5
                err = f" (gt=({gt[0]:.2f},{gt[1]:.2f}), err={d:.2f} m)"
            else:
                err = " (no gt in layout)"
            print(f"  id={rec.marker_id}  at ({rec.x:+.2f}, {rec.y:+.2f}){err}")
    if run.fsm_events:
        print("FSM events:")
        for ev in run.fsm_events:
            print(f"  t={ev.t - t0:6.2f}s   {ev.prev} -> {ev.new}   (alt={ev.alt:.2f})")


def _bands_from_samples(run: Run, t0: float):
    """Iterate (state, t_start, t_end) bands from the sample stream."""
    if not run.samples:
        return
    cur_state = run.samples[0].state
    t_start = run.samples[0].t - t0
    for s in run.samples[1:]:
        ts = s.t - t0
        if s.state != cur_state:
            yield cur_state, t_start, ts
            cur_state = s.state
            t_start = ts
    yield cur_state, t_start, run.samples[-1].t - t0


def _shade_state_bands(ax, run: Run, t0: float) -> None:
    for st, ts, te in _bands_from_samples(run, t0):
        ax.axvspan(ts, te, color=STATE_COLORS.get(st, "#cccccc"), alpha=0.18)


def _state_strip(ax, run: Run, t0: float) -> None:
    for st, ts, te in _bands_from_samples(run, t0):
        ax.barh(0, te - ts, left=ts, height=1.0,
                color=STATE_COLORS.get(st, "#cccccc"))
    # legend
    handles = [mpatches.Patch(color=c, label=s) for s, c in STATE_COLORS.items()
               if any(sm.state == s for sm in run.samples)]
    ax.legend(handles=handles, loc="upper right", fontsize=8, ncol=3)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="line_tracer mission log")
    ap.add_argument("--out", type=Path, default=None,
                    help="output PNG (default: <log>.png)")
    ap.add_argument("--layout", type=Path, default=None,
                    help="aruco_layout.yaml path (auto-detected if omitted)")
    args = ap.parse_args()

    if not args.log.is_file():
        print(f"log not found: {args.log}", file=sys.stderr)
        return 1

    out = args.out or args.log.with_suffix(".png")
    run = parse(args.log)
    layout = load_layout(args.layout)
    plot(run, layout, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
