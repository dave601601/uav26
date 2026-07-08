#!/usr/bin/env python3
"""Record /line_tracer/debug_image and encode a speed-up video.

Runs inside the sim container (needs rclpy + cv_bridge). Two phases:

  record  — subscribe and dump every frame as JPEG plus its sim-time
            stamp; stops on SIGINT and writes times.json.
  encode  — pick frames so the output plays at --speedup x sim time
            with a sane output fps, then write an mp4.

Usage:
  python3 record_debug_video.py record [outdir]
  python3 record_debug_video.py encode [outdir] [--speedup 10] [--fps 30]

The default outdir is /workspace/build/debug_video so the result is
visible on the host under build/debug_video/.
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_OUT = "/workspace/build/debug_video"


class Recorder(Node):
    def __init__(self, outdir: str) -> None:
        super().__init__("debug_video_recorder")
        self._dir = os.path.join(outdir, "frames")
        os.makedirs(self._dir, exist_ok=True)
        self._bridge = CvBridge()
        self._times: list[float] = []
        self._n = 0
        self.create_subscription(
            Image, "/line_tracer/debug_image", self._on_image, 10
        )
        self.get_logger().info(f"recording to {self._dir}")

    def _on_image(self, msg: Image) -> None:
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(
            os.path.join(self._dir, f"{self._n:06d}.jpg"),
            img,
            [cv2.IMWRITE_JPEG_QUALITY, 85],
        )
        self._times.append(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        self._n += 1
        if self._n % 500 == 0:
            self.get_logger().info(f"{self._n} frames")

    def finish(self, outdir: str) -> None:
        with open(os.path.join(outdir, "times.json"), "w") as f:
            json.dump(self._times, f)
        self.get_logger().info(f"done: {self._n} frames")


def record(outdir: str) -> int:
    rclpy.init()
    node = Recorder(outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish(outdir)
        node.destroy_node()
    return 0


def encode(outdir: str, speedup: float, out_fps: float) -> int:
    with open(os.path.join(outdir, "times.json")) as f:
        times = json.load(f)
    if len(times) < 2:
        print("not enough frames", file=sys.stderr)
        return 1
    # Sim time can reset mid-recording (a second sim session starting
    # while the recorder is still subscribed stamps from zero again).
    # Use the longest monotonic segment instead of trusting
    # times[0]..times[-1].
    seg_start = best_start = 0
    best_len = 0
    for i in range(1, len(times) + 1):
        if i == len(times) or times[i] < times[i - 1] - 1.0:
            if i - seg_start > best_len:
                best_start, best_len = seg_start, i - seg_start
            seg_start = i
    lo, hi = best_start, best_start + best_len - 1
    if best_len < len(times):
        print(
            f"clock reset detected: using frames {lo}..{hi} "
            f"({best_len}/{len(times)})"
        )
    t0, t1 = times[lo], times[hi]
    duration_out = (t1 - t0) / speedup
    total_out = max(1, int(duration_out * out_fps))
    # For each output tick, pick the source frame nearest in sim time.
    picks: list[int] = []
    src = lo
    for k in range(total_out):
        target = t0 + (k / out_fps) * speedup
        while src + 1 <= hi and times[src + 1] <= target:
            src += 1
        picks.append(src)

    first = cv2.imread(os.path.join(outdir, "frames", f"{picks[0]:06d}.jpg"))
    h, w = first.shape[:2]
    out_path = os.path.join(outdir, f"debug_{int(speedup)}x.mp4")
    writer = cv2.VideoWriter(
        out_path, cv2.VideoWriter_fourcc(*"mp4v"), out_fps, (w, h)
    )
    for idx in picks:
        frame = cv2.imread(os.path.join(outdir, "frames", f"{idx:06d}.jpg"))
        if frame is not None:
            writer.write(frame)
    writer.release()
    print(
        f"wrote {out_path}: {len(picks)} frames, "
        f"{duration_out:.1f} s at {out_fps:.0f} fps "
        f"({speedup:.0f}x of {t1 - t0:.0f} sim-seconds)"
    )
    return 0


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] not in ("record", "encode"):
        print(__doc__, file=sys.stderr)
        return 2
    mode = args[0]
    outdir = args[1] if len(args) > 1 and not args[1].startswith("--") else DEFAULT_OUT
    speedup = 10.0
    out_fps = 30.0
    if "--speedup" in args:
        speedup = float(args[args.index("--speedup") + 1])
    if "--fps" in args:
        out_fps = float(args[args.index("--fps") + 1])
    if mode == "record":
        return record(outdir)
    return encode(outdir, speedup, out_fps)


if __name__ == "__main__":
    sys.exit(main())
