"""격자 + 잔디 통합 바닥 텍스처 생성기.

공식 스펙 (2026-07 실사 공지): 실외 연병장 초록 잔디밭 위에 하얀색 새틴
리본 격자선 (폭 10 cm, 셀 3 m). aruco_layout.yaml 의 marker pose 를 읽어,
그 위치 (marker_size × marker_size) 정사각 구역에는 격자선을 그리지 않은
PNG (grass + white grid) 를 생성한다.

이 한 장으로:
  - 별도 grid STL mesh 가 필요 없고 (z-fight 도 사라짐)
  - ArUco 마커 영역엔 격자선이 침범하지 않으며
  - line_tracer 카메라 perception 은 밝은 격자선을 잘 잡는다
    (Canny+Hough 는 극성 무관; 흰 선 vs 잔디는 mono 에서 고대비).

사용 (script/ 에서):
  python3 floor_tex.py \
      --layout ../config/aruco_layout.yaml \
      --width 30 --depth 21 --cell 3 --line-width 0.10 \
      --px-per-m 100 \
      -o ../textures/floor.png
"""
from __future__ import annotations

import argparse
import os
import re

import cv2
import numpy as np


def parse_layout(path: str) -> tuple[float, list[tuple[float, float]]]:
    """layout YAML 의 marker_size 와 markers[].pose(x,y) 를 자체 파서로 추출."""
    marker_size = 0.4
    markers: list[tuple[float, float]] = []
    current_id: int | None = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            m = re.match(r"^marker_size:\s*([0-9.]+)", stripped)
            if m:
                marker_size = float(m.group(1))
                continue
            m = re.match(r"^- id:\s*(\d+)", stripped)
            if m:
                current_id = int(m.group(1))
                continue
            m = re.match(
                r"^pose:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)", stripped
            )
            if m and current_id is not None:
                markers.append((float(m.group(1)), float(m.group(2))))
                current_id = None
    return marker_size, markers


def make_luma_noise(h: int, w: int, contrast: float, seed: int) -> np.ndarray:
    """Multi-octave zero-mean 노이즈 (float32, std=contrast)."""
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), dtype=np.float32)
    long_side = max(h, w)
    for cells_long, amp in [(8, 1.0), (32, 0.7), (128, 0.5), (512, 0.3)]:
        ch = max(2, int(round(cells_long * h / long_side)))
        cw = max(2, int(round(cells_long * w / long_side)))
        n = rng.standard_normal((ch, cw)).astype(np.float32)
        n = cv2.resize(n, (w, h), interpolation=cv2.INTER_LINEAR)
        img += amp * n
    img = (img - img.mean()) / max(float(img.std()), 1e-6)
    return contrast * img


def make_grass(h: int, w: int, base_bgr: tuple[int, int, int],
               contrast: float, seed: int) -> np.ndarray:
    """잔디 느낌 BGR: 초록 base 에 공통 luma 노이즈 + 어두운 specks.

    노이즈를 세 채널에 같은 부호로 실어(밝기 변화 위주) mono 카메라에서도
    자연스러운 잔디 질감이 나온다. specks (~0.3 %) 는 흙/그림자 점.
    """
    rng = np.random.default_rng(seed + 1)
    luma = make_luma_noise(h, w, contrast, seed)
    out = np.zeros((h, w, 3), dtype=np.float32)
    # 채널별 가중: 잔디는 G 변동이 가장 큼.
    for ch, gain in enumerate((0.7, 1.0, 0.6)):     # B, G, R
        out[:, :, ch] = base_bgr[ch] + gain * luma
    out = np.clip(out, 0, 255).astype(np.uint8)

    speckle = rng.random((h, w)) > 0.997
    dark = np.clip(out[speckle].astype(np.int16) - 60, 0, 255).astype(np.uint8)
    out[speckle] = dark
    return out


def world_to_pixel(
    x: float, y: float, depth_m: float, px_per_m: float
) -> tuple[int, int]:
    """world (x,y) → image (col, row).

    image row 0 은 world y = depth_m (격자 북쪽 끝),
    image row H-1 은 world y = 0 (남쪽 끝).
    Gazebo plane 의 UV 매핑이 V-flip 되어 들어오면 cv2.flip(img, 0) 한 줄로 보정.
    """
    col = int(round(x * px_per_m))
    row = int(round((depth_m - y) * px_per_m))
    return col, row


def draw_grid(
    img: np.ndarray,
    width_m: float,
    depth_m: float,
    cell_m: float,
    line_w_m: float,
    px_per_m: float,
    line_bgr: tuple[int, int, int],
    marker_centers: list[tuple[float, float]],
    marker_size_m: float,
) -> None:
    """In-place: 격자선(BGR)을 그리되, marker 정사각 영역엔 그리지 않는다."""
    h, w = img.shape[:2]

    no_grid = np.zeros((h, w), dtype=bool)
    half = marker_size_m / 2.0
    for cx, cy in marker_centers:
        c0, _ = world_to_pixel(cx - half, cy + half, depth_m, px_per_m)  # top-left
        c1, _ = world_to_pixel(cx + half, cy + half, depth_m, px_per_m)
        _, r0 = world_to_pixel(cx - half, cy + half, depth_m, px_per_m)
        _, r1 = world_to_pixel(cx - half, cy - half, depth_m, px_per_m)
        c0c, c1c = max(0, min(c0, c1)), min(w, max(c0, c1))
        r0c, r1c = max(0, min(r0, r1)), min(h, max(r0, r1))
        no_grid[r0c:r1c, c0c:c1c] = True

    half_pw = max(1, int(round(line_w_m * px_per_m / 2.0)))
    line_px = np.array(line_bgr, dtype=np.uint8)

    # 세로 격자선 (X = 0, cell, 2*cell, ..., width)
    n_x = int(np.floor(width_m / cell_m + 1e-9))
    xs = [i * cell_m for i in range(n_x + 1)]
    if abs(xs[-1] - width_m) > 1e-6:
        xs.append(width_m)
    for x in xs:
        col = int(round(x * px_per_m))
        c0, c1 = max(0, col - half_pw), min(w, col + half_pw + 1)
        if c1 > c0:
            keep = no_grid[:, c0:c1]
            region = img[:, c0:c1]
            region[~keep] = line_px

    # 가로 격자선 (Y = 0, cell, 2*cell, ..., depth)
    n_y = int(np.floor(depth_m / cell_m + 1e-9))
    ys = [i * cell_m for i in range(n_y + 1)]
    if abs(ys[-1] - depth_m) > 1e-6:
        ys.append(depth_m)
    for y in ys:
        row = int(round((depth_m - y) * px_per_m))
        r0, r1 = max(0, row - half_pw), min(h, row + half_pw + 1)
        if r1 > r0:
            keep = no_grid[r0:r1, :]
            region = img[r0:r1, :]
            region[~keep] = line_px


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--layout", required=True, help="aruco_layout.yaml 경로")
    p.add_argument("--width", type=float, default=30.0)
    p.add_argument("--depth", type=float, default=21.0)
    p.add_argument("--cell", type=float, default=3.0)
    p.add_argument("--line-width", type=float, default=0.10)
    p.add_argument("--px-per-m", type=float, default=100.0)
    p.add_argument(
        "--grass-bgr", default="45,105,55",
        help="잔디 base 색 B,G,R (0..255)",
    )
    p.add_argument("--contrast", type=float, default=18.0, help="노이즈 진폭 0..50")
    p.add_argument(
        "--line-bgr", default="245,245,245",
        help="격자선 색 B,G,R (하얀 새틴 리본)",
    )
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("-o", "--output", required=True)
    args = p.parse_args()

    h = int(round(args.depth * args.px_per_m))
    w = int(round(args.width * args.px_per_m))
    print(f"texture size = {w} x {h} px  (= {args.width} x {args.depth} m @ {args.px_per_m} px/m)")

    marker_size, markers = parse_layout(args.layout)
    print(f"markers loaded: {len(markers)}, size={marker_size} m, centers={markers}")

    grass_bgr = tuple(int(v) for v in args.grass_bgr.split(","))
    line_bgr = tuple(int(v) for v in args.line_bgr.split(","))

    img = make_grass(h, w, grass_bgr, args.contrast, args.seed)
    draw_grid(
        img,
        width_m=args.width,
        depth_m=args.depth,
        cell_m=args.cell,
        line_w_m=args.line_width,
        px_per_m=args.px_per_m,
        line_bgr=line_bgr,
        marker_centers=markers,
        marker_size_m=marker_size,
    )

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(args.output, img)
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
