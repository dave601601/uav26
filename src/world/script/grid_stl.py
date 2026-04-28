"""격자 바닥 STL 생성기.

본 사양 (260429_world.md):
  python3 grid_stl.py --width 30 --depth 20 --cell 4 --line-width 0.10 \
                     --thickness 0.005 -o ../mesh/grid_30x20_t0.10_cell4.stl

원점 (0,0,0) 이 격자의 한 모서리 (=ground level) 가 되며,
mesh 는 +X / +Y 방향으로 width × depth 만큼 뻗는다.
Z 방향으로는 thickness 만큼 살짝 솟아 있다 (바닥 위 painted line 모사).

mode="grid" : 격자선만 (가로/세로 빔이 교차)
mode="slab" : 단순 평판 (디버그용)
"""
from __future__ import annotations

import argparse
import os

import numpy as np
import trimesh


def make_slab(width: float, depth: float, thickness: float) -> trimesh.Trimesh:
    box = trimesh.creation.box(extents=(width, depth, thickness))
    box.apply_translation((width / 2.0, depth / 2.0, thickness / 2.0))
    return box


def _gridline_positions(extent: float, cell: float) -> list[float]:
    """[0, cell, 2*cell, ..., last <= extent] + [extent] 의 정렬·중복제거 리스트."""
    n = int(np.floor(extent / cell + 1e-9))
    xs = [i * cell for i in range(n + 1)]
    if abs(xs[-1] - extent) > 1e-6:
        xs.append(extent)
    return xs


def make_grid(
    width: float,
    depth: float,
    cell: float,
    line_width: float,
    thickness: float,
) -> trimesh.Trimesh:
    """가로/세로 격자선의 합."""
    beams: list[trimesh.Trimesh] = []

    # 가로선 (Y 방향으로 늘어선 X-축 빔)
    for y in _gridline_positions(depth, cell):
        b = trimesh.creation.box(extents=(width, line_width, thickness))
        b.apply_translation((width / 2.0, y, thickness / 2.0))
        beams.append(b)

    # 세로선 (X 방향으로 늘어선 Y-축 빔)
    for x in _gridline_positions(width, cell):
        b = trimesh.creation.box(extents=(line_width, depth, thickness))
        b.apply_translation((x, depth / 2.0, thickness / 2.0))
        beams.append(b)

    # boolean union 은 manifold3d 가 필요하므로 단순 concat.
    # STL viewer / Gazebo 모두 잘 처리하며, 같은 material 이면 시각적 차이 없음.
    return trimesh.util.concatenate(beams)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["grid", "slab"], default="grid")
    parser.add_argument("--width", type=float, default=30.0, help="X 방향 길이 [m]")
    parser.add_argument("--depth", type=float, default=20.0, help="Y 방향 길이 [m]")
    parser.add_argument("--cell", type=float, default=4.0, help="격자 한 칸 크기 [m]")
    parser.add_argument(
        "--line-width", type=float, default=0.10, help="격자선 폭 [m] (grid 전용)"
    )
    parser.add_argument(
        "--thickness", type=float, default=0.005, help="Z 방향 두께 [m]"
    )
    parser.add_argument("-o", "--output", required=True, help="출력 STL 경로")
    args = parser.parse_args()

    if args.mode == "slab":
        mesh = make_slab(args.width, args.depth, args.thickness)
    else:
        mesh = make_grid(
            args.width, args.depth, args.cell, args.line_width, args.thickness
        )

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    mesh.export(args.output)

    print(f"saved: {args.output}")
    print(f"triangles: {len(mesh.faces)}")
    print(f"bounds: {mesh.bounds}")


if __name__ == "__main__":
    main()
