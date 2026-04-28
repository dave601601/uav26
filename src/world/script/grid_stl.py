"""
4m x 4m, 두께 10cm 격자 STL 생성기

사용:
  - mode="slab"  : 그냥 평판 (4m x 4m x 0.1m)
  - mode="grid"  : 격자 구조 (가로세로 빔이 교차하는 형태)
"""
import trimesh
import numpy as np

# ---------- 파라미터 (단위: meter) ----------
WIDTH  = 4.0     # X 방향
DEPTH  = 4.0     # Y 방향
THICK  = 0.1     # 두께 (Z) = 10cm
MODE   = "slab"  # "slab" or "grid"

# grid mode 전용
CELL    = 0.5    # 격자 한 칸 크기 (m), 4m / 0.5m = 8x8 셀
BEAM_W  = 0.05   # 빔 두께 (m) = 5cm
# --------------------------------------------

def make_slab():
    box = trimesh.creation.box(extents=(WIDTH, DEPTH, THICK))
    # 원점이 중심이라 한쪽 모서리를 (0,0,0)으로 옮김
    box.apply_translation((WIDTH/2, DEPTH/2, THICK/2))
    return box

def make_grid():
    """가로/세로 빔이 격자로 교차하는 구조"""
    beams = []

    # X 방향 빔 (Y축을 따라 늘어선 가로 빔들)
    n_y = int(round(DEPTH / CELL)) + 1
    for i in range(n_y):
        y = i * CELL
        b = trimesh.creation.box(extents=(WIDTH, BEAM_W, THICK))
        b.apply_translation((WIDTH/2, y, THICK/2))
        beams.append(b)

    # Y 방향 빔 (X축을 따라 늘어선 세로 빔들)
    n_x = int(round(WIDTH / CELL)) + 1
    for i in range(n_x):
        x = i * CELL
        b = trimesh.creation.box(extents=(BEAM_W, DEPTH, THICK))
        b.apply_translation((x, DEPTH/2, THICK/2))
        beams.append(b)

    # boolean union으로 합쳐도 되지만 STL은 그냥 concat해도 슬라이서/뷰어에서 잘 처리함
    # union이 필요하면: mesh = trimesh.boolean.union(beams) - manifold3d 필요
    mesh = trimesh.util.concatenate(beams)
    return mesh

if __name__ == "__main__":
    if MODE == "slab":
        mesh = make_slab()
        out = "/mnt/user-data/outputs/slab_4x4x0.1m.stl"
    else:
        mesh = make_grid()
        out = "/mnt/user-data/outputs/grid_4x4_t0.1_cell0.5_beam0.05.stl"

    mesh.export(out)
    print(f"saved: {out}")
    print(f"triangles: {len(mesh.faces)}")
    print(f"bounds: {mesh.bounds}")
