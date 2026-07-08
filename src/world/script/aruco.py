"""ArUco DICT_6X6_250 마커 PNG 일괄 생성기.

Gazebo SDF 의 PBR `<diffuse>` 텍스처로 사용하기 위한 정사각 PNG 를 출력한다.
출력 경로: <out-dir>/aruco_<id>.png

사용:
  python3 aruco.py --ids 0,1,2,3,4,5,6,7,8 --out-dir ../textures
  python3 aruco.py --ids-from ../config/aruco_layout.yaml --out-dir ../textures
"""
from __future__ import annotations

import argparse
import os

import cv2
import cv2.aruco as aruco
import numpy as np

DICTIONARY = aruco.DICT_6X6_250


def parse_ids(spec: str) -> list[int]:
    out: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            a, b = chunk.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(chunk))
    return out


def parse_ids_from_yaml(path: str) -> list[int]:
    """layout YAML 의 markers[].id 를 추출. PyYAML 없어도 되도록 자체 파서.

    형식 예:
        markers:
          - id: 0
            pose: [4.0, 4.0, 0.001, 0, 0, 0]
    """
    ids: list[int] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("- id:") or stripped.startswith("id:"):
                # "- id: 12" or "id: 12"
                _, _, val = stripped.partition("id:")
                try:
                    ids.append(int(val.strip()))
                except ValueError:
                    pass
    return ids


def generate_marker_png(
    marker_id: int, size_px: int, out_path: str, margin_modules: float = 1.0
) -> None:
    """Marker code centered on a white quiet zone.

    generateImageMarker emits the black border flush to the image edge,
    but the ArUco detector REQUIRES a light quiet zone around the code
    to isolate the quad contour — a real printed marker always has one.
    Without it, a marker sitting on a grid intersection fuses with the
    black grid lines into one blob and the detector never even forms a
    candidate quad (this killed every oblique side-camera detection in
    r61/r63; the nadir camera survived only because at its scale the
    adaptive threshold happened to separate the thin lines).

    The plate stays size_px (0.5 m in the world); the code shrinks to
    8/(8 + 2*margin_modules) of it — 0.4 m for the default 1-module
    margin, mirroring how a 0.5 m sheet would actually be printed.
    """
    aruco_dict = aruco.getPredefinedDictionary(DICTIONARY)
    modules = 8  # 6x6 code + 1-module black border each side
    total = modules + 2.0 * margin_modules
    code_px = int(round(size_px * modules / total))
    img = aruco.generateImageMarker(aruco_dict, marker_id, code_px)
    canvas = 255 * np.ones((size_px, size_px), dtype=np.uint8)
    pad = (size_px - code_px) // 2
    canvas[pad:pad + code_px, pad:pad + code_px] = img
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    cv2.imwrite(out_path, canvas)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ids",
        default="",
        help="콤마 구분 ID 또는 범위 (예: 0,1,2 또는 0-8)",
    )
    parser.add_argument(
        "--ids-from",
        default="",
        help="layout YAML 파일에서 markers[].id 읽기",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="aruco_<id>.png 들을 떨어뜨릴 디렉토리",
    )
    parser.add_argument(
        "--size-px",
        type=int,
        default=512,
        help="PNG 한 변 픽셀 (기본 512)",
    )
    args = parser.parse_args()

    ids: list[int] = []
    if args.ids:
        ids.extend(parse_ids(args.ids))
    if args.ids_from:
        ids.extend(parse_ids_from_yaml(args.ids_from))
    if not ids:
        parser.error("--ids 또는 --ids-from 중 하나는 지정해야 함")

    seen = set()
    unique_ids = [i for i in ids if not (i in seen or seen.add(i))]

    for marker_id in unique_ids:
        out_path = os.path.join(args.out_dir, f"aruco_{marker_id}.png")
        generate_marker_png(marker_id, args.size_px, out_path)
        print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
