"""ArUco 마커 PNG 일괄 생성기.

Gazebo SDF 의 PBR `<diffuse>` 텍스처로 사용하기 위한 정사각 PNG 를 출력한다.
출력 경로: <out-dir>/aruco_<id>.png

사용:
  python3 aruco.py --ids 0-49 --out-dir ../textures
  python3 aruco.py --ids-from ../config/aruco_layout.yaml --out-dir ../textures

Dictionary: 대회 공지는 "ID 0~49 중 4개"만 명시하고 dictionary 는 밝히지
않았다. 0~49 는 50-마커 dictionary (DICT_4X4_50 등) 와 정확히 일치하므로
기본값은 DICT_4X4_50 — 규정이 확정되면 --dict 한 번으로 교체한다.
"""
from __future__ import annotations

import argparse
import os

import cv2
import cv2.aruco as aruco
import numpy as np

# name -> (cv2 dictionary constant, marker modules per side incl. the
# 1-module black border on each side: NxN code + 2).
DICTIONARIES = {
    "4X4_50": (aruco.DICT_4X4_50, 6),
    "5X5_50": (aruco.DICT_5X5_50, 7),
    "6X6_50": (aruco.DICT_6X6_50, 8),
    "6X6_250": (aruco.DICT_6X6_250, 8),
    "7X7_50": (aruco.DICT_7X7_50, 9),
}
DEFAULT_DICT = "4X4_50"


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
    marker_id: int,
    size_px: int,
    out_path: str,
    dict_name: str = DEFAULT_DICT,
) -> None:
    """A plain ArUco marker: the code fills the whole 0.4 m sheet.

    Official spec (2026-07 survey notice): "크기 0.4m x 0.4m", "색상 :
    (바탕) 검정색, (마커) 하얀색". That describes a STANDARD ArUco, whose
    own field is black and whose data cells are white — not a negated
    one. `generateImageMarker` already produces exactly this, so nothing
    is inverted here and `line_tracer` detects it with OpenCV's default
    polarity (perception.aruco_white_on_black stays false).

    No margin. The sheet IS the code, all `modules` of it edge to edge,
    which is also how the rules read: 0.4 m is the marker's size. Padding
    it with extra BLACK would merge with the code's own black border ring
    — the detector would contour the 0.4 m sheet, then sample a module
    grid that no longer lines up with the code inside, and decode
    garbage. Padding it with WHITE is what the pre-2026-07-09 texture
    did, and the rules say the background is black.

    The quiet zone the detector needs is supplied by the FLOOR: the sheet
    sits on green grass (luma ~83) crossed by white 10 cm ribbons (~245),
    both far lighter than the code's black border. This is the reverse of
    the r61/r63 failure, where a black border met black grid lines on a
    white floor and fused into one blob.

    Side effect worth noting: at 0.4 m the module pitch is 0.4/6 =
    6.67 cm for DICT_4X4_50, a third larger than the 0.3 m code the
    1-module-margin texture carried. The lookahead's far band gains from
    that.
    """
    dict_const, _modules = DICTIONARIES[dict_name]
    aruco_dict = aruco.getPredefinedDictionary(dict_const)
    canvas = aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    cv2.imwrite(out_path, canvas)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ids",
        default="",
        help="콤마 구분 ID 또는 범위 (예: 0,1,2 또는 0-49)",
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
    parser.add_argument(
        "--dict",
        dest="dict_name",
        default=DEFAULT_DICT,
        choices=sorted(DICTIONARIES),
        help=f"ArUco dictionary (기본 {DEFAULT_DICT})",
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
        generate_marker_png(
            marker_id, args.size_px, out_path, dict_name=args.dict_name
        )
        print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
