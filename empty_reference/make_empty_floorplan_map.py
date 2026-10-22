#!/usr/bin/env python3
"""Render a ROS occupancy map from an empty apartment layout + generated truth.

绘制顺序（关键，改过）：房间全幅铺 → 墙体覆盖 → 门洞回填。
原版是每个房间各自内缩 0.06 m 再画，遇到"两间相邻共边但该处没有墙"的情况
（起居室各分块之间、走廊与起居室之间）就会留下 0.12 m 宽的未知带；
改成全幅铺 + 墙体覆盖后，有墙的地方由墙覆盖，没墙的地方自然连成一片。
"""
from collections import deque
from pathlib import Path
import argparse
import json
import yaml
from PIL import Image

OCC, FREE, UNKNOWN = 0, 254, 205


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="apartment_empty_reference.yaml")
    ap.add_argument("--truth", default="apartment_empty_reference_truth.json")
    ap.add_argument("--out", default="apartment_empty_reference.pgm")
    ap.add_argument("--yaml", dest="yaml_out", default="apartment_empty_reference_map.yaml")
    ap.add_argument("--png", default="apartment_empty_reference.png")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=1.5,
                    help="unknown border kept around the plan, in metres"
                         " (was hardcoded 0.15, i.e. only 3 cells)")
    args = ap.parse_args()

    layout = yaml.safe_load(Path(args.layout).read_text(encoding="utf-8"))
    truth = json.loads(Path(args.truth).read_text(encoding="utf-8"))
    res = args.resolution
    bb = truth["bounding_box"]
    m = args.margin
    xmin, xmax = bb["x_min"] - m, bb["x_max"] + m
    ymin, ymax = bb["y_min"] - m, bb["y_max"] + m
    width = round((xmax - xmin) / res)
    height = round((ymax - ymin) / res)
    image = Image.new("L", (width, height), UNKNOWN)
    pixels = image.load()

    def cell(x, y):
        return round((x - xmin) / res), height - 1 - round((y - ymin) / res)

    def rect(x0, x1, y0, y1, value):
        ax, ay = cell(x0, y1)
        bx, by = cell(x1, y0)
        for py in range(max(0, ay), min(height, by + 1)):
            for px in range(max(0, ax), min(width, bx + 1)):
                pixels[px, py] = value

    # 1) 房间全幅铺自由格（不内缩）
    for room in layout["rooms"]:
        rect(room["x_min"], room["x_max"], room["y_min"], room["y_max"], FREE)
    # 2) 墙体覆盖
    for wall in truth["walls"]:
        rect(wall["x_min"], wall["x_max"], wall["y_min"], wall["y_max"], OCC)
    # 3) 门洞回填自由格
    for door in layout.get("doors", []):
        w = float(door.get("width", 0.9))
        if "wall_y" in door:
            rect(door["center_x"] - w / 2, door["center_x"] + w / 2,
                 door["wall_y"] - 0.10, door["wall_y"] + 0.10, FREE)
        else:
            rect(door["wall_x"] - 0.10, door["wall_x"] + 0.10,
                 door["center_y"] - w / 2, door["center_y"] + w / 2, FREE)

    # ---- 自检：室内不得残留被墙包死的未知空洞 ----
    seen = [[False] * width for _ in range(height)]
    q = deque()
    for px in range(width):
        for py in (0, height - 1):
            if not seen[py][px] and pixels[px, py] != OCC:
                seen[py][px] = True
                q.append((px, py))
    for py in range(height):
        for px in (0, width - 1):
            if not seen[py][px] and pixels[px, py] != OCC:
                seen[py][px] = True
                q.append((px, py))
    while q:
        px, py = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = px + dx, py + dy
            if 0 <= nx < width and 0 <= ny < height and not seen[ny][nx] and pixels[nx, ny] != OCC:
                seen[ny][nx] = True
                q.append((nx, ny))
    voids, void_area = 0, 0.0
    for py in range(height):
        for px in range(width):
            if not seen[py][px]:
                voids += 1
                if pixels[px, py] == UNKNOWN:
                    void_area += res * res
    free_cells = sum(1 for py in range(height) for px in range(width) if pixels[px, py] == FREE)

    image.save(args.out)
    image.save(args.png)
    Path(args.yaml_out).write_text(
        f"image: {Path(args.out).name}\nmode: trinary\nresolution: {res:.3f}\n"
        f"origin: [{xmin:.3f}, {ymin:.3f}, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8")
    print(f"generated {width}x{height} map, origin=({xmin:.2f},{ymin:.2f})")
    print(f"free {free_cells * res * res:.2f} m2  |  enclosed unknown {void_area:.2f} m2 "
          f"({voids} cells)")
    if void_area > 0.01:
        print("WARNING: 存在被墙包死的未知区域，导航不通行")


if __name__ == "__main__":
    main()
