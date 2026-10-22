#!/usr/bin/env python3
"""Burn furniture footprints into a ROS occupancy PGM.

Usage: burn_furniture.py <layout.yaml> <truth.json> <in_map.yaml>
                         <out.pgm> <out_map.yaml>

Rasterisation rule: a cell is marked occupied if the cell's world-space square
ACTUALLY OVERLAPS the object's footprint rectangle.  The previous version used
floor/ceil of the footprint edges, which over-covers by up to one full cell on
the high side (it marks every cell the edge touches plus one more), so blocks
came out ~5 cm larger and shifted by half a cell.  Testing the overlap
explicitly is both exact and obviously correct.

Footprints come from generate_world.rotated_footprint(), the same function that
builds the SDF, so the map cannot disagree with the simulated world.
"""
import importlib.util
import math
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location(
    'gw', str(REPO / 'src/semnav_bringup/scripts/generate_world.py'))
gw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gw)


def load_pgm(path):
    raw = Path(path).read_bytes()
    parts, i = [], 0
    while len(parts) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        s = i
        while not raw[i:i + 1].isspace():
            i += 1
        parts.append(raw[s:i])
    i += 1
    w, h = int(parts[1]), int(parts[2])
    return w, h, bytearray(raw[i:i + w * h])


def save_pgm(path, w, h, data):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(bytes(data))


def cells_for_rect(x0, x1, y0, y1, res, ox, oy, w, h):
    """Cells whose world square overlaps [x0,x1] x [y0,y1]."""
    # cell c covers x in [ox + c*res, ox + (c+1)*res)
    c_lo = math.floor((x0 - ox) / res)
    c_hi = math.ceil((x1 - ox) / res) - 1
    # row r covers y in [oy + (h-1-r)*res, oy + (h-r)*res)
    r_lo = math.floor(h - 1 - (y1 - oy) / res) + 1
    r_hi = math.ceil(h - (y0 - oy) / res) - 1

    c_lo = max(0, c_lo)
    c_hi = min(w - 1, c_hi)
    r_lo = max(0, r_lo)
    r_hi = min(h - 1, r_hi)

    EPS = 1e-9
    out = []
    for r in range(r_lo, r_hi + 1):
        cy0 = oy + (h - 1 - r) * res
        cy1 = cy0 + res
        if cy1 <= y0 + EPS or cy0 >= y1 - EPS:
            continue
        base = r * w
        for c in range(c_lo, c_hi + 1):
            cx0 = ox + c * res
            cx1 = cx0 + res
            if cx1 <= x0 + EPS or cx0 >= x1 - EPS:
                continue
            out.append(base + c)
    return out


def main():
    layout_p, truth_p, in_yaml, out_pgm, out_yaml = sys.argv[1:6]
    lay = yaml.safe_load(Path(layout_p).read_text(encoding='utf-8'))
    my = yaml.safe_load(Path(in_yaml).read_text(encoding='utf-8'))
    res = float(my['resolution'])
    ox, oy = float(my['origin'][0]), float(my['origin'][1])

    src = Path(in_yaml).parent / my['image']
    if not src.is_file():
        src = Path(in_yaml).with_name(Path(my['image']).name)
    w, h, data = load_pgm(src)

    burned = 0
    for o in lay.get('objects') or []:
        fx0, fx1, fy0, fy1 = gw.rotated_footprint(o)
        idx = cells_for_rect(fx0, fx1, fy0, fy1, res, ox, oy, w, h)
        n = 0
        for i in idx:
            if data[i] >= 200:          # never overwrite an existing wall
                data[i] = 0
                n += 1
        burned += n
        print('  %-20s %-16s world x[%+.2f,%+.2f] y[%+.2f,%+.2f] -> %d cells'
              % (o.get('label', o['class']), o['class'], fx0, fx1, fy0, fy1, n))

    save_pgm(out_pgm, w, h, data)
    Path(out_yaml).write_text(
        'image: %s\n'
        'mode: trinary\n'
        'resolution: %.3f\n'
        'origin: [%.3f, %.3f, 0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
        % (Path(out_pgm).name, res, ox, oy), encoding='utf-8')

    free = sum(1 for b in data if b >= 250)
    occ = sum(1 for b in data if b < 90)
    print('\n%dx%d map, origin=(%.2f,%.2f)' % (w, h, ox, oy))
    print('free %.2f m2 | occupied %.2f m2 | furniture cells %d'
          % (free * res * res, occ * res * res, burned))


if __name__ == '__main__':
    main()
