#!/usr/bin/env python3
"""Measure the corridor's drivable band vs where the global path actually runs.

Question: at the stall (AMCL 2.31,0.33) the robot's footprint had 111/241 cells
at cost>=99 and center_raw hit 99. Is the corridor lane the planner chose simply
too tight, i.e. is the *geometry* the bug rather than the heading?
"""
import sys

import numpy as np

yaml_path = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/rufus-dargon/projects/semnav/maps/apartment.yaml'

res, ox, oy, img = None, None, None, None
for line in open(yaml_path):
    s = line.strip()
    if s.startswith('resolution:'):
        res = float(s.split(':')[1])
    elif s.startswith('origin:'):
        vals = s.split(':', 1)[1].strip().strip('[]').split(',')
        ox, oy = float(vals[0]), float(vals[1])
    elif s.startswith('image:'):
        img = s.split(':', 1)[1].strip()

pgm = img if img.startswith('/') else yaml_path.rsplit('/', 1)[0] + '/' + img
raw = open(pgm, 'rb').read()
# P5 header: magic, whitespace/comments, w, h, maxval
tok, i = [], 2
while len(tok) < 3 and i < len(raw):
    c = raw[i:i + 1]
    if c in b' \t\r\n':
        i += 1
    elif c == b'#':
        while raw[i:i + 1] not in (b'\n', b''):
            i += 1
    else:
        j = i
        while raw[j:j + 1] not in b' \t\r\n' and j < len(raw):
            j += 1
        tok.append(int(raw[i:j]))
        i = j
w, h = tok[0], tok[1]
i += 1
pgm_data = np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w)
occ = (pgm_data[::-1].T < 90)          # (nx, ny) grid, X-flip like the map
nx, ny = occ.shape
print(f'地图 {nx}x{ny} res={res} origin=({ox},{oy}) 已占 {occ.sum()} 格')

try:
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res
    have_dt = True
except ImportError:
    dist, have_dt = None, False
    print('无 scipy，改用逐格暴力最近距离')
    occs = np.argwhere(occ)
    step = 400
    dist = np.zeros_like(occ, dtype=float)
    for a in range(0, nx, step):
        sub = np.arange(a, min(a + step, nx))
        d = np.sqrt((sub[:, None, None] - occs[None, :, 0]) ** 2 +
                    (np.arange(ny)[None, None, :] - occs[None, :, 1]) ** 2).min(axis=1)
        dist[a:min(a + step, nx)] = d * res

R = 0.1745


def to_cell(x, y):
    return int(round((x - ox) / res)), int(round((y - oy) / res))


print('\n=== 走廊 x 方向：可行驶带（净空 >= 机器人半径 %.4f m）===' % R)
print('    x     自由 y 范围            可行驶中心 y 带        带宽   最大净空')
for x in np.arange(2.00, 3.65, 0.10):
    cx, _ = to_cell(x, 0.0)
    if not (0 <= cx < nx):
        continue
    col_free = ~occ[cx] & (pgm_data[::-1].T[cx] < 250) if False else ~occ[cx]
    free_ys = np.where(col_free)[0]
    if len(free_ys) == 0:
        continue
    ok = np.where(col_free & (dist[cx] >= R))[0]
    f_lo, f_hi = (free_ys.min() * res + oy, free_ys.max() * res + oy)
    if len(ok):
        c_lo, c_hi = ok.min() * res + oy, ok.max() * res + oy
        band = ok.max() - ok.min() + 1
        best = int(ok[np.argmax(dist[cx][ok])])
        print(f' {x:5.2f}  [{f_lo:6.2f},{f_hi:6.2f}]   '
              f'[{c_lo:6.2f},{c_hi:6.2f}]  {band*res:5.2f} m  '
              f'{dist[cx][best]:.3f} m @ y={best*res+oy:.2f}')
    else:
        print(f' {x:5.2f}  [{f_lo:6.2f},{f_hi:6.2f}]   无可行中心（带宽 0）')

print('\n=== 停滞点附近逐点净空 ===')
for (px, py, tag) in [(2.306, 0.325, '停滞点(probe5实测)'), (2.26, 0.24, '全局路径 lane'),
                      (2.60, 0.22, '路径中段'), (3.05, 0.60, '厨房门口'),
                      (3.20, 0.00, '走廊·厨房门')]:
    cx, cy = to_cell(px, py)
    if 0 <= cx < nx and 0 <= cy < ny:
        print(f'  {tag:22s} ({px:.2f},{py:.2f}) -> 格({cx},{cy}) '
              f'净空 {dist[cx][cy]:.3f} m  已占={bool(occ[cx][cy])}  '
              f'{"够" if dist[cx][cy] >= R else "★不够，机器人塞不进"}')
    else:
        print(f'  {tag:22s} ({px:.2f},{py:.2f}) 超出地图')
