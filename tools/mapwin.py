#!/usr/bin/env python3
"""ASCII 显示地图局部 + A* 路径叠加，用来判定"墙到底在不在"。

判读符号：  '#'=已占(<90)   '.'=自由(>=250)   '?'=未知(90..249)   'o'=A* 路径
坐标窗口由 --x0 --x1 --y0 --y1 给出（米，世界坐标）。
"""
import argparse
import heapq
import math

import numpy as np


def load_map(path):
    res = ox = oy = img = None
    for line in open(path):
        s = line.strip()
        if s.startswith('resolution:'):
            res = float(s.split(':')[1])
        elif s.startswith('origin:'):
            v = s.split(':', 1)[1].strip().strip('[]').split(',')
            ox, oy = float(v[0]), float(v[1])
        elif s.startswith('image:'):
            img = s.split(':', 1)[1].strip()
    pgm = img if img.startswith('/') else path.rsplit('/', 1)[0] + '/' + img
    raw = open(pgm, 'rb').read()
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
    img = np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w)
    # PGM 行序自上而下 = y 由大到小；转成 (nx, ny)、y 递增
    return img[::-1].T, res, ox, oy


def astar(occ, free_mask, res, start, goal, radius):
    nx, ny = occ.shape
    sc = (int(round((start[0] - ox) / res)), int(round((start[1] - oy) / res)))
    gc = (int(round((goal[0] - ox) / res)), int(round((goal[1] - oy) / res)))
    # 可通行 = 自由 且 到最近障碍的距离 >= radius
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res
    ok = free_mask & (dist >= radius)

    def inbox(c):
        return 0 <= c[0] < nx and 0 <= c[1] < ny

    if not inbox(sc) or not inbox(gc):
        return None, f'起/终点出图: {sc} {gc}'
    if not ok[sc]:
        # 起点不可行：找最近可行格
        cand = np.argwhere(ok)
        if len(cand) == 0:
            return None, '没有可行格'
        d = np.hypot(cand[:, 0] - sc[0], cand[:, 1] - sc[1])
        sc = tuple(cand[int(np.argmin(d))])
    if not ok[gc]:
        cand = np.argwhere(ok)
        d = np.hypot(cand[:, 0] - gc[0], cand[:, 1] - gc[1])
        gc = tuple(cand[int(np.argmin(d))])

    INF = float('inf')
    g = {sc: 0.0}
    prev = {}
    pq = [(0.0, sc)]
    seen = set()
    while pq:
        _, c = heapq.heappop(pq)
        if c in seen:
            continue
        seen.add(c)
        if c == gc:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                n = (c[0] + dx, c[1] + dy)
                if not inbox(n) or not ok[n]:
                    continue
                if dx != 0 and dy != 0:
                    if not ok[(c[0] + dx, c[1])] or not ok[(c[0], c[1] + dy)]:
                        continue
                step = res * (math.sqrt(2) if dx and dy else 1.0)
                ng = g[c] + step
                if ng < g.get(n, INF):
                    g[n] = ng
                    prev[n] = c
                    hh = res * math.hypot(n[0] - gc[0], n[1] - gc[1])
                    heapq.heappush(pq, (ng + hh, n))
    if gc not in prev and gc != sc:
        return None, '无路'
    path = [gc]
    while path[-1] != sc:
        path.append(prev[path[-1]])
    return path[::-1], None


ap = argparse.ArgumentParser()
ap.add_argument('--map', default='/home/rufus-dargon/projects/semnav/maps/apartment.yaml')
ap.add_argument('--x0', type=float, default=1.6)
ap.add_argument('--x1', type=float, default=3.6)
ap.add_argument('--y0', type=float, default=-0.2)
ap.add_argument('--y1', type=float, default=1.9)
ap.add_argument('--start', default='2.31,0.33')
ap.add_argument('--goal', default='2.30,1.60')
ap.add_argument('--radius', type=float, default=0.1745)
ap.add_argument('--unknown-ok', action='store_true')
a = ap.parse_args()

occ_raw, res, ox, oy = load_map(a.map)
occupied = occ_raw < 90
unknown = (occ_raw >= 90) & (occ_raw < 250)
free = occ_raw >= 250
print(f'地图 {occ_raw.shape} res={res} origin=({ox},{oy})')
print(f'窗口 x[{a.x0},{a.x1}] y[{a.y0},{a.y1}]   机器人半径 {a.radius} m')
print(f'未知格是否算可通行: {a.unknown_ok}')

print('\n=== 原始栅格（# 已占 / . 自由 / ? 未知）===')
cx0 = int(round((a.x0 - ox) / res)); cx1 = int(round((a.x1 - ox) / res))
cy0 = int(round((a.y0 - oy) / res)); cy1 = int(round((a.y1 - oy) / res))
print('        ' + ''.join(
    str(int(round((x * res + ox) * 10)) % 10) for x in range(cx0, cx1 + 1)))
print('        ' + ''.join(
    ('|' if abs((x * res + ox) - round(x * res + ox)) < 1e-6 else ' ')
    for x in range(cx0, cx1 + 1)))
for cy in range(cy1, cy0 - 1, -1):
    row = ''.join('#' if occupied[cx, cy] else ('.' if free[cx, cy] else '?')
                  for cx in range(cx0, cx1 + 1))
    print(f'  y={cy*res+oy:6.2f} {row}')

start = tuple(float(v) for v in a.start.split(','))
goal = tuple(float(v) for v in a.goal.split(','))
passable = (free | unknown) if a.unknown_ok else free
path, err = astar(occ_raw, passable, res, start, goal, a.radius)
print(f'\n=== A* {start} -> {goal}  未知算{"可通行" if a.unknown_ok else "障碍"} ===')
if err:
    print(f'  {err}')
else:
    pts = [(p[0] * res + ox, p[1] * res + oy) for p in path]
    print(f'  {len(pts)} 点  长度 {sum(math.dist(pts[i], pts[i+1]) for i in range(len(pts)-1)):.2f} m')
    on_unk = sum(1 for p in path if unknown[p])
    print(f'  经过未知格 {on_unk}/{len(path)} = {100*on_unk/len(path):.1f}%')
    print('  前 8 点: ' + ' '.join(f'({x:.2f},{y:.2f})' for x, y in pts[:8]))
    # 叠加显示
    print('\n=== 叠加 A* 路径（o）===')
    pmap = {p for p in path}
    for cy in range(cy1, cy0 - 1, -1):
        row = ''
        for cx in range(cx0, cx1 + 1):
            if (cx, cy) in pmap:
                row += 'o'
            elif occupied[cx, cy]:
                row += '#'
            elif free[cx, cy]:
                row += '.'
            else:
                row += '?'
        print(f'  y={cy*res+oy:6.2f} {row}')
    # 逐段方向，看它是否"北上穿墙"
    print('\n  路径分段方向：')
    for i in range(0, len(pts) - 1, max(1, len(pts) // 10)):
        x0_, y0_ = pts[i]
        x1_, y1_ = pts[min(i + 1, len(pts) - 1)]
        ang = math.degrees(math.atan2(y1_ - y0_, x1_ - x0_))
        print(f'    #{i:3d} ({x0_:.2f},{y0_:.2f}) -> ({x1_:.2f},{y1_:.2f})  {ang:7.1f}°')
