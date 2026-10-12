#!/usr/bin/env python3
"""巡游路线预检：在建好的地图上逐段复算"规划器会怎么走"，并数出穿墙次数。

为什么需要它
------------
建图巡游由 Nav2 规划器驱动，而建图期的 `allow_unknown=true` 会把**未知**
栅格当成可通行。于是当两个目标点之间隔着一面"激光还没看到过"的墙时，
规划器会挑一条穿墙近路 —— 代价函数只算路径长度，穿墙比绕门短。
控制器推不过去，表现为反复 `Failed to make progress` 直到超时。

实测踩过：次卧(1.20,3.00) → 厨房(2.30,1.60)。x=+1.6 隔墙在 y∈[0.8,1.8]
一段被次卧书桌挡住，激光从未观测到，栅格值仍是"未知"；规划器从这段直接
横穿（3.6 m）而不是绕门（8.2 m），机器人卡死在书桌/墙之间，180 s 超时。

本脚本对每一段相邻点用 A* 复算，并**对照世界真值**（apartment_truth.json
的 walls）判断路径是否穿墙，输出两种规划语义下的结果：

    严格 = 未知视为致命（导航期 `allow_unknown=false` 的语义）
    宽松 = 未知视为可通行（建图期 `allow_unknown=true` 的语义）

几秒钟出结果，能在烧掉十几分钟仿真之前把路线缺陷挑出来。

用法
----
    python3 check_tour.py --map ~/projects/semnav/maps/apartment.yaml \
        --layout src/semnav_bringup/config/apartment.yaml \
        --truth install/semnav_bringup/share/semnav_bringup/config/apartment_truth.json

退出码 0 = 各段均不穿墙（宽松语义）；1 = 存在穿墙段。
"""

import argparse
import heapq
import sys

import numpy as np
import yaml


def load_map(path):
    """读取 map_saver 产出的 PGM+YAML，返回 (occ, res, (ox, oy))。

    occ[ix, iy] —— 索引方向即世界坐标方向：ix 随 x 增大，iy 随 y 增大。

    ⚠️ PGM 行 0 是图像顶部（对应最大 y），ROS 地图原点在左下角，必须翻转行序。
    ⚠️ 三值图里 205 是"未知"而**不是**空闲；判空闲要用 >=250。
    """
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    pgm_path = path.rsplit('/', 1)[0] + '/' + meta['image']
    raw = open(pgm_path, 'rb').read()
    parts = raw.split(b'\n', 3)
    if parts[0].strip() != b'P5':
        sys.exit(f'只支持 P5 二进制 PGM，看到的是 {parts[0]!r}')
    width, height = map(int, parts[1].split())
    data = np.frombuffer(parts[3][:width * height],
                         dtype=np.uint8).reshape(height, width)
    occ = np.ascontiguousarray(data[::-1].T)
    origin = meta['origin']
    return occ, float(meta['resolution']), (float(origin[0]), float(origin[1]))


def make_passable(occ, res, radius, unknown_ok):
    """返回 (passable, dist)，模拟规划器看到的可通行集合。

    dist = 到最近**占用**栅格的距离（米）。真实 costmap 里膨胀层只从致命
    栅格（占用）膨胀，未知栅格（255 NO_INFORMATION）不参与膨胀，所以两种
    语义下净空都按"到占用的距离"算。
    """
    from scipy import ndimage
    occupied = occ < 90
    unknown = (occ >= 90) & (occ < 250)
    allowed = ~occupied
    if not unknown_ok:
        allowed = allowed & ~unknown
    dist = ndimage.distance_transform_edt(~occupied) * res
    return allowed & (dist >= radius), dist


def astar(passable, start, goal):
    """8 邻域 A*（对角代价 √2，禁止切角）。返回 [(ix,iy), ...] 或 None。

    ⚠️ 每次索引邻居前都必须显式判边界。numpy 的下标取负是**回绕**而不是
    越界：passable[-1, cy] 会安静地读到最后一行的栅格。少了这道判断，A*
    会以为地图左边缘和右边缘是相邻的，从而"规划"出一条横穿整张图的假路径 ——
    预检会把一段实际无路的行程报成畅通。
    """
    nx, ny = passable.shape

    def ok(c):
        return 0 <= c[0] < nx and 0 <= c[1] < ny

    if not (ok(start) and ok(goal)):
        return None
    if not passable[start] or not passable[goal]:
        return None

    sq2 = 2 ** 0.5
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, sq2), (1, -1, sq2), (-1, 1, sq2), (-1, -1, sq2)]
    g = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    closed = set()
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            out = [cur]
            while cur in prev:
                cur = prev[cur]
                out.append(cur)
            return out[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        cx, cy = cur
        for dx, dy, cost in moves:
            nb = (cx + dx, cy + dy)
            if not ok(nb) or nb in closed or not passable[nb]:
                continue
            # 禁止斜穿墙角：两个正交邻居都得能走
            if dx and dy and not (passable[(cx + dx, cy)]
                                  and passable[(cx, cy + dy)]):
                continue
            ng = g[cur] + cost
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                prev[nb] = cur
                h = max(abs(goal[0] - nb[0]), abs(goal[1] - nb[1]))
                heapq.heappush(heap, (ng + h, nb))
    return None


def path_metrics(path, occ, res, walls):
    """返回 (长度m, 未知栅格占比, 穿过的墙 id 列表)。"""
    if not path:
        return 0.0, 0.0, []
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += res * (2 ** 0.5 if a[0] != b[0] and a[1] != b[1] else 1.0)
    unknown = sum(1 for c in path if 90 <= occ[c] < 250)
    hit = []
    for c in path:
        x = OX + c[0] * res
        y = OY + c[1] * res
        for w in walls:
            if (w['x_min'] + 0.03 < x < w['x_max'] - 0.03
                    and w['y_min'] + 0.03 < y < w['y_max'] - 0.03):
                if w['id'] not in hit:
                    hit.append(w['id'])
                break
    return length, unknown / len(path), hit


def main():
    global OX, OY
    ap = argparse.ArgumentParser(description='巡游路线穿墙预检')
    ap.add_argument('--map', required=True)
    ap.add_argument('--layout', required=True)
    ap.add_argument('--truth', default=None, help='真值 json（含 walls）')
    ap.add_argument('--tour', default=None,
                    help='用布局里的哪个列表当路线：mapping_tour / nav_goals'
                         '（缺省优先 mapping_tour）')
    ap.add_argument('--robot-radius', type=float, default=0.1745)
    args = ap.parse_args()

    layout = yaml.safe_load(open(args.layout, encoding='utf-8'))
    occ, res, (OX, OY) = load_map(args.map)

    if args.tour:
        key = args.tour
    else:
        key = 'mapping_tour' if layout.get('mapping_tour') else 'nav_goals'
    points = [{'name': '出生点', 'x': layout['spawn']['x'],
               'y': layout['spawn']['y']}]
    points += [{'name': g.get('label') or g.get('id') or f'#{i}', 'x': g['x'],
                'y': g['y']} for i, g in enumerate(layout.get(key, []))]

    walls = []
    if args.truth:
        import json
        walls = json.load(open(args.truth, encoding='utf-8'))['walls']

    lax, _ = make_passable(occ, res, args.robot_radius, unknown_ok=True)
    strict, _ = make_passable(occ, res, args.robot_radius, unknown_ok=False)

    def idx(x, y):
        return int(round((x - OX) / res)), int(round((y - OY) / res))

    print(f'地图   : {occ.shape[0]}x{occ.shape[1]} 格 @ {res} m，'
          f'原点 ({OX:.3f}, {OY:.3f})')
    print(f'路线   : {key}，共 {len(points) - 1} 段'
          f'（含从出生点出发的第 1 段）')
    print(f'真值墙 : {len(walls)} 面'
          + ('' if walls else '  ⚠️ 未提供 --truth，无法判定穿墙'))
    print()

    hdr = (f'{"#":>2} {"段":<34} {"严格":>10} {"宽松":>10} '
           f'{"未知占比":>8}  {"穿墙":<8}')
    print(hdr)
    print('-' * len(hdr))

    bad = 0
    no_strict = 0
    for i, (a, b) in enumerate(zip(points, points[1:]), start=1):
        pa, pb = idx(a['x'], a['y']), idx(b['x'], b['y'])
        sa = astar(strict, pa, pb)
        la = astar(lax, pa, pb)
        seg = f'{a["name"]}({a["x"]:+.2f},{a["y"]:+.2f})→{b["name"]}'

        s_txt = f'{path_metrics(sa, occ, res, walls)[0]:.2f} m' if sa else '无路'
        if sa is None:
            no_strict += 1
        if la is None:
            print(f'{i:>2} {seg:<34} {s_txt:>10} {"无路":>10} {"--":>8}  ✗')
            bad += 1
            continue
        L, unk, hit = path_metrics(la, occ, res, walls)
        hit_txt = ('✓' if not hit else '✗ ' + ','.join(
            h.replace('wall_', '') for h in hit))
        if hit:
            bad += 1
        print(f'{i:>2} {seg:<34} {s_txt:>10} {L:>8.2f} m '
              f'{unk:>7.0%}  {hit_txt}')

    print()
    if bad:
        print(f'✗ {bad} 段在宽松语义下会穿墙 —— 建图巡游会卡死在这些路段上。')
        print('  改法：让每段的两端都落在走廊上（走廊是已知区域，穿墙近路被'
              '已知墙挡住），')
        print('  即"房间↔走廊"交替，不要"房间→房间"直达。')
        return 1
    if no_strict:
        print(f'⚠️  有 {no_strict} 段在严格语义下无路 —— 说明建图后仍有未观测区，'
              f'导航期（allow_unknown=false）会到不了。')
    print('✓ 各段均不穿墙')
    return 0


if __name__ == '__main__':
    sys.exit(main())
