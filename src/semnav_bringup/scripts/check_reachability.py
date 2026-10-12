#!/usr/bin/env python3
"""开跑前的可达性预检：在建好的地图上验证每个目标点是否真的到得了。

为什么需要这个脚本
------------------
"目标点坐标没问题"和"机器人到得了"是两件事。实测踩过的坑：

  * 次卧曾被家具封死（书桌与隔墙只剩 0.29 m 通道，车体直径 0.349 m），
    目标点本身在空闲栅格上、净空 0.34 m，看着完全正常，但**物理上进不去**。
    现象是规划器反复绕道、80~180 s 后超时，极易误判成"导航栈坏了"。
  * NavFn 配了 allow_unknown=True，会把未知区域当可通行。若只在"连通性"
    上判可达，仍可能规划出一条穿墙路径。

本脚本按**机器人半径**做栅格膨胀后做连通域标记，判断目标点是否与出生点同域。
快速、确定性、不依赖仿真。

用法
----
    python3 check_reachability.py --map ~/projects/semnav/maps/apartment.yaml \
        --layout ~/projects/semnav/src/semnav_bringup/config/apartment.yaml

退出码 0 = 全部可达；1 = 存在不可达目标（此时不要开跑导航）。
"""

import argparse
import sys

import numpy as np
import yaml

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover
    sys.exit('需要 scipy：pip3 install scipy')


def load_map(path):
    """读取 map_saver 产出的 PGM+YAML。

    返回 (occ, res, origin)。occ[ix, iy] —— 索引即世界坐标方向：
    ix 随 x 增大，iy 随 y 增大。

    ⚠️ PGM 行 0 是图像顶部（对应最大 y），而 ROS 地图原点在左下角，
    所以必须翻转行序，否则整张图上下颠倒。
    ⚠️ 三值化地图里 205 是"未知"而**不是**空闲；判空闲要用 >=250。
    """
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    pgm_path = path.rsplit('/', 1)[0] + '/' + meta['image']
    raw = open(pgm_path, 'rb').read()
    parts = raw.split(b'\n', 3)
    if parts[0].strip() != b'P5':
        sys.exit(f'只支持 P5 二进制 PGM，看到的是 {parts[0]!r}')
    width, height = map(int, parts[1].split())
    data = np.frombuffer(parts[3][:width * height], dtype=np.uint8).reshape(height, width)
    occ = np.ascontiguousarray(data[::-1].T)
    origin = meta['origin']
    return occ, float(meta['resolution']), (float(origin[0]), float(origin[1]))


def main():
    ap = argparse.ArgumentParser(description='导航目标点可达性预检')
    ap.add_argument('--map', required=True, help='map_saver 产出的地图 YAML')
    ap.add_argument('--layout', required=True, help='布局 YAML（含 spawn 与 nav_goals）')
    ap.add_argument('--robot-radius', type=float, default=None,
                    help='车体半径 m（缺省从 nav2_params.yaml 的 robot_radius 读）')
    ap.add_argument('--nav2-params', default=None, help='用于读取 robot_radius')
    ap.add_argument('--allow-unknown', action='store_true',
                    help='按建图期语义判可达（未知当可通行）。缺省按导航期'
                         '语义（未知当致命），见文件头注释。')
    ap.add_argument('--draw', default=None, help='可选：输出 PNG 预览路径')
    args = ap.parse_args()

    occ, res, (ox, oy) = load_map(args.map)
    nx, ny = occ.shape

    radius = args.robot_radius
    if radius is None and args.nav2_params:
        params = yaml.safe_load(open(args.nav2_params, encoding='utf-8'))
        radius = float(params['local_costmap']['local_costmap']
                       ['ros__parameters']['robot_radius'])
    if radius is None:
        radius = 0.1745
        print(f'⚠️  未指定车体半径，用默认 {radius} m')

    layout = yaml.safe_load(open(args.layout, encoding='utf-8'))
    spawn = layout['spawn']
    goals = layout['nav_goals']

    # 占用判据：三值图中 0=占用、205=未知、254=空闲
    #
    # ⚠️ 未知**不能**当空闲。曾经写成 `free = ~occupied`（即把 205 也算自由），
    # 于是未观测区看起来四通八达，预检给一个实际上过不去的目标点发了绿灯：
    # 它报告厨房目标净空 0.300~0.950 m "✓ 可达"，而严格语义的 A* 证明
    # **根本不存在路径** —— 那 0.9 m 净空量在一片从没被激光看到的空地上。
    # 预检放行 → 真去跑 → 撞墙 4 分钟 → 得出"导航栈有问题"的错误结论。
    occupied = occ < 90
    unknown = (occ >= 90) & (occ < 250)
    # 默认按导航期语义（allow_unknown=false）判：未知一律不可通行。
    # --allow-unknown 用于建图期语义（探索时必须能规划进未知区），
    # 此时会额外提示哪些目标点其实踩在未知区上、建图跑必须覆盖到。
    free = ~occupied & ~unknown if not args.allow_unknown else ~occupied

    # 按车体半径膨胀：dist 是每个自由栅格到最近**障碍**的距离（米）。
    # 注意膨胀只从占用栅格发起：真实 costmap 里未知栅格不是致命障碍，
    # 它不产生膨胀代价（这是"未知当致命"与"未知当可通行"唯一的差别）。
    dist = ndimage.distance_transform_edt(~occupied) * res
    passable = free & (dist >= radius)
    labels, n_comp = ndimage.label(passable, structure=np.ones((3, 3)))

    def idx(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    sx, sy = idx(spawn['x'], spawn['y'])
    spawn_comp = labels[sx, sy] if (0 <= sx < nx and 0 <= sy < ny) else 0

    print(f'地图    : {nx} x {ny} 格 = {nx * res:.2f} x {ny * res:.2f} m  '
          f'原点 ({ox:.3f}, {oy:.3f})  分辨率 {res:.3f}')
    print(f'车体半径: {radius:.4f} m（直径 {2 * radius:.4f} m）')
    print(f'连通域  : {n_comp} 个；出生点 ({spawn["x"]:.2f}, {spawn["y"]:.2f}) '
          f'属域 {spawn_comp}（{int((labels == spawn_comp).sum()) * res * res:.2f} m²）')
    print()

    bad = []
    unknown_goals = []
    for g in goals:
        gx, gy = idx(g['x'], g['y'])
        inside = 0 <= gx < nx and 0 <= gy < ny
        comp = labels[gx, gy] if inside else 0
        clearance = dist[gx, gy] if inside else 0.0
        on_unknown = bool(inside and 90 <= occ[gx, gy] < 250)
        if comp == 0:
            if on_unknown and not args.allow_unknown:
                verdict = '✗ 在未知区'
                why = '该点从未被激光观测到 —— 先建图覆盖它'
            else:
                verdict = '✗ 不可通行'
                why = '该点自身在障碍/膨胀区内' if inside else '该点在地图范围外'
        elif comp != spawn_comp:
            verdict = f'✗ 孤立（域 {comp}）'
            why = '与出生点不连通，机器人过不去'
        else:
            verdict = '✓ 可达'
            why = ''
        label = g.get('label') or g.get('id', '?')
        print(f'  {label:<12} ({g["x"]:6.2f},{g["y"]:6.2f})  '
              f'净空 {clearance:5.3f} m  {verdict}  {why}')
        if comp != spawn_comp or comp == 0:
            bad.append(label)
        if on_unknown:
            unknown_goals.append(label)

    if args.draw:
        render(args.draw, occ, res, ox, oy, labels, spawn_comp, radius, layout)

    if args.allow_unknown and unknown_goals:
        print(f'⚠️  目标点踩在未知区（建图期语义下放行，导航期会不可达）：'
              f'{", ".join(unknown_goals)}')
        print('   建图跑必须让激光真正观测到这些点所在的空间。')
    print()
    if bad:
        print(f'✗ {len(bad)} 个目标不可达：{", ".join(bad)}')
        print('  不要开跑导航 —— 会浪费时间并产出误导性的"导航失败"结论。')
        print('  先修世界几何（挪家具/改目标点）、或让建图跑覆盖到那些区域。')
        print('  排查提示：用 --draw 出图看目标点周围是不是被家具堵死；')
        print('  典型原因是家具与墙/家具之间的缝隙小于车体直径。')
        return 1
    print(f'✓ 全部 {len(goals)} 个目标可达')
    return 0


def render(path, occ, res, ox, oy, labels, spawn_comp, radius, layout):
    """输出预览 PNG：灰=未知/墙外，白=空闲，黑=占用，绿=可达区，橙=出生，绿点=目标。

    注意索引约定：occ 的形状是 (nx, ny)（第一维是 x），渲染用的图像数组是
    (行=y, 列=x)。两者互为转置，写错就会得到 IndexError 或镜像图。
    """
    import struct
    import zlib

    nx, ny = occ.shape
    occT = np.ascontiguousarray(occ.T)          # -> (ny, nx)
    assert occT.shape == (ny, nx)

    img = np.empty((ny, nx, 3), dtype=np.uint8)
    img[...] = (128, 128, 128)
    img[occT >= 250] = (255, 255, 255)
    img[(occT > 150) & (occT < 250)] = (180, 180, 180)
    img[occT < 90] = (0, 0, 0)

    reach = np.ascontiguousarray((labels == spawn_comp).T) if spawn_comp else None
    if reach is not None:
        img[reach] = (215, 255, 215)

    def mark(x, y, rgb, half=2):
        c, r = int((x - ox) / res), int((y - oy) / res)
        img[max(0, r - half):r + half + 1, max(0, c - half):c + half + 1] = rgb

    mark(layout['spawn']['x'], layout['spawn']['y'], (255, 140, 0), 3)
    for g in layout['nav_goals']:
        mark(g['x'], g['y'], (0, 170, 0), 3)

    scale = 5
    big = np.repeat(np.repeat(img, scale, 0), scale, 1)
    out = np.flipud(big)
    h, w = out.shape[:2]
    rows = b''.join(b'\x00' + out[i].tobytes() for i in range(h))

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload
                + struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(rows, 6))
           + chunk(b'IEND', b''))
    open(path, 'wb').write(png)
    print(f'\n预览图 -> {path}  ({w}x{h})')


if __name__ == '__main__':
    sys.exit(main())
