#!/usr/bin/env python3
"""按语义真值评估地图质量 —— 不靠肉眼看图，用数字说话。

三条指标，各自对应一个能证伪的命题：

  1. 配准误差（最重要）
     在 ±0.5 m 范围内搜索地图形变，看把真值几何平移到哪个位置时，
     「地图障碍落在真值几何附近」的比例最高。若最佳位移 ≈ (0,0) 且
     得分高，说明地图与真值坐标系一致、且没有系统性偏移。
     这一条替代了"整体错位"的肉眼判断。

  2. 物体检出率
     沿物体足印**周长**采样，检查附近是否存在障碍栅格。
     为什么不用"足印内部有多少障碍格"：激光只能看到物体外表面，
     物体内部本来就应该是空的。早期版本按足印填充率算，得到
     "召回率 0%"这种荒谬数字 —— 错的是指标，不是地图。

  3. 幽灵障碍
     离任何真值几何（物体足印 + 墙体矩形）都超过阈值的障碍栅格。
     墙体必须计入真值，否则每一面正确建出的墙都会被误判成幽灵
     （实测踩过：漏掉墙时 41.6% 的障碍被算作幽灵，大半其实是墙）。

也提供 --ascii 混淆矩阵叠加图便于人工核对。注意叠加图按块降采样，
块边界上的墙体可能显示成 X/o，那是渲染伪影，不是地图错误 ——
以数值指标为准。

用法：
    python3 evaluate_map.py --map ~/projects/semnav/maps/apartment.yaml
    python3 evaluate_map.py --map ... --ascii
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_path

PERIMETER_STEP = 0.05     # 物体周长采样步长
DETECT_BAND = 0.15        # 周长采样点周围多远内有障碍算"已检出"
GHOST_THRESHOLD = 0.30    # 障碍离真值几何超过此距离算幽灵
ALIGN_SEARCH = 0.50       # 配准搜索范围 ±m
# map_saver trinary PGM: 0=障碍 205=未知 254=自由
OCC_MAX_VALUE = 90
FREE_MIN_VALUE = 200


def read_pgm(path: Path):
    """读 P5 二进制 PGM（map_server 的默认输出格式）。"""
    raw = path.read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while pos < len(raw) and raw[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        fields.append(raw[start:pos])
    pos += 1
    magic, w, h, maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    if magic != b'P5':
        raise ValueError(f'只支持 P5 PGM，实际 {magic!r}')
    dtype = np.uint16 if maxval > 255 else np.uint8
    nbytes = 2 if maxval > 255 else 1
    arr = np.frombuffer(raw[pos:pos + w * h * nbytes], dtype=dtype)
    return arr.reshape((h, w))


def load_map(map_yaml: Path):
    """读 YAML + PGM，转成「[x][y] 索引、原点在左下」的掩码。

    ⚠️ Y 轴方向容易搞错：PGM 第 0 行是**图像顶部**，而 ROS 地图的 origin 指向
    **左下角**栅格。不翻转的话整张图上下颠倒，所有物体都对不上位置
    （实测症状：召回率 0%，看着像"全建错了"，其实只是读反了）。
    """
    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    data = read_pgm(map_yaml.parent / meta['image'])
    if meta.get('negate'):
        data = 255 - data
    res = float(meta['resolution'])
    origin = meta['origin']
    return {
        'occupied': (data < OCC_MAX_VALUE)[::-1].T,
        'free': (data > FREE_MIN_VALUE)[::-1].T,
        'resolution': res,
        'origin': (float(origin[0]), float(origin[1])),
        'width': int(data.shape[1]), 'height': int(data.shape[0]),
        'path': str(map_yaml),
    }


def world_to_grid(m, x, y):
    gx = int((x - m['origin'][0]) / m['resolution'])
    gy = int((y - m['origin'][1]) / m['resolution'])
    if 0 <= gx < m['width'] and 0 <= gy < m['height']:
        return gx, gy
    return None


def dilate(mask, k):
    """形态学膨胀 k 格（用滚动实现，避免依赖 scipy）。"""
    out = mask.copy()
    for _ in range(k):
        acc = out.copy()
        for ax in (0, 1):
            for shift in (-1, 1):
                acc |= np.roll(out, shift, axis=ax)
        out = acc
    return out


def rect_points(cx, cy, sx, sy, yaw_rad, step):
    """在（可旋转的）矩形内均匀采样。"""
    cyaw, syaw = math.cos(yaw_rad), math.sin(yaw_rad)
    nx, ny = max(1, int(sx / step)), max(1, int(sy / step))
    return [(cx + (-sx / 2 + (i + 0.5) * sx / nx) * cyaw
             - (-sy / 2 + (j + 0.5) * sy / ny) * syaw,
             cy + (-sx / 2 + (i + 0.5) * sx / nx) * syaw
             + (-sy / 2 + (j + 0.5) * sy / ny) * cyaw)
            for i in range(nx) for j in range(ny)]


def rect_perimeter(cx, cy, sx, sy, yaw_rad, step):
    """沿矩形周长采样（只采边界，不采内部）。"""
    cyaw, syaw = math.cos(yaw_rad), math.sin(yaw_rad)
    half_x, half_y = sx / 2, sy / 2
    pts = []
    # 两组对边：y = ±half_y 两条（沿 x 走），x = ±half_x 两条（沿 y 走）
    for edge_len, fixed_half, along_x in ((sx, half_y, True),
                                          (sy, half_x, False)):
        n = max(2, int(edge_len / step))
        for i in range(n + 1):
            t = -edge_len / 2 + edge_len * i / n
            for sgn in (+1, -1):
                lx = t if along_x else sgn * fixed_half
                ly = sgn * fixed_half if along_x else t
                pts.append((cx + lx * cyaw - ly * syaw,
                            cy + lx * syaw + ly * cyaw))
    return pts


def object_rect(obj):
    return (obj['x'], obj['y'], float(obj['size'][0]), float(obj['size'][1]),
            math.radians(float(obj.get('yaw_deg', 0.0))))


def wall_rect(w):
    x0, x1 = float(w['x_min']), float(w['x_max'])
    y0, y1 = float(w['y_min']), float(w['y_max'])
    return ((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0)


def truth_mask(m, truth):
    """把真值几何（墙体矩形 + 物体足印）光栅化到地图栅格。"""
    mask = np.zeros((m['width'], m['height']), dtype=bool)
    for w in truth.get('walls', []):
        for x, y in rect_points(*wall_rect(w), m['resolution']):
            gi = world_to_grid(m, x, y)
            if gi:
                mask[gi] = True
    for o in truth.get('objects', []):
        for x, y in rect_points(*object_rect(o), m['resolution']):
            gi = world_to_grid(m, x, y)
            if gi:
                mask[gi] = True
    return mask


def alignment(m, occupied, tmask):
    """在 ±ALIGN_SEARCH 内搜索最佳平移，返回 (dx, dy, score, score_at_zero)。

    score = 障碍栅格中「落在真值几何附近」的比例。真值不动、地图平移：
    若地图整体偏了，最佳位移就会明显偏离 (0,0)。
    """
    res = m['resolution']
    k = max(1, int(DETECT_BAND / res))
    span = int(ALIGN_SEARCH / res)
    base_band = dilate(tmask, k)
    total = max(1, int(occupied.sum()))
    best = (0, 0, -1.0)
    score_zero = None
    for dy in range(-span, span + 1, 2):
        for dx in range(-span, span + 1, 2):
            shifted = np.roll(np.roll(base_band, dx, axis=0), dy, axis=1)
            s = float((occupied & shifted).sum()) / total
            if dx == 0 and dy == 0:
                score_zero = s
            if s > best[2]:
                best = (dx, dy, s)
    return (best[0] * res, best[1] * res, round(best[2], 4),
            None if score_zero is None else round(score_zero, 4))


def evaluate(map_yaml: Path, truth_path: Path, want_ascii=False,
             ascii_width=100):
    m = load_map(map_yaml)
    truth = json.loads(truth_path.read_text(encoding='utf-8'))
    objects, walls = truth.get('objects', []), truth.get('walls', [])
    occupied = m['occupied']
    res = m['resolution']

    tmask = truth_mask(m, truth)
    occ_band = dilate(occupied, max(1, int(DETECT_BAND / res)))

    # --- 指标二：物体检出（沿周长找障碍）---
    per_object = []
    for obj in objects:
        per = rect_perimeter(*object_rect(obj), PERIMETER_STEP)
        seen = tot = 0
        for x, y in per:
            gi = world_to_grid(m, x, y)
            if gi is None:
                continue
            tot += 1
            if occ_band[gi]:
                seen += 1
        per_object.append({
            'name': obj.get('instance_id') or obj.get('label'),
            'class': obj.get('class'), 'label': obj.get('label'),
            'room': obj.get('room'),
            'perimeter_samples': tot,
            'detected_ratio': (None if tot == 0 else round(seen / tot, 3)),
        })
    scored = [p for p in per_object if p['detected_ratio'] is not None]
    detected = sum(1 for p in scored if p['detected_ratio'] >= 0.5)

    # --- 指标三：幽灵障碍 ---
    ghost_band = dilate(tmask, max(1, int(GHOST_THRESHOLD / res)))
    ghosts = int((occupied & ~ghost_band).sum())
    total_occ = int(occupied.sum())

    # --- 指标一：配准 ---
    dx, dy, best_score, score_zero = alignment(m, occupied, tmask)

    result = {
        'map': m['path'], 'truth': str(truth_path),
        'grid': {'width': m['width'], 'height': m['height'], 'resolution': res},
        'extent_m': [round(m['width'] * res, 3), round(m['height'] * res, 3)],
        'origin': list(m['origin']),
        'occupied_cells': total_occ,
        'occupied_area_m2': round(total_occ * res * res, 3),
        'free_cells': int(m['free'].sum()),
        'free_area_m2': round(int(m['free'].sum()) * res * res, 3),
        'alignment': {'best_dx_m': dx, 'best_dy_m': dy,
                      'best_score': best_score, 'score_at_zero': score_zero},
        'objects_total': len(objects),
        'objects_scored': len(scored),
        'objects_detected': detected,
        'object_recall': round(detected / max(1, len(objects)), 4),
        'object_mean_detected': (round(float(np.mean(
            [p['detected_ratio'] for p in scored])), 4) if scored else None),
        'walls_total': len(walls),
        'ghost_occupied_cells': ghosts,
        'ghost_ratio': round(ghosts / max(1, total_occ), 4),
        'per_object': per_object,
    }
    if want_ascii:
        result['ascii_overlay'] = ascii_overlay(m, tmask, ascii_width)
    return result


def ascii_overlay(m, tmask, width):
    step = max(1, int(round(m['width'] / width)))
    occ = m['occupied']
    lines = []
    for yb in range(m['height'] - 1, -1, -step):
        row = []
        for xb in range(0, m['width'], step):
            sl = (slice(xb, xb + step), slice(max(0, yb - step + 1), yb + 1))
            bo, bt, bf = occ[sl], tmask[sl], m['free'][sl]
            if bo.any() and bt.any():
                row.append('#')     # 建对
            elif bo.any():
                row.append('X')     # 幽灵
            elif bt.any():
                row.append('o')     # 漏建
            elif bf.any():
                row.append('·')     # 已探明自由
            else:
                row.append(' ')     # 未探索
        lines.append(''.join(row))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=str(Path.home()
                                        / 'projects/semnav/maps/apartment.yaml'))
    ap.add_argument('--truth', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--ascii', action='store_true')
    ap.add_argument('--ascii-width', type=int, default=100)
    args = ap.parse_args()

    truth = Path(args.truth) if args.truth else (
        get_package_share_path('semnav_bringup') / 'config'
        / 'apartment_truth.json')
    r = evaluate(Path(args.map), Path(truth), args.ascii, args.ascii_width)

    a = r['alignment']
    print(f"地图 : {r['map']}")
    print(f"范围 : {r['extent_m'][0]:.2f} x {r['extent_m'][1]:.2f} m"
          f"   原点 {r['origin']}   栅格 {r['grid']['resolution']}m")
    print(f"障碍 : {r['occupied_cells']} 格 ({r['occupied_area_m2']} m²)"
          f"   自由 : {r['free_cells']} 格 ({r['free_area_m2']} m²)")
    print()
    print('--- 指标一：配准误差 ---')
    print(f"最佳平移 : ({a['best_dx_m']:+.2f}, {a['best_dy_m']:+.2f}) m"
          f"   得分 {a['best_score']:.1%}")
    print(f"零位移得分 : {a['score_at_zero']:.1%}")
    if abs(a['best_dx_m']) <= 0.05 and abs(a['best_dy_m']) <= 0.05:
        print("判定 : ✓ 地图与真值坐标系一致，无系统性偏移")
    else:
        print(f"判定 : ✗ 地图整体偏移约 "
              f"{math.hypot(a['best_dx_m'], a['best_dy_m']):.2f} m")
    print()
    print('--- 指标二：物体检出（沿足印周长找障碍）---')
    md = r['object_mean_detected']
    print(f"检出 : {r['objects_detected']}/{r['objects_total']}"
          f" = {r['object_recall']:.1%}"
          f"   平均周长检出率 {'n/a' if md is None else f'{md:.1%}'}")
    print()
    print('--- 指标三：幽灵障碍 ---')
    print(f"幽灵 : {r['ghost_occupied_cells']} 格"
          f" ({r['ghost_ratio']:.2%} 的障碍离真值几何 > {GHOST_THRESHOLD}m)")
    print()
    print(f"{'物体':<22}{'类别':<16}{'房间':<16}{'检出率':>7}")
    for p in r['per_object']:
        v = p['detected_ratio']
        vs = ' 未探索' if v is None else f'{v:>6.0%}'
        print(f"{str(p['name']):<22}{str(p['class']):<16}"
              f"{str(p['room']):<16}{vs:>7}")

    if args.ascii:
        print()
        print('=== 叠加图（# 建对  X 幽灵  o 漏建  · 已知自由  空格 未探索）===')
        print('（按块降采样；块边界上的墙可能显示成 X/o，属渲染伪影）')
        print(r['ascii_overlay'])

    if args.out:
        Path(args.out).write_text(
            json.dumps(r, indent=2, ensure_ascii=False), encoding='utf-8')
        print(f'\n产物: {args.out}')


if __name__ == '__main__':
    main()
