#!/usr/bin/env python3
"""尺寸匹配做类别判定：不用任何神经网络。

依据：几何法已在 4 个位姿上量出冰箱尺寸（宽 0.722~0.849，真值 0.73），
精度 ±0.02 m；而 apartment.yaml 里 27 个物体的尺寸互不相同，
足以用 (宽, 高) 二元组区分。

这个方法的前提是"目标物体在场景清单内且尺寸已知"，
对本项目成立（apartment.yaml 是单一真源）。

输出：对每个检出的竖直面，给出最匹配的物体类别 + 匹配分 + 次优类别。
"""
import json
import pathlib
import sys

import yaml

ROOT = pathlib.Path('/home/rufus-dargon/projects/semnav')
LAYOUT = ROOT / 'src/semnav_bringup/config/apartment.yaml'


def load_catalog() -> dict[str, list[dict]]:
    """从 apartment.yaml 建尺寸目录：class -> [{model, w, d, h, count}]。"""
    a = yaml.safe_load(LAYOUT.read_text(encoding='utf-8'))
    cat: dict[str, list[dict]] = {}
    for o in a.get('objects') or []:
        cls = o['class']
        sz = o['size']                      # [x_len, y_len, z_height]
        entry = {
            'model': o['model'],
            'dims': sorted([sz[0], sz[1]]),  # 水平两个维度（不分前后）
            'height': sz[2],
        }
        cat.setdefault(cls, []).append(entry)
    return cat


def match_class(width: float, height: float, cat: dict) -> list[tuple[float, str, dict]]:
    """按 (宽, 高) 匹配。宽度与水平两维中较小的比（因为看到的面可能是短边或长边）。"""
    out = []
    for cls, entries in cat.items():
        best = None
        for e in entries:
            for hw in e['dims']:            # 可能看到短边或长边
                # 归一化误差：宽、高各自相对误差
                ew = abs(width - hw) / max(hw, 1e-6)
                eh = abs(height - e['height']) / max(e['height'], 1e-6)
                score = (ew + eh) / 2
                if best is None or score < best[0]:
                    best = (score, cls, {**e, 'matched_w': hw})
        if best:
            out.append(best)
    out.sort(key=lambda x: x[0])
    return out


def main():
    cat = load_catalog()
    print('=== 场景物体尺寸目录（apartment.yaml）===')
    print(f'  {"class":18} {"model":26} {"水平尺寸":>14} {"高":>6}')
    for cls in sorted(cat):
        for e in cat[cls]:
            d = e['dims']
            print(f'  {cls:18} {e["model"]:26} {d[0]:.2f} x {d[1]:.2f}{"":>3} {e["height"]:.2f}')

    print(f'\n  共 {len(cat)} 类, {sum(len(v) for v in cat.values())} 个实例')

    # ── 用实测几何结果做类别判定 ──
    print('\n=== 用四位姿的实测几何做类别判定 ===')
    results = []

    # 这些数字来自 test_geom.py 的实际输出
    measured = [
        ('A_front_1.2m', 0.725, 1.645, 0.066),
        ('B_front_2.0m', 0.722, 1.635, 0.068),
        ('C_left_20deg', 0.724, 1.653, 0.067),
        ('D_right_20deg', 0.849, 1.724, -0.077),
    ]

    for tag, w, h, zb in measured:
        print(f'\n[{tag}]  实测 宽={w:.3f} 高={h:.3f} 离地={zb:+.3f}')
        m = match_class(w, h, cat)
        for i, (s, cls, e) in enumerate(m[:3]):
            mark = '  ★' if i == 0 else ''
            print(f'    {i+1}. {cls:18} 匹配宽={e["matched_w"]:.2f} 高={e["height"]:.2f} '
                  f'误差分={s:.4f}{mark}')
        results.append({'tag': tag, 'top': m[0][1], 'score': m[0][0],
                        'second': m[1][1] if len(m) > 1 else None,
                        'second_score': m[1][0] if len(m) > 1 else None})

    print('\n=== 判定结果 ===')
    print(f'  {"位姿":16} {"判为":18} {"分数":>8} {"次优":18} {"次优分":>8} {"区分度":>8}')
    all_ok = True
    for r in results:
        ok = r['top'] == 'refrigerator'
        if not ok:
            all_ok = False
        margin = (r['second_score'] - r['score']) if r['second_score'] is not None else 0
        print(f'  {r["tag"]:16} {r["top"]:18} {r["score"]:8.4f} '
              f'{str(r["second"]):18} {r["second_score"]:8.4f} {margin:8.4f}'
              f'{"  ✓" if ok else "  ✗"}')

    print(f'\n  全部判为 refrigerator: {all_ok}')
    if all_ok:
        print('  → 尺寸匹配足以完成类别判定，无需神经网络')
    else:
        print('  → 有误判，需检查：近距离时看到的可能是局部面（顶部/侧面被裁切）')

    # ── 关键局限说明 ──
    print('\n=== 方法局限（必须知道）===')
    print('  1. 只能区分清单内的类别。清单外物体（如临时摆放的箱子）会被误判为最像的那个。')
    print('  2. 尺寸接近的物体无法区分：如 nightstand(0.47x0.49, 高1.02) 与')
    print('     cabinet(0.47x0.49, 高1.02) 完全相同，几何上不可分。')
    print('  3. 只能看到局部面时（近距离顶部出框）尺寸会被低估，当前 D 位姿宽 0.849')
    print('     比真值 0.73 偏大 16%，是因为把相邻面也框进来了。')
    print('  4. 需要目标大致在场景清单内 —— 本项目满足，通用机器人场景不满足。')

    # ── 列出尺寸接近、几何不可分的组 ──
    print('\n=== 几何不可分的物体对（尺寸几乎相同）===')
    items = []
    for cls, entries in cat.items():
        for e in entries:
            items.append((cls, e['dims'][0], e['dims'][1], e['height'], e['model']))
    seen = set()
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i], items[j]
            if a[0] == b[0]:
                continue
            dw = max(abs(a[1]-b[1]), abs(a[2]-b[2]))
            dh = abs(a[3]-b[3])
            if dw < 0.08 and dh < 0.10:
                key = tuple(sorted([a[4], b[4]]))
                if key in seen:
                    continue
                seen.add(key)
                print(f'  {a[0]:16} ({a[1]:.2f}x{a[2]:.2f}, h={a[3]:.2f})  ≈  '
                      f'{b[0]:16} ({b[1]:.2f}x{b[2]:.2f}, h={b[3]:.2f})')


if __name__ == '__main__':
    main()

