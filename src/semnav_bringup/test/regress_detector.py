#!/usr/bin/env python3
"""v3 离线回归：用已采集的 5 个场景深度数据，验证新门槛下判定正确。

必须在 ROS 环境跑（可 import rclpy），直接复用节点里的 GeomDetector。

关键：v3 的 find() 改用「光学系点云 + TF 变换」，
离线测试需要构造等效的 R,t（相机在 base_link 前向 0.1745、高 0.5125）。
"""
import importlib.util
import sys

import numpy as np

NODE = '/home/rufus-dargon/projects/semnav/src/semnav_bringup/scripts/semantic_locator.py'
spec = importlib.util.spec_from_file_location('slv3', NODE)
sl = importlib.util.module_from_spec(spec)
sys.modules['slv3'] = sl
spec.loader.exec_module(sl)

# 构造等效变换：odom <- depth_camera_optical_frame
# 光学系 z 前、x 右、y 下；base_link x 前、y 左、z 上
# 相机在 base 的 (0.1745, 0, 0.5125)
R_base_opt = np.array([
    [0.0, 0.0, 1.0],    # base.x = opt.z
    [-1.0, 0.0, 0.0],   # base.y = -opt.x
    [0.0, -1.0, 0.0],   # base.z = -opt.y
], dtype=np.float64)
t_base = np.array([0.1745, 0.0, 0.5125], dtype=np.float64)

cat = sl.load_catalog(
    '/home/rufus-dargon/projects/semnav/src/semnav_bringup/config/apartment.yaml')

CFG_V2 = dict(min_depth=0.15, max_depth=4.5, min_area_px=2500,
              normal_z_tol=0.45, depth_jump=0.06, max_planarity=999.0)
CFG_V3 = dict(min_depth=0.15, max_depth=4.5, min_area_px=2500,
              normal_z_tol=0.45, depth_jump=0.06, max_planarity=0.05)


def run(cfg, gz_min, gz_max, target='refrigerator',
        min_score=0.15, min_margin=0.02):
    """返回 {tag: (accept, detail)}"""
    det = sl.GeomDetector(cat, cfg)
    det.ground_z_min = gz_min
    det.ground_z_max = gz_max

    cases = [
        ('W1_wardrobe_bed2', '次卧二衣柜', False),
        ('W2_wardrobe_master', '主卧衣柜', False),
        ('C1_cabinet_kitchen', '厨房橱柜', False),
        ('S1_stove_kitchen', '厨房灶台', False),
        ('REF_control', '厨房冰箱', True),
    ]
    out = []
    for tag, label, is_fridge in cases:
        try:
            d = np.load(f'/tmp/fp_{tag}_depth.npy')
        except FileNotFoundError:
            out.append((label, is_fridge, None, 'mising'))
            continue
        cands = det.find(d, 320.0, 320.0, 320.0, 457.0, R_base_opt, t_base)
        best = None
        rej = []
        for c in cands:
            why = det.reject_reason(c)
            if why:
                rej.append(f'#{c.n_px}px {why}')
                continue
            ranked = det.classify(c)
            if not ranked:
                continue
            sc, cls, entry = ranked[0]
            margin = (ranked[1][0] - sc) if len(ranked) > 1 else 1.0
            if cls != target:
                continue
            if sc > min_score or margin < min_margin:
                continue
            if best is None or sc < best[0]:
                best = (sc, cls, margin, c)

        if best is None:
            out.append((label, is_fridge, False, f'无通过候选; 拒: {rej}'))
        else:
            sc, cls, margin, c = best
            out.append((label, is_fridge, True,
                        f'score={sc:.4f} 宽={c.width:.3f} 高={c.height:.3f} '
                        f'z_min={c.z_min:+.3f} 平面性={c.planarity:.4f}'))
    return out


print('=' * 86)
print('v2 配置（无 planarity / 无 z_min 门槛）')
print('=' * 86)
v2 = run(CFG_V2, -9.0, 9.0)
for label, is_fridge, acc, detail in v2:
    ok = (acc == is_fridge) if acc is not None else False
    print(f'  {label:12} {"冰箱" if is_fridge else "非冰箱":6} '
          f'{"接受" if acc else "拒绝":4} {"✓" if ok else "✗"}  {detail}')

print()
print('=' * 86)
print('v3 配置（planarity<=0.05, z_min∈[-0.15,0.35]）')
print('=' * 86)
v3 = run(CFG_V3, -0.15, 0.35)
allok = True
for label, is_fridge, acc, detail in v3:
    ok = (acc == is_fridge) if acc is not None else False
    if not ok:
        allok = False
    print(f'  {label:12} {"冰箱" if is_fridge else "非冰箱":6} '
          f'{"接受" if acc else "拒绝":4} {"✓" if ok else "✗"}  {detail}')

print()
print('=' * 86)
print('结论')
print('=' * 86)
print(f'  v2 全对: {all((a == f) for _, f, a, _ in v2 if a is not None)}')
print(f'  v3 全对: {allok}')
print()
print('  说明：v3 的 find() 用光学系点云 + TF，坐标系与 v2 不同，')
print('  因此宽/高/z_min 数值应与 v2 一致（都是同一物理量），已在上表核对。')

