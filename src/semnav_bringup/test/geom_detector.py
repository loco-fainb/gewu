#!/usr/bin/env python3
"""几何优先的物体检测：从深度图找回地竖直平面，按尺寸筛选。

动机（实测得出）：
  YOLOv8n COCO 在 Gazebo 无纹理场景上不可用 —— 4 个位姿只检出 2 个，
  且 D 位姿 0.436 置信度的框 (1,1,265,261) 实际框在左侧白墙上。
  根本原因：白墙和白色冰箱在纹理上无法区分。

  但几何上完全可以区分：
    冰箱 = 离地的竖直矩形面，宽 0.73 m、高 1.74 m、底边接近地面、贴墙
    白墙 = 同一个面，但宽高远超物体尺度

  深度数据实测质量：冰箱正面主层占 74.7%，标准差 2.4 mm。足够做几何拟合。

  而且深度相机在 0.5125 m 高、vFoV 110 度，1.03 m 处可见最高 1.98 m，
  能完整看到 1.74 m 高的冰箱；RGB 相机只能看到 1.39 m，顶部必然出框。

算法：
  1) 深度 -> base_link 系点云
  2) 逐像素法向量（邻居差分叉积）
  3) 筛出「竖直表面」：法向量水平（|n_z| 小）
  4) 连通域聚类
  5) 每簇拟合平面，量出宽、高、离地高度、距离
  6) 按物体先验打分（尺寸区间 + 离地 + 距离在有效范围）
"""

from __future__ import annotations

import math

import cv2
import numpy as np

# 相机参数（与 semloc.py 一致）
W, H = 640, 914
FX = FY = 320.0
CX, CY = 320.0, 457.0
CAM_X = 0.349 / 2      # 0.1745
CAM_Z = 0.0125 + 0.50  # 0.5125

MIN_D, MAX_D = 0.15, 4.5


def depth_to_points(depth: np.ndarray) -> np.ndarray:
    """深度图 -> base_link 系点云 (H, W, 3)。

    光学系：z 前、x 右、y 下；光轴为 base 的 +x。
      base.x = CAM_X + d
      base.y = 0 - (u - CX)/FX * d
      base.z = CAM_Z - (v - CY)/FY * d
    """
    v, u = np.mgrid[0:H, 0:W].astype(np.float32)
    d = depth.astype(np.float32)
    bx = CAM_X + d
    by = -(u - CX) / FX * d
    bz = CAM_Z - (v - CY) / FY * d
    return np.stack([bx, by, bz], axis=-1)


def normals_from_points(pts: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """逐像素法向量（沿 u、v 方向差分后叉积）。"""
    n = np.zeros_like(pts)
    # 沿 u 的切向
    du = np.zeros_like(pts)
    du[:, 1:-1] = pts[:, 2:] - pts[:, :-2]
    # 沿 v 的切向
    dv = np.zeros_like(pts)
    dv[1:-1, :] = pts[2:, :] - pts[:-2, :]
    nn = np.cross(du, dv)
    norm = np.linalg.norm(nn, axis=-1, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        n = np.where(norm > 1e-9, nn / norm, 0.0)
    # 深度不连续处的法向无意义
    bad = (norm[..., 0] < 1e-9)
    n[bad] = 0.0
    return n


def find_vertical_surfaces(depth: np.ndarray,
                           min_area_px: int = 3000,
                           normal_z_tol: float = 0.45,
                           height_range: tuple[float, float] = (-0.6, 3.0),
                           depth_jump: float = 0.06) -> list[dict]:
    """找出所有「离地竖直面」候选，返回几何描述列表。"""
    valid = np.isfinite(depth) & (depth > MIN_D) & (depth < MAX_D)
    pts = depth_to_points(np.where(valid, depth, 0.0))
    nrm = normals_from_points(pts, depth)

    # 竖直面：法向量的 z 分量接近 0
    vertical = valid & (np.abs(nrm[..., 2]) < normal_z_tol)
    # 排除天花板/地板：法向几乎垂直的那些已经被上面排除
    z = pts[..., 2]
    vertical &= (z > height_range[0]) & (z < height_range[1])

    # 深度不连续处断开（避免把多个物体连成一片）
    d = depth.astype(np.float32)
    jump = np.zeros_like(d, dtype=bool)
    dx = np.abs(np.diff(d, axis=1, prepend=d[:, :1]))
    dy = np.abs(np.diff(d, axis=0, prepend=d[:1, :]))
    jump = (dx > depth_jump) | (dy > depth_jump) | ~valid
    vertical &= ~jump

    mask = vertical.astype(np.uint8)
    n_lab, lab = cv2.connectedComponents(mask, connectivity=8)

    out = []
    for i in range(1, n_lab):
        m = (lab == i)
        cnt = int(m.sum())
        if cnt < min_area_px:
            continue

        p = pts[m]                      # (N,3)
        dd = depth[m]
        # 平面拟合（最小二乘）
        c = p.mean(axis=0)
        u, s, vt = np.linalg.svd(p - c, full_matrices=False)
        n = vt[-1]
        if n[2] < 0:
            n = -n
        # 要求是竖直面：法向量的 z 分量小
        if abs(n[2]) > normal_z_tol:
            continue

        # 平面内基：水平方向 + 竖直方向
        e_v = np.array([0.0, 0.0, 1.0])
        e_u = np.cross(e_v, n)
        nu = np.linalg.norm(e_u)
        if nu < 1e-6:
            continue
        e_u /= nu
        proj_u = (p - c) @ e_u
        proj_v = (p - c) @ e_v

        width = float(proj_u.max() - proj_u.min())
        height = float(proj_v.max() - proj_v.min())
        z_min = float(p[:, 2].min())
        z_max = float(p[:, 2].max())
        dist_mean = float(np.median(dd))

        # 该面中心在地面上的落点（base 系）：取面中心，z 压到 0
        center_u = float((proj_u.max() + proj_u.min()) / 2)
        ground = c + e_u * center_u
        ground = ground * np.array([1.0, 1.0, 0.0])

        out.append({
            'label_id': i,
            'n_px': cnt,
            'dist': dist_mean,
            'width': width,
            'height': height,
            'z_min': z_min,
            'z_max': z_max,
            'ground_base': (float(ground[0]), float(ground[1])),
            'normal': tuple(float(v) for v in n),
            'centroid': tuple(float(v) for v in c),
            's_gap': float(s[-1] / max(s[0], 1e-9)),   # 平面性
        })

    out.sort(key=lambda r: -r['n_px'])
    return out


# 冰箱先验（不是真值位置，是物理常识区间）
FRIDGE_PRIOR = {
    'width': (0.45, 1.30),      # 家用冰箱宽
    'height': (1.20, 2.40),     # 家用冰箱高
    'z_min': (-0.15, 0.45),     # 落地（允许地面噪声）
    'dist': (0.4, 4.0),
}


def score_fridge(c: dict) -> float:
    """按冰箱先验给候选打分。0 表示不符合。"""
    w, h, zb, dd = c['width'], c['height'], c['z_min'], c['dist']
    score = 0.0

    if not (FRIDGE_PRIOR['width'][0] <= w <= FRIDGE_PRIOR['width'][1]):
        return 0.0
    if not (FRIDGE_PRIOR['height'][0] <= h <= FRIDGE_PRIOR['height'][1]):
        return 0.0
    if not (FRIDGE_PRIOR['z_min'][0] <= zb <= FRIDGE_PRIOR['z_min'][1]):
        return 0.0
    if not (FRIDGE_PRIOR['dist'][0] <= dd <= FRIDGE_PRIOR['dist'][1]):
        return 0.0

    # 越接近典型尺寸分越高
    score += 1.0 - abs(w - 0.75) / 0.55
    score += 1.0 - abs(h - 1.75) / 0.85
    score += 1.0 - abs(zb) / 0.35
    # 平面性越好分越高
    score += max(0.0, 1.0 - c['s_gap'] * 20)
    # 像素越多越可靠（但不要压过几何符合度）
    score += min(1.0, c['n_px'] / 60000.0) * 0.5
    return max(0.0, score)

