#!/usr/bin/env python3
"""生成 semnav 语义家具模型集（SDF）。

为什么自建而不是复用上游模型：
  1. 上游 Sofa / CoffeeTable / TVStand 等**没有或只有残缺的碰撞体**，
     机器人会直接穿过去 —— 对建图与导航验证是致命的；
  2. 上游 Cabinet 实际是高柜（0.45×0.45×1.02），不能当床头柜；
  3. 语义导航需要「每个物体一个明确类别」，盒子模型让类别与几何一一对应，
     Phase 2 的检测/融合评测才有干净的对照。

约定：所有模型正面朝本地 -Y（沿用 URDF 里家具朝向的既有约定）。
      模型原点在底面中心，z=0 贴地。
"""

import argparse
import json
from pathlib import Path

# name -> (size_x, size_y, size_z, 颜色 RGB, 说明)
# size_y 是"进深"方向（正面朝 -Y），size_x 是"宽度"（左右）
# 一律用单一包围盒：一个模型 = 一个语义实例，Phase 2 的检测/融合评测才有
# 干净的一一对应关系（拆成座+靠背会让"一个物体"变成"两个模型"）。
MODELS = {
    "sem_bed_double":   (1.50, 2.00, 0.55,  (0.72, 0.70, 0.78), "双人床 1.5×2.0"),
    "sem_bed_single":   (1.05, 2.00, 0.50,  (0.70, 0.73, 0.80), "单人床 1.05×2.0"),
    "sem_nightstand":   (0.45, 0.40, 0.55,  (0.52, 0.38, 0.26), "床头柜 0.45×0.40×0.55"),
    "sem_wardrobe":     (1.20, 0.60, 2.00,  (0.58, 0.43, 0.30), "衣柜 1.2×0.6×2.0"),
    "sem_sofa":         (2.00, 0.90, 0.80,  (0.30, 0.36, 0.55), "沙发 2.0×0.9×0.8"),
    "sem_coffee_table": (1.00, 0.50, 0.42,  (0.46, 0.30, 0.18), "茶几 1.0×0.5×0.42"),
    "sem_tv_stand":     (1.60, 0.40, 0.45,  (0.35, 0.25, 0.18), "电视柜 1.6×0.4×0.45"),
    "sem_tv":           (1.45, 0.08, 0.85,  (0.08, 0.08, 0.09), "电视 1.45×0.08×0.85"),
    "sem_desk":         (1.00, 0.60, 0.75,  (0.50, 0.36, 0.22), "书桌 1.0×0.6×0.75"),
    "sem_bookshelf":    (0.80, 0.35, 1.80,  (0.42, 0.29, 0.18), "书架 0.8×0.35×1.8"),
    "sem_dining_table": (1.20, 0.80, 0.75,  (0.52, 0.37, 0.22), "餐桌 1.2×0.8×0.75"),
    "sem_chair":        (0.45, 0.45, 0.90,  (0.40, 0.28, 0.18), "餐椅 0.45×0.45×0.9"),
    "sem_fridge":       (0.90, 0.70, 1.78,  (0.80, 0.81, 0.83), "冰箱 0.9×0.7×1.78"),
    "sem_kitchen_counter": (1.30, 0.60, 0.90, (0.85, 0.84, 0.80), "厨房台面 1.3×0.6×0.9"),
    "sem_oven":         (0.60, 0.60, 0.90,  (0.30, 0.30, 0.32), "烤箱 0.6×0.6×0.9"),
    "sem_coffee_maker": (0.21, 0.28, 0.36,  (0.20, 0.20, 0.22), "咖啡机 0.21×0.28×0.36"),
    "sem_shower":       (0.90, 0.90, 2.00,  (0.80, 0.86, 0.88), "淋浴间 0.9×0.9×2.0"),
    "sem_washer":       (0.60, 0.60, 0.85,  (0.88, 0.88, 0.90), "洗衣机 0.6×0.6×0.85"),
}

MODEL_CONFIG = """<?xml version="1.0" ?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>semnav</name>
  </author>
  <description>{desc}（semnav 自建；碰撞体与视觉一致，正面朝 -Y，原点在底面中心）</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 {hz:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 {hz:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
          <diffuse>{dr:.3f} {dg:.3f} {db:.3f} 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True, help="模型集输出目录（semnav_models/models）")
    args = ap.parse_args()

    root = Path(args.out_dir)
    manifest = []
    for name, (sx, sy, sz, rgb, desc) in MODELS.items():
        d = root / name
        d.mkdir(parents=True, exist_ok=True)
        dr, dg, db = (min(1.0, c * 1.15) for c in rgb)
        (d / "model.config").write_text(
            MODEL_CONFIG.format(name=name, desc=f"{name} {sx}×{sy}×{sz}"), encoding="utf-8")
        (d / "model.sdf").write_text(
            MODEL_SDF.format(name=name, sx=sx, sy=sy, sz=sz, hz=sz / 2,
                             r=rgb[0], g=rgb[1], b=rgb[2],
                             dr=dr, dg=dg, db=db),
            encoding="utf-8")
        manifest.append({"model": name, "size": [sx, sy, sz]})
        print(f"  ✓ {name:<22} {sx:.2f} × {sy:.2f} × {sz:.2f}")

    (root / "models_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"\n共生成 {len(manifest)} 个模型 -> {root}")


if __name__ == "__main__":
    main()
