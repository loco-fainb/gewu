#!/usr/bin/env python3
"""从 apartment.yaml 生成 Gazebo 世界文件与真值清单。

用法：
    python3 generate_world.py --layout <apartment.yaml> --out-world <world.sdf> --out-truth <truth.json>

为什么用生成器而不是手写 SDF：
  1. 墙体带门洞需要精确切分，手算易错；
  2. 真值清单（物体类别 + 精确位姿）必须与世界同源，否则 Phase 2 的
     「地图位置 vs 真值误差」评测会失去意义；
  3. 加房间 / 换布局时不用重写 800 行 SDF。
"""

import argparse
import json
import math
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("需要 python3-yaml：sudo apt install -y python3-yaml")

# 模型正面默认朝局部 -Y（上游 Gazebo 模型约定）。
# front 语义 -> 绕 Z 轴旋转角度（度，逆时针为正）
FRONT_YAW_DEG = {
    "south": 0.0,    # -Y
    "east": 90.0,    # +X
    "north": 180.0,  # +Y
    "west": 270.0,   # -X
}

WALL_HEIGHT = 2.5
DOOR_HEIGHT = 2.05


def wall_x(name, x0, x1, y, thickness, height=WALL_HEIGHT):
    """沿 x 轴的水平墙段（东西走向）。"""
    length = x1 - x0
    if length <= 1e-6:
        return None
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{(x0 + x1) / 2:.4f} {y:.4f} {height / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{length:.4f} {thickness:.4f} {height:.4f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{length:.4f} {thickness:.4f} {height:.4f}</size></box></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.92 0.92 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def wall_y(name, y0, y1, x, thickness, height=WALL_HEIGHT):
    """沿 y 轴的垂直墙段（南北走向）。"""
    length = y1 - y0
    if length <= 1e-6:
        return None
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {(y0 + y1) / 2:.4f} {height / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{thickness:.4f} {length:.4f} {height:.4f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{thickness:.4f} {length:.4f} {height:.4f}</size></box></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.92 0.92 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def lintel(name, x, y, width, thickness, axis="x"):
    """门洞上方的过梁：让门洞在垂直方向有结构，但不阻挡机器人通行。"""
    h = WALL_HEIGHT - DOOR_HEIGHT
    z = DOOR_HEIGHT + h / 2
    sx, sy = (width, thickness) if axis == "x" else (thickness, width)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>
          <material><ambient>0.80 0.80 0.80 1</ambient><diffuse>0.88 0.88 0.86 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def split_wall_by_doors(seg_lo, seg_hi, doors, door_width):
    """把 [seg_lo, seg_hi] 按门洞切分成若干实心墙段，返回 [(lo,hi), ...] 与门洞列表。

    doors 直接来自 YAML，字段名是 center_x；这里统一成内部使用的 center，
    避免「配置文件字段名」泄漏进切割算法。
    """
    norm = [{"center": d["center_x"], "id": d["id"]} for d in doors]
    solids, holes = [], []
    cursor = seg_lo
    for d in sorted(norm, key=lambda v: v["center"]):
        lo, hi = d["center"] - door_width / 2, d["center"] + door_width / 2
        if lo > cursor:
            solids.append((cursor, lo))
        holes.append((lo, hi))
        cursor = max(cursor, hi)
    if cursor < seg_hi:
        solids.append((cursor, seg_hi))
    return solids, holes


def object_pose(x, y, front):
    """返回 (x, y, z, yaw_rad)。z=0 表示模型底面贴地。"""
    return x, y, 0.0, math.radians(FRONT_YAW_DEG[front])


def rotated_footprint(o):
    """返回物体在世界上占据的轴对齐矩形 (x0, x1, y0, y1)。

    yaw 只取 0/90/180/270（由 front 决定），所以旋转后必是轴对齐：
    90/270 时交换 x/y 尺寸。
    """
    sx, sy = o.get("size", [0.0, 0.0, 0.0])[0], o.get("size", [0.0, 0.0, 0.0])[1]
    yaw = FRONT_YAW_DEG[o["front"]]
    if yaw in (90.0, 270.0):
        sx, sy = sy, sx
    return (o["x"] - sx / 2, o["x"] + sx / 2,
            o["y"] - sy / 2, o["y"] + sy / 2)


def wall_rects(rooms, doors, ext, inte, door_width=0.9, wall_segments=None):
    """构造所有墙体在平面上的矩形 (name, x0, x1, y0, y1)。

    这些矩形同时用于两处：生成 SDF，以及校验家具是否插进墙里。
    两处同源，才不会有「校验通过但仿真里穿墙」这种自欺。
    过梁（门洞上方 2.05m 以上）不列入：没有任何家具高过 2.0m。
    """
    if wall_segments:
        x_min = min(min(float(s["x1"]), float(s["x2"])) for s in wall_segments)
        x_max = max(max(float(s["x1"]), float(s["x2"])) for s in wall_segments)
        y_min = min(min(float(s["y1"]), float(s["y2"])) for s in wall_segments)
        y_max = max(max(float(s["y1"]), float(s["y2"])) for s in wall_segments)
    else:
        x_min = min(r["x_min"] for r in rooms)
        x_max = max(r["x_max"] for r in rooms)
        y_min = min(r["y_min"] for r in rooms)
        y_max = max(r["y_max"] for r in rooms)

    rects = []

    def add_h(name, x0, x1, yc, t):
        if x1 - x0 > 1e-6:
            rects.append((name, x0, x1, yc - t / 2, yc + t / 2))

    def add_v(name, y0, y1, xc, t):
        if y1 - y0 > 1e-6:
            rects.append((name, xc - t / 2, xc + t / 2, y0, y1))

    if wall_segments:
        for seg in wall_segments:
            name = seg["id"]
            thickness = float(seg.get("thickness", inte))
            x1, y1 = float(seg["x1"]), float(seg["y1"])
            x2, y2 = float(seg["x2"]), float(seg["y2"])
            openings = sorted(seg.get("openings", []),
                              key=lambda o: float(o["center"]))
            if abs(y2 - y1) < 1e-6:
                lo, hi, cursor = min(x1, x2), max(x1, x2), min(x1, x2)
                for opening in openings:
                    center = float(opening["center"])
                    width = float(opening.get("width", door_width))
                    add_h(f"{name}_{len(rects)}", cursor, center - width / 2, y1, thickness)
                    cursor = center + width / 2
                add_h(f"{name}_{len(rects)}", cursor, hi, y1, thickness)
            elif abs(x2 - x1) < 1e-6:
                lo, hi, cursor = min(y1, y2), max(y1, y2), min(y1, y2)
                for opening in openings:
                    center = float(opening["center"])
                    width = float(opening.get("width", door_width))
                    add_v(f"{name}_{len(rects)}", cursor, center - width / 2, x1, thickness)
                    cursor = center + width / 2
                add_v(f"{name}_{len(rects)}", cursor, hi, x1, thickness)
        return rects, (x_min, x_max, y_min, y_max)

    # 外墙
    add_h("wall_north", x_min, x_max, y_max, ext)
    add_h("wall_south", x_min, x_max, y_min, ext)
    add_v("wall_west", y_min, y_max, x_min, ext)
    add_v("wall_east", y_min, y_max, x_max, ext)

    # 走廊两侧内墙（按门洞切分）
    for wall_y_coord in (0.6, -0.6):
        ds = [d for d in doors if abs(d["wall_y"] - wall_y_coord) < 1e-6]
        solids, _ = split_wall_by_doors(x_min, x_max, ds, door_width)
        tag = "north" if wall_y_coord > 0 else "south"
        for i, (lo, hi) in enumerate(solids):
            add_h(f"wall_corridor_{tag}_{i}", lo, hi, wall_y_coord, inte)

    # 房间隔墙：从矩形房间边界推导，避免布局变成三室时还要同步修改
    # 一组手写坐标。只取跨过走廊边界的相邻房间分界线。
    north_rooms = [r for r in rooms if r["y_min"] <= 0.6 + 1e-6
                   and r["y_max"] > 0.6 + 1e-6]
    north_xs = sorted({r["x_min"] for r in north_rooms}
                      | {r["x_max"] for r in north_rooms})
    for xc in north_xs[1:-1]:
        adjacent = [r for r in north_rooms
                    if abs(r["x_min"] - xc) < 1e-6
                    or abs(r["x_max"] - xc) < 1e-6]
        top = max(r["y_max"] for r in adjacent)
        add_v(f"wall_north_div_{xc:+.1f}", 0.6, top, xc, inte)

    south_rooms = [r for r in rooms if r["y_max"] >= -0.6 - 1e-6
                   and r["y_min"] < -0.6 - 1e-6]
    south_xs = sorted({r["x_min"] for r in south_rooms}
                      | {r["x_max"] for r in south_rooms})
    for xc in south_xs[1:-1]:
        adjacent = [r for r in south_rooms
                    if abs(r["x_min"] - xc) < 1e-6
                    or abs(r["x_max"] - xc) < 1e-6]
        bottom = min(r["y_min"] for r in adjacent)
        add_v(f"wall_south_div_{xc:+.1f}", bottom, -0.6, xc, inte)

    return rects, (x_min, x_max, y_min, y_max)


def validate_layout(rooms, doors, objects, walls):
    """生成前静态校验：插墙、越界、物体互相重叠、堵门。

    这些错误如果留到仿真里排查，代价是「启动 40 秒 + 肉眼看 Gazebo」；
    在生成阶段拦下来几乎零成本。返回问题清单（空 = 全部通过）。
    """
    problems = []

    def find_room(x, y):
        for r in rooms:
            if r["x_min"] <= x <= r["x_max"] and r["y_min"] <= y <= r["y_max"]:
                return r
        return None

    # 1) 物体中心必须落在某个房间内
    for o in objects:
        if find_room(o["x"], o["y"]) is None:
            problems.append(f"物体 {o.get('label', o['class'])} 中心不在任何房间内 "
                            f"({o['x']}, {o['y']})")

    # 2) 物体不得与任何墙体相交（含 1cm 装配余量）
    for o in objects:
        x0, x1, y0, y1 = rotated_footprint(o)
        for name, wx0, wx1, wy0, wy1 in walls:
            ox = min(x1, wx1) - max(x0, wx0)
            oy = min(y1, wy1) - max(y0, wy0)
            if ox > 0.005 and oy > 0.005:
                problems.append(
                    f"物体 {o.get('label', o['class'])} 插进墙体 {name}："
                    f"重叠 {ox:.2f}×{oy:.2f} m "
                    f"（家具 x[{x0:+.2f},{x1:+.2f}] y[{y0:+.2f},{y1:+.2f}]，"
                    f"墙 x[{wx0:+.2f},{wx1:+.2f}] y[{wy0:+.2f},{wy1:+.2f}]）")

    # 3) 物体之间不得重叠。必须先判 z 轴：写在台面上的物体（咖啡机）、
    #    摆在柜上的物体（电视）在 2D 投影上必然与承托物重叠，那是设计意图。
    objs = list(objects)
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = objs[i], objs[j]
            size_a = a.get("size", [0, 0, 0])
            size_b = b.get("size", [0, 0, 0])
            az0 = float(a.get("z", 0.0))
            az1 = az0 + float(size_a[2])
            bz0 = float(b.get("z", 0.0))
            bz1 = bz0 + float(size_b[2])
            if min(az1, bz1) - max(az0, bz0) <= 0.01:
                continue
            ax0, ax1, ay0, ay1 = rotated_footprint(a)
            bx0, bx1, by0, by1 = rotated_footprint(b)
            ox = min(ax1, bx1) - max(ax0, bx0)
            oy = min(ay1, by1) - max(ay0, by0)
            if ox > 0.005 and oy > 0.005:
                problems.append(
                    f"物体重叠：{a.get('label', a['class'])} 与 {b.get('label', b['class'])} "
                    f"重叠 {ox:.2f}×{oy:.2f} m")

    # 4) 门洞净空：门洞正前方 0.6m 的通行带内不得有家具
    for d in doors:
        if "wall_y" not in d:
            continue
        dcx, dw = d["center_x"], float(d.get("width", 0.9))
        wy = d["wall_y"]
        zone = (wy, wy + 0.60) if wy > 0 else (wy - 0.60, wy)
        for o in objects:
            x0, x1, y0, y1 = rotated_footprint(o)
            if x0 <= dcx + dw / 2 and x1 >= dcx - dw / 2:
                if min(y1, zone[1]) - max(y0, zone[0]) > 0.005:
                    problems.append(
                        f"门 {d['id']} 通行带被 {o.get('label', o['class'])} 侵占"
                        f"（家具 y[{y0:+.2f},{y1:+.2f}]，通行带 y"
                        f"[{zone[0]:+.2f},{zone[1]:+.2f}]）")

    return problems


def gen_objects(objects):
    out = []
    truth = []
    for i, o in enumerate(objects):
        name = f"sem_{o['class']}_{i:02d}"
        yaw = math.radians(FRONT_YAW_DEG[o["front"]])
        z = float(o.get("z", 0.0))
        scale = o.get("scale", [1.0, 1.0, 1.0])
        scale_tag = ""
        if scale != [1.0, 1.0, 1.0]:
            scale_tag = f"""
      <scale>{scale[0]} {scale[1]} {scale[2]}</scale>"""
        out.append(f"""    <include>
      <name>{name}</name>
      <uri>model://{o['model']}</uri>
      <pose>{o['x']:.4f} {o['y']:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>{scale_tag}
    </include>
""")
        truth.append({
            "instance_id": name,
            "class": o["class"],
            "label": o.get("label", o["class"]),
            "model": o["model"],
            "x": round(o["x"], 4),
            "y": round(o["y"], 4),
            "z": round(z, 4),
            "yaw_deg": round(FRONT_YAW_DEG[o["front"]], 1),
            "size": o.get("size", [0.0, 0.0, 0.0]),
        })
    return "".join(out), truth


def room_of(truth_obj, rooms):
    for r in rooms:
        if (r["x_min"] <= truth_obj["x"] <= r["x_max"]
                and r["y_min"] <= truth_obj["y"] <= r["y_max"]):
            return r["id"]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out-world", required=True)
    ap.add_argument("--out-truth", required=True)
    ap.add_argument("--force", action="store_true",
                    help="布局校验失败时仍然生成（默认失败即退出）")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.layout).read_text(encoding="utf-8"))
    # A scaled layout must scale vertical dimensions too.  The 3x map uses
    # model variants with the same baked factor, so wall and door heights are
    # kept in the same coordinate frame.
    global WALL_HEIGHT, DOOR_HEIGHT
    world_scale = float(cfg.get("world_scale", 1.0))
    WALL_HEIGHT = 2.5 * world_scale
    DOOR_HEIGHT = 2.05 * world_scale
    rooms = cfg["rooms"]
    doors = cfg["doors"]
    objs = cfg["objects"]
    ext = cfg["walls"]["exterior_thickness"]
    inte = cfg["walls"]["interior_thickness"]

    # 户型包围盒
    wall_segments = cfg.get("wall_segments")
    if wall_segments:
        x_min = min(min(float(s["x1"]), float(s["x2"])) for s in wall_segments)
        x_max = max(max(float(s["x1"]), float(s["x2"])) for s in wall_segments)
        y_min = min(min(float(s["y1"]), float(s["y2"])) for s in wall_segments)
        y_max = max(max(float(s["y1"]), float(s["y2"])) for s in wall_segments)
    else:
        x_min = min(r["x_min"] for r in rooms)
        x_max = max(r["x_max"] for r in rooms)
        y_min = min(r["y_min"] for r in rooms)
        y_max = max(r["y_max"] for r in rooms)

    # ---- 墙体：先出数据，再由数据出 SDF（校验与生成同源）----
    walls, _ = wall_rects(rooms, doors, ext, inte, wall_segments=wall_segments)

    # ---- 生成前校验 ----
    problems = validate_layout(rooms, doors, objs, walls)
    if problems:
        print("✗ 布局校验未通过：", file=sys.stderr)
        for p in problems:
            print(f"    - {p}", file=sys.stderr)
        if not args.force:
            sys.exit(1)
        print("  （--force：仍然继续生成）", file=sys.stderr)
    else:
        print("✓ 布局校验通过（墙体相交 / 越界 / 重叠 / 门洞净空）")

    parts = []

    # ---- 地板 ----
    span_x, span_y = x_max - x_min, y_max - y_min
    parts.append(f"""    <model name="apartment_floor">
      <static>true</static>
      <pose>{(x_min + x_max) / 2:.4f} {(y_min + y_max) / 2:.4f} -0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{span_x + ext:.4f} {span_y + ext:.4f} 0.02</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{span_x + ext:.4f} {span_y + ext:.4f} 0.02</size></box></geometry>
          <material><ambient>0.55 0.50 0.45 1</ambient><diffuse>0.65 0.58 0.50 1</diffuse></material>
        </visual>
      </link>
    </model>
""")

    # ---- 墙体（直接由 wall_rects 的数据产出，保证与校验完全一致）----
    for name, x0, x1, y0, y1 in walls:
        length_x, length_y = x1 - x0, y1 - y0
        parts.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {WALL_HEIGHT / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{length_x:.4f} {length_y:.4f} {WALL_HEIGHT:.4f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{length_x:.4f} {length_y:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.92 0.92 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>
""")

    # ---- 门洞过梁（纯装饰，不参与校验）----
    for d in doors:
        if "wall_y" in d:
            parts.append(lintel(f"lintel_{d['id']}", d["center_x"], d["wall_y"],
                                float(d.get("width", 0.9)), inte, axis="x"))
        elif "wall_x" in d:
            parts.append(lintel(f"lintel_{d['id']}", d["wall_x"], d["center_y"],
                                float(d.get("width", 0.9)), inte, axis="y"))

    # ---- 家具 ----
    obj_sdf, truth = gen_objects(objs)
    for t in truth:
        t["room"] = room_of(t, rooms)

    # 房间清单（供评测按房间聚合）
    rooms_meta = [{
        "id": r["id"], "label": r["label"],
        "x_min": r["x_min"], "x_max": r["x_max"],
        "y_min": r["y_min"], "y_max": r["y_max"],
        "center": [round((r["x_min"] + r["x_max"]) / 2, 3),
                   round((r["y_min"] + r["y_max"]) / 2, 3)],
        "size": [round(r["x_max"] - r["x_min"], 3), round(r["y_max"] - r["y_min"], 3)],
    } for r in rooms]

    light_points = []
    for r in rooms:
        light_points.append(((float(r["x_min"]) + float(r["x_max"])) / 2,
                             (float(r["y_min"]) + float(r["y_max"])) / 2))
    lights_sdf = "".join(
        f'''    <light type="point" name="light_room_{i:02d}">
      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT - 0.45:.3f} 0 0 0</pose>
      <diffuse>0.72 0.72 0.68 1</diffuse><attenuation><range>{max(span_x, span_y) / 2.2:.2f}</range></attenuation>
    </light>\n'''
        for i, (x, y) in enumerate(light_points))

    world = f"""<?xml version="1.0" ?>
<!--
  三室一厅一卫一厨住宅场景 —— 由 config/apartment.yaml 自动生成，请勿手工编辑。
  重新生成： ros2 run semnav_bringup generate_world.py --layout <yaml> ...
  生成器同时输出语义真值清单（物体类别 + 精确位姿），供 Phase 2 评测使用。
-->
<sdf version="1.8">
  <world name="semnav_apartment">
    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver><type>quick</type><iters>150</iters><precon_iters>0</precon_iters>
          <sor>1.3999999999999999</sor><use_dynamic_moi_rescaling>true</use_dynamic_moi_rescaling></solver>
        <constraints><cfm>1.0000000000000001e-05</cfm><erp>0.20000000000000001</erp>
          <contact_max_correcting_vel>2000</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer></constraints>
      </ode>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <!-- 渲染雷达与相机。缺了它 /scan 静默无输出（Gazebo Harmonic 无 cpu_lidar 系统）。 -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <!-- 接触/碰撞条传感器必需；模型级 contact 插件是空操作。 -->
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

    <scene>
      <ambient>0.45 0.45 0.45</ambient>
      <background>0.72 0.75 0.80</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -1</direction>
    </light>
    <!-- 室内补光：每个房间一个与地图坐标同尺度的点光源 -->
{lights_sdf}

{''.join(parts)}
{obj_sdf}  </world>
</sdf>
"""

    out_world = Path(args.out_world)
    out_world.parent.mkdir(parents=True, exist_ok=True)
    out_world.write_text(world, encoding="utf-8")

    truth_doc = {
        "layout": str(args.layout),
        "coordinate_frame": "world; x=east, y=north; yaw=deg CCW",
        "bounding_box": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        # 墙体也是「真值几何」：建图评估时必须计入，否则每一面被正确建出的墙
        # 都会被当成幽灵障碍（实测踩过：83.8% 的障碍被误判为幽灵，其实全是墙）。
        "walls": [
            {"id": name,
             "x_min": round(x0, 4), "x_max": round(x1, 4),
             "y_min": round(y0, 4), "y_max": round(y1, 4)}
            for name, x0, x1, y0, y1 in walls
        ],
        "rooms": rooms_meta,
        "doors": doors,
        "spawn": cfg.get("spawn", {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}),
        "nav_goals": cfg.get("nav_goals", []),
        "objects": truth,
    }
    out_truth = Path(args.out_truth)
    out_truth.parent.mkdir(parents=True, exist_ok=True)
    out_truth.write_text(
        json.dumps(truth_doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    n_models = world.count("<model name=") + world.count("<include>")
    print(f"✓ world   -> {out_world}  ({len(world)} 字节, {n_models} 个模型)")
    print(f"✓ truth   -> {out_truth}  ({len(truth)} 个语义对象, {len(rooms)} 个房间, {len(walls)} 面墙)")
    for t in truth:
        print(f"    {t['class']:<16} room={str(t['room']):<16} ({t['x']:+.2f},{t['y']:+.2f})")


if __name__ == "__main__":
    main()
