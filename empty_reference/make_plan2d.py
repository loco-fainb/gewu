#!/usr/bin/env python3
"""Render a labelled 2D floor-plan figure with furniture footprint blocks.

Rooms carry their Chinese name; every piece of furniture is drawn as a filled
block sized to its real footprint.

Guarantees:
  * Footprints come from `generate_world.rotated_footprint()` — the same
    function that builds the SDF and the occupancy map — so a block here is by
    construction the rectangle Gazebo puts there.
  * Each room label is placed at the point of maximum clearance from furniture
    and walls inside that room, so labels never sit on top of a block.
"""
import importlib.util
import os
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    HERE, '..', 'src', 'semnav_bringup', 'config', 'apartment.yaml')
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/apartment_plan2d.png'
SCALE = float(sys.argv[3]) if len(sys.argv) > 3 else 78.0

GEN = os.path.join(HERE, '..', 'src', 'semnav_bringup', 'scripts',
                   'generate_world.py')
spec = importlib.util.spec_from_file_location('gw', GEN)
gw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gw)

FONT_R = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_B = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'

BG = (24, 28, 36)
PLAN = (247, 245, 241)
WALL = (32, 36, 44)
ROOM_FILL = (252, 251, 248)
LABEL = (26, 30, 38)
GRID = (229, 227, 221)
FRAME = (126, 132, 144)
FG = (238, 238, 240)
CAPTION = (150, 156, 168)

FURN = {
    'bed':             (198, 130, 98),
    'wardrobe':        (150, 112, 88),
    'nightstand':      (178, 148, 118),
    'desk':            (198, 158, 90),
    'chair':           (92, 138, 176),
    'sofa':            (126, 156, 130),
    'coffee_table':    (174, 178, 126),
    'armchair':        (148, 172, 146),
    'tv_stand':        (102, 114, 136),
    'washer':          (118, 146, 168),
    'refrigerator':    (148, 156, 170),
    'stove':           (132, 94, 78),
    'sink':            (114, 154, 172),
    'cabinet':         (166, 138, 106),
    'dining_table':    (200, 158, 110),
    'dining_chair':    (108, 146, 180),
    'toilet':          (206, 210, 216),
    'bathroom_vanity': (174, 194, 202),
}
FURN_DEFAULT = (168, 168, 168)
FURN_EDGE = (58, 62, 72)

# Rooms shown on the figure.  The two 起居室 arms are the short arms of the
# same T-shaped circulation space as 走廊, so a second "起居室" label would
# double-label one open area.
SHOW = {
    'kitchen': '厨房',
    'dining_room': '餐厅',
    'entry': '玄关',
    'third_bedroom': '次卧二',
    'master_bedroom': '主卧',
    'second_bedroom': '次卧一',
    'south_balcony': '南阳台',
    'west_balcony': '西侧阳台',
    'bathroom': '卫生间',
    'living_room': '客厅',
    'corridor': '走廊',
}

cfg = yaml.safe_load(open(LAYOUT, encoding='utf-8'))
rooms = cfg['rooms']
walls = cfg['wall_segments']
objects = cfg.get('objects', [])

# ------------------------------------------------------------------ extents
xs, ys = [], []
for r in rooms:
    xs += [r['x_min'], r['x_max']]
    ys += [r['y_min'], r['y_max']]
for w in walls:
    xs += [w['x1'], w['x2']]
    ys += [w['y1'], w['y2']]
PAD = 0.30
x0, x1 = min(xs) - PAD, max(xs) + PAD
y0, y1 = min(ys) - PAD, max(ys) + PAD
W = int(round((x1 - x0) * SCALE))
H = int(round((y1 - y0) * SCALE))
MARGIN = 44
LEGEND_H = 116
CANVAS = (W + MARGIN * 2, H + MARGIN * 2 + LEGEND_H)

im = Image.new('RGB', CANVAS, BG)
d = ImageDraw.Draw(im)


def px(x):
    return MARGIN + (x - x0) * SCALE


def py(y):
    return MARGIN + (y1 - y) * SCALE


# ------------------------------------------------------------------ plate
d.rectangle([px(x0), py(y1), px(x1), py(y0)], fill=PLAN, outline=FRAME, width=2)
for r in rooms:
    d.rectangle([px(r['x_min']), py(r['y_max']), px(r['x_max']), py(r['y_min'])],
                fill=ROOM_FILL)

gx = 0.0
while gx <= 14.0:
    if x0 < gx < x1:
        d.line([px(gx), py(y1), px(gx), py(y0)], fill=GRID, width=1)
    gx += 1.0
gy = -2.0
while gy <= 10.0:
    if y0 < gy < y1:
        d.line([px(x0), py(gy), px(x1), py(gy)], fill=GRID, width=1)
    gy += 1.0


def spans_after_openings(a, b, openings):
    out = [(a, b)]
    for o in openings or []:
        c, wd = float(o['center']), float(o['width'])
        nxt = []
        for s, e in out:
            if c + wd / 2 <= s or c - wd / 2 >= e:
                nxt.append((s, e))
            else:
                if c - wd / 2 > s:
                    nxt.append((s, c - wd / 2))
                if c + wd / 2 < e:
                    nxt.append((c + wd / 2, e))
        out = nxt
    return [(s, e) for s, e in out if e - s > 1e-6]


# --------------------------------------------------------------- furniture
furn_rects = []
for o in objects:
    fx0, fx1, fy0, fy1 = gw.rotated_footprint(o)   # (x0, x1, y0, y1)
    furn_rects.append((fx0, fx1, fy0, fy1))
    col = FURN.get(o.get('class'), FURN_DEFAULT)
    d.rectangle([px(fx0), py(fy1), px(fx1), py(fy0)],
                fill=col, outline=FURN_EDGE, width=1)

# ------------------------------------------------------------------ walls
wall_rects = []
for w in walls:
    t = float(w.get('thickness', 0.12))
    if abs(w['y2'] - w['y1']) < 1e-9:
        yc = w['y1']
        for a, b in spans_after_openings(min(w['x1'], w['x2']),
                                         max(w['x1'], w['x2']),
                                         w.get('openings')):
            wall_rects.append((a, b, yc - t / 2, yc + t / 2))
            d.rectangle([px(a), py(yc + t / 2), px(b), py(yc - t / 2)], fill=WALL)
    else:
        xc = w['x1']
        for a, b in spans_after_openings(min(w['y1'], w['y2']),
                                         max(w['y1'], w['y2']),
                                         w.get('openings')):
            wall_rects.append((xc - t / 2, xc + t / 2, a, b))
            d.rectangle([px(xc - t / 2), py(b), px(xc + t / 2), py(a)], fill=WALL)


# ------------------------------------------------------------------ labels
_fc = {}


def font(sz, bold=True):
    key = (sz, bold)
    if key not in _fc:
        _fc[key] = ImageFont.truetype(FONT_B if bold else FONT_R, sz)
    return _fc[key]


def text_center(cx, cy, s, sz, fill=LABEL, bold=True, halo=None):
    f = font(sz, bold)
    bb = d.textbbox((0, 0), s, font=f)
    tx = cx - (bb[2] - bb[0]) / 2 - bb[0]
    ty = cy - (bb[3] - bb[1]) / 2 - bb[1]
    if halo:
        for dx in (-2, -1, 0, 1, 2):
            for dy in (-2, -1, 0, 1, 2):
                if dx or dy:
                    d.text((tx + dx, ty + dy), s, font=f, fill=halo)
    d.text((tx, ty), s, font=f, fill=fill)


def dist_to_rect(pxm, pym, r):
    rx0, rx1, ry0, ry1 = r
    dx = max(rx0 - pxm, 0.0, pxm - rx1)
    dy = max(ry0 - pym, 0.0, pym - ry1)
    return (dx * dx + dy * dy) ** 0.5


def best_label_point(r, text_w_m, text_h_m):
    """Grid-search the point in the room with the largest clearance.

    Clearance is the distance to the nearest furniture block or wall; the
    label needs half its width/height of room, so the search is restricted to
    an inset box and the score is the min over the label's own extent.
    """
    inset_x = text_w_m / 2 + 0.04
    inset_y = text_h_m / 2 + 0.04
    ax0, ax1 = r['x_min'] + inset_x, r['x_max'] - inset_x
    ay0, ay1 = r['y_min'] + inset_y, r['y_max'] - inset_y
    if ax1 < ax0:
        ax0 = ax1 = (r['x_min'] + r['x_max']) / 2
    if ay1 < ay0:
        ay0 = ay1 = (r['y_min'] + r['y_max']) / 2

    best, best_score = ((ax0 + ax1) / 2, (ay0 + ay1) / 2), -1.0
    nx = max(2, int((ax1 - ax0) / 0.08) + 1)
    ny = max(2, int((ay1 - ay0) / 0.08) + 1)
    for i in range(nx):
        cx = ax0 + (ax1 - ax0) * (i / (nx - 1) if nx > 1 else 0.5)
        for j in range(ny):
            cy = ay0 + (ay1 - ay0) * (j / (ny - 1) if ny > 1 else 0.5)
            # score the whole label box: sample its four corners + centre
            corners = [(cx - text_w_m / 2, cy - text_h_m / 2),
                       (cx + text_w_m / 2, cy - text_h_m / 2),
                       (cx - text_w_m / 2, cy + text_h_m / 2),
                       (cx + text_w_m / 2, cy + text_h_m / 2),
                       (cx, cy)]
            score = 1e9
            for sx, sy in corners:
                for rr in furn_rects:
                    score = min(score, dist_to_rect(sx, sy, rr))
                for rr in wall_rects:
                    score = min(score, dist_to_rect(sx, sy, rr))
            # keep the label away from the room border as well
            score = min(score,
                        cx - r['x_min'], r['x_max'] - cx,
                        cy - r['y_min'], r['y_max'] - cy)
            if score > best_score:
                best_score, best = score, (cx, cy)
    return best, best_score


for r in rooms:
    name = SHOW.get(r['id'])
    if not name:
        continue
    wpx = (r['x_max'] - r['x_min']) * SCALE
    hpx = (r['y_max'] - r['y_min']) * SCALE
    sz = int(min(25, wpx / (len(name) + 0.9), hpx * 0.32))
    sz = max(sz, 12)
    f = font(sz)
    tw = d.textlength(name, font=f) / SCALE
    th = (f.size * 1.15) / SCALE
    (cx, cy), score = best_label_point(r, tw, th)
    text_center(px(cx), py(cy), name, sz, halo=(252, 251, 248))

# ------------------------------------------------------------------ legend
ly = CANVAS[1] - LEGEND_H + 20
lx = MARGIN

sb_m = 2.0
d.line([lx, ly + 22, lx + sb_m * SCALE, ly + 22], fill=FG, width=4)
d.line([lx, ly + 15, lx, ly + 29], fill=FG, width=4)
d.line([lx + sb_m * SCALE, ly + 15, lx + sb_m * SCALE, ly + 29], fill=FG, width=4)
text_center(lx + sb_m * SCALE / 2, ly + 2, '2 m', 16, fill=FG)

items = [
    ('卧室', 'bed'), ('衣柜', 'wardrobe'), ('餐桌', 'dining_table'),
    ('餐椅', 'dining_chair'), ('沙发', 'sofa'), ('茶几', 'coffee_table'),
    ('电视柜', 'tv_stand'), ('书桌', 'desk'), ('灶台', 'stove'),
    ('水槽', 'sink'), ('冰箱', 'refrigerator'), ('坐便', 'toilet'),
    ('洗手台', 'bathroom_vanity'), ('洗衣机', 'washer'),
]
f_lab = font(15, False)
bx = lx + sb_m * SCALE + 42
bw, bh = 15, 13
row_y = ly + 14
for name, cls in items:
    lab_w = d.textlength(name, font=f_lab)
    if bx + bw + 5 + lab_w > CANVAS[0] - MARGIN:
        bx = lx
        row_y += 26
    d.rectangle([bx, row_y, bx + bw, row_y + bh],
                fill=FURN.get(cls, FURN_DEFAULT), outline=FURN_EDGE, width=1)
    d.text((bx + bw + 5, row_y - 3), name, font=f_lab, fill=(206, 211, 221))
    bx += bw + 5 + int(lab_w) + 20

d.text((MARGIN, row_y + 30),
       '家具块 = 布局 YAML 的真实 footprint（调用 generate_world.rotated_footprint，'
       '与仿真世界/占用栅格同源）· 栅格 0.05 m/格 · 户型 13.63 × 10.88 m · 北向上',
       font=font(14, False), fill=CAPTION)

# ------------------------------------------------------------------ compass
cnx, cny = CANVAS[0] - MARGIN - 26, MARGIN + 30
d.ellipse([cnx - 17, cny - 17, cnx + 17, cny + 17], outline=(200, 205, 215), width=2)
d.polygon([(cnx, cny - 13), (cnx - 6, cny + 5), (cnx + 6, cny + 5)], fill=(226, 92, 72))
text_center(cnx, cny - 30, 'N', 15, fill=FG)

im.save(OUT)
print('wrote', OUT, im.size, '|', len(objects), 'furniture blocks')
im.resize((im.width * 2, im.height * 2), Image.LANCZOS).save(
    OUT.replace('.png', '_2x.png'))
print('wrote', OUT.replace('.png', '_2x.png'))
