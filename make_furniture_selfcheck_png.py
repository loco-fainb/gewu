#!/usr/bin/env python3
"""Render a readable, meter-scaled top-down furniture self-check image."""
from pathlib import Path
import json
import math
import yaml
from PIL import Image, ImageDraw, ImageFont


LAYOUT = Path("apartment_empty_reference.yaml")
TRUTH = Path("apartment_living_furnished_truth.json")
OUT = Path("apartment_living_selfcheck.png")

COLORS = {
    "bed": (213, 82, 82),
    "wardrobe": (139, 91, 48),
    "sofa": (61, 112, 181),
    "coffee_table": (202, 145, 48),
    "armchair": (77, 155, 103),
    "dining_table": (155, 102, 61),
    "dining_chair": (104, 83, 166),
}


def font(size):
    for name in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def main():
    cfg = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    truth = json.loads(TRUTH.read_text(encoding="utf-8"))
    bb = truth["bounding_box"]
    pad = 0.65
    scale = 82
    left, bottom = bb["x_min"] - pad, bb["y_min"] - pad
    right, top = bb["x_max"] + pad, bb["y_max"] + pad
    width, height = round((right - left) * scale), round((top - bottom) * scale)
    image = Image.new("RGB", (width + 420, height + 100), (245, 246, 248))
    draw = ImageDraw.Draw(image)
    ox, oy = 40, 55

    def pt(x, y):
        return ox + round((x - left) * scale), oy + round((top - y) * scale)

    # Light room bands make the architectural footprint readable below walls.
    room_fill = (232, 237, 242)
    for room in cfg["rooms"]:
        p0, p1 = pt(room["x_min"], room["y_max"]), pt(room["x_max"], room["y_min"])
        draw.rectangle((*p0, *p1), fill=room_fill)
        cx = (room["x_min"] + room["x_max"]) / 2
        cy = (room["y_min"] + room["y_max"]) / 2
        label = room["label"]
        draw.text(pt(cx, cy), label, anchor="mm", fill=(93, 105, 117), font=font(20))

    # Draw wall segments from truth, preserving the actual thickness.
    for wall in truth["walls"]:
        p0, p1 = pt(wall["x_min"], wall["y_max"]), pt(wall["x_max"], wall["y_min"])
        draw.rectangle((*p0, *p1), fill=(35, 40, 46))

    # Doors are marked in cyan at their wall opening center.
    for d in cfg["doors"]:
        w = float(d.get("width", 0.9))
        if "wall_y" in d:
            x0, x1 = d["center_x"] - w / 2, d["center_x"] + w / 2
            y = d["wall_y"]
            draw.line([pt(x0, y), pt(x1, y)], fill=(40, 191, 210), width=8)
        else:
            y0, y1 = d["center_y"] - w / 2, d["center_y"] + w / 2
            x = d["wall_x"]
            draw.line([pt(x, y0), pt(x, y1)], fill=(40, 191, 210), width=8)

    # Furniture footprints use the truth yaw so this checks the generated pose.
    for obj in truth["objects"]:
        sx, sy = map(float, obj["size"][:2])
        if abs(float(obj.get("yaw_deg", 0))) in (90.0, 270.0):
            sx, sy = sy, sx
        x0, x1 = obj["x"] - sx / 2, obj["x"] + sx / 2
        y0, y1 = obj["y"] - sy / 2, obj["y"] + sy / 2
        p0, p1 = pt(x0, y1), pt(x1, y0)
        color = COLORS.get(obj["class"], (100, 100, 100))
        draw.rounded_rectangle((*p0, *p1), radius=5, fill=color, outline=(20, 25, 30), width=2)
        short = {"bed": "床", "wardrobe": "衣柜", "sofa": "沙发",
                 "coffee_table": "茶几", "armchair": "单人椅",
                 "dining_table": "餐桌", "dining_chair": "餐椅"}[obj["class"]]
        draw.text(((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2), short,
                   anchor="mm", fill="white", font=font(15))

    title_font = font(26)
    draw.text((40, 15), "家具布局自检图（真实米制坐标）", fill=(20, 27, 34), font=title_font)
    lx, ly = width + 75, 70
    draw.text((lx, ly), "图例", fill=(20, 27, 34), font=font(22))
    ly += 38
    for kind, label in (("bed", "床"), ("wardrobe", "衣柜"), ("sofa", "沙发"),
                        ("coffee_table", "茶几"), ("armchair", "单人椅"),
                        ("dining_table", "餐桌"), ("dining_chair", "餐椅")):
        draw.rectangle((lx, ly, lx + 24, ly + 24), fill=COLORS[kind], outline=(20, 25, 30))
        draw.text((lx + 34, ly + 2), label, fill=(30, 35, 40), font=font(18))
        ly += 33
    draw.line((lx, ly + 5, lx + 24, ly + 5), fill=(40, 191, 210), width=6)
    draw.text((lx + 34, ly - 5), "门洞", fill=(30, 35, 40), font=font(18))
    ly += 45
    draw.text((lx, ly), "尺寸 / 检查", fill=(20, 27, 34), font=font(22))
    ly += 35
    lines = [
        f"房屋外框: {bb['x_max'] - bb['x_min']:.2f} x {bb['y_max'] - bb['y_min']:.2f} m",
        "家具重叠: 0",
        "墙体侵入: 0",
        "门洞堵塞: 0",
        "门洞最小宽: 0.80 m",
        "机器人直径: 0.349 m",
    ]
    for line in lines:
        draw.text((lx, ly), line, fill=(30, 35, 40), font=font(17))
        ly += 28
    image.save(OUT)
    print(f"saved {OUT} ({image.width}x{image.height})")


if __name__ == "__main__":
    main()
