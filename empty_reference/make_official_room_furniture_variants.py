#!/usr/bin/env python3
"""Create real-size variants from the project's official OpenRobotics models."""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

VARIANTS = {
    "official_cafe_table": ("official_x3_cafe_table", 1.0 / 3.0),
    "official_room_armchair": ("official_x3_armchair", 1.0 / 3.0),
}

def nums(text):
    return re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", text or "")

def fmt(values):
    return " ".join(f"{v:.8g}" for v in values)

def transform(element, factor):
    tag = element.tag.rsplit("}", 1)[-1]
    if tag in {"pose", "size", "scale"} and element.text and nums(element.text):
        values = [float(v) for v in nums(element.text)]
        if tag == "pose" and len(values) >= 3:
            values[:3] = [v * factor for v in values[:3]]
        else:
            values = [v * factor for v in values]
        element.text = fmt(values)
    elif tag in {"radius", "length"} and element.text and nums(element.text):
        element.text = f"{float(nums(element.text)[0]) * factor:.8g}"
    for child in element:
        transform(child, factor)

    # The Fuel variants store mesh dimensions in the DAE itself.  Scale the
    # visual mesh explicitly so the rendered geometry matches the collision
    # envelope after making a real-size variant.
    if tag == "mesh" and not any(c.tag.rsplit("}", 1)[-1] == "scale" for c in element):
        scale = ET.Element("scale")
        scale.text = fmt([factor, factor, factor])
        element.append(scale)

def sanitize_dae(path):
    """Keep official geometry while removing VM-incompatible texture bindings."""
    tree = ET.parse(path)
    root = tree.getroot()
    for parent in list(root.iter()):
        for child in list(parent):
            tag = child.tag.rsplit("}", 1)[-1]
            if tag == "library_images":
                parent.remove(child)
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "diffuse":
            continue
        for child in list(element):
            if child.tag.rsplit("}", 1)[-1] == "texture":
                element.remove(child)
        if not list(element):
            color = ET.SubElement(element, "color")
            color.text = "0.55 0.36 0.18 1"
    tree.write(path, encoding="utf-8", xml_declaration=True)

def main():
    root = Path(sys.argv[1]).expanduser()
    for output, (source, factor) in VARIANTS.items():
        src = root / source
        dst = root / output
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        for dae in dst.rglob("*.dae"):
            sanitize_dae(dae)
        sdf = dst / "model.sdf"
        tree = ET.parse(sdf)
        root_element = tree.getroot()
        transform(root_element, factor)
        for model in root_element.iter():
            if model.tag.rsplit("}", 1)[-1] == "model":
                name = model.find("name")
                if name is not None:
                    name.text = output
        tree.write(sdf, encoding="utf-8", xml_declaration=True)
        (dst / "model.config").write_text(
            f'''<?xml version="1.0"?>\n<model><name>{output}</name><version>1.0</version>\n'''
            f'''<sdf version="1.6">model.sdf</sdf><author><name>OpenRobotics / semnav</name></author>\n'''
            f'''<description>OpenRobotics Fuel official furniture mesh at real dimensions.</description></model>\n''',
            encoding="utf-8")
        print(f"created {output} from {source}")

if __name__ == "__main__":
    main()
