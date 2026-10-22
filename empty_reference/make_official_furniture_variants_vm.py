#!/usr/bin/env python3
"""Make named, dimensioned variants from OSRF's official Cabinet model."""
import re, shutil, sys, xml.etree.ElementTree as ET
from pathlib import Path

# Dimensions are already in the 3x apartment frame.
VARIANTS = {
    # Real-size wardrobe; mesh and collision remain the official OSRF Cabinet asset.
    "official_wardrobe": (1.20, 0.60, 2.00),
    "official_x3_nightstand": (1.35, 1.20, 1.65),
    "official_x3_wardrobe": (3.60, 1.80, 6.00),
    "official_x3_bathroom_vanity": (1.80, 1.50, 2.55),
}
BASE = (0.45, 0.45, 1.02)

def values(t):
    return re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", t or "")

def fmt(v):
    return " ".join(f"{x:.8g}" for x in v)

def transform(e, sx, sy, sz):
    tag=e.tag.rsplit('}',1)[-1]
    if tag in {'pose','size'} and e.text:
        v=[float(x) for x in values(e.text)]
        if tag=='pose' and len(v)>=3:
            v[0]*=sx; v[1]*=sy; v[2]*=sz
        else:
            for i in range(min(3,len(v))): v[i]*=(sx,sy,sz)[i]
        e.text=fmt(v)
    elif tag in {'radius','length'} and e.text and values(e.text):
        # Cabinet only uses boxes; retain a sensible conservative z scaling.
        e.text=f'{float(values(e.text)[0])*sz:.8g}'
    for c in e: transform(c,sx,sy,sz)

def simplify(root):
    for m in root.iter():
        if m.tag.rsplit('}',1)[-1]!='material': continue
        for c in list(m):
            if c.tag.rsplit('}',1)[-1] in {'script','pbr'}: m.remove(c)
        if not any(c.tag.rsplit('}',1)[-1]=='diffuse' for c in m):
            d=ET.SubElement(m,'diffuse'); d.text='0.54 0.32 0.16 1'

def main():
    root=Path(sys.argv[1]).expanduser()
    src=root/'gazebo_cabinet'
    for name,target in VARIANTS.items():
        dst=root/name
        if dst.exists(): shutil.rmtree(dst)
        shutil.copytree(src,dst)
        sx,sy,sz=[target[i]/BASE[i] for i in range(3)]
        sdf=dst/'model.sdf'; tree=ET.parse(sdf); transform(tree.getroot(),sx,sy,sz); simplify(tree.getroot()); tree.write(sdf,encoding='utf-8',xml_declaration=True)
        (dst/'model.config').write_text(f'''<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><author><name>OSRF gazebo_models</name></author><description>Official OSRF Cabinet mesh, dimensioned as a {name}.</description></model>\n''',encoding='utf-8')
        print(name,target)
if __name__=='__main__': main()
