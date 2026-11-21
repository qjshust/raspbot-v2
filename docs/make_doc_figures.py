# -*- coding: utf-8 -*-
"""把结构渲染图压成适合嵌入 Word 的 JPEG（1200px 宽、质量 90）。"""
import os
from PIL import Image

SRC = r'E:\code\raspberrypi\mech\render'
DST = os.path.join(SRC, 'doc')
os.makedirs(DST, exist_ok=True)

for name in ('00_assembly_top.png', '01_lidar_shelf_iso.png', '02_imu_exploded.png'):
    im = Image.open(os.path.join(SRC, name)).convert('RGB')
    w = 1200
    h = round(im.height * w / im.width)
    im = im.resize((w, h), Image.LANCZOS)
    out = os.path.join(DST, name.replace('.png', '.jpg'))
    im.save(out, 'JPEG', quality=90, optimize=True)
    print(f'{name} -> {out}  {os.path.getsize(out) // 1024} KB')
