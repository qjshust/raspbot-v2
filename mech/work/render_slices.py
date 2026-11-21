# -*- coding: utf-8 -*-
"""XY slices of the vertex cloud at chosen Z bands, annotated in mm."""
import sys
import numpy as np
from PIL import Image, ImageDraw

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
verts = np.load(r"E:\code\raspberrypi\mech\work\verts.npy")
circles = np.load(r"E:\code\raspberrypi\mech\work\circles.npy")

X0, X1, Y0, Y1 = -180.0, 80.0, -220.0, 155.0
S = 6.0  # px per mm
W, H = int((X1 - X0) * S) + 90, int((Y1 - Y0) * S) + 70

def xy(px, py):
    return 60 + (px - X0) * S, 50 + (Y1 - py) * S

def base(title):
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    for g in range(-180, 81, 10):
        x, _ = xy(g, 0)
        d.line([(x, 40), (x, H - 30)], fill=(240, 240, 240) if g % 50 else (215, 215, 215))
        if g % 20 == 0:
            d.text((x - 8, H - 26), str(g), fill=(80, 80, 80))
    for g in range(-220, 156, 10):
        _, y = xy(0, g)
        d.line([(50, y), (W - 30, y)], fill=(240, 240, 240) if g % 50 else (215, 215, 215))
        if g % 20 == 0:
            d.text((14, y - 4), str(g), fill=(80, 80, 80))
    x0, y0 = xy(0, 0)
    d.line([(x0, 40), (x0, H - 30)], fill=(255, 140, 140))
    d.line([(50, y0), (W - 30, y0)], fill=(140, 140, 255))
    d.text((62, 12), title, fill="black")
    return img, d

bands = [
    ("z165_176", 165, 176), ("z176_181", 176, 181), ("z185_190", 185, 190),
    ("z200_212", 200, 212), ("z213_222", 213, 222), ("z228_246", 228, 246),
    ("z250_268", 250, 268), ("z272_292", 272, 292),
]
for name, z0, z1 in bands:
    img, d = base(f"{name}  Z {z0}-{z1} mm   (X right, +Y up)")
    m = (verts[:, 2] >= z0) & (verts[:, 2] <= z1)
    sub = verts[m]
    for px, py, pz in sub:
        x, y = xy(px, py)
        d.ellipse([x - 1.2, y - 1.2, x + 1.2, y + 1.2], fill=(40, 40, 40))
    mc = (circles[:, 3] >= z0) & (circles[:, 3] <= z1) & (np.abs(np.abs(circles[:, 6]) - 1) < 0.02)
    for r, px, py, pz, *_ in circles[mc]:
        x, y = xy(px, py)
        rr = max(r * S, 2.5)
        if 1.2 <= r <= 2.3:
            d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=(220, 30, 30), width=2)
    img.save(rf"E:\code\raspberrypi\mech\work\{name}.png")
    print(name, "verts", len(sub), "holes", int(mc.sum()))
