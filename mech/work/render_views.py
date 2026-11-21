# -*- coding: utf-8 -*-
"""Render orthographic density projections of the STEP point cloud."""
import numpy as np
from PIL import Image, ImageDraw

pts = np.load(r"E:\code\raspberrypi\mech\work\pts.npy").astype(np.float64)
N = len(pts)
print("points", N)

def project(a, b, fname, size=1400, flip_b=True, grid=10.0, title=""):
    a, b = pts[:, a], pts[:, b]
    lo = np.array([a.min(), b.min()]); hi = np.array([a.max(), b.max()])
    pad = 15.0
    lo -= pad; hi += pad
    span = max(hi - lo)
    scale = (size - 60) / span
    img = Image.new("RGB", (size, size), "white")
    d = ImageDraw.Draw(img)
    # grid every 10 mm
    for gv in np.arange(np.floor(lo[0] / grid) * grid, hi[0], grid):
        x = 30 + (gv - lo[0]) * scale
        col = (255, 90, 90) if abs(gv) < 1e-9 else (235, 235, 235)
        d.line([(x, 30), (x, size - 30)], fill=col)
    for gv in np.arange(np.floor(lo[1] / grid) * grid, hi[1], grid):
        y = 30 + (gv - lo[1]) * scale
        col = (90, 90, 255) if abs(gv) < 1e-9 else (235, 235, 235)
        d.line([(30, y), (size - 30, y)], fill=col)
    px = (30 + (a - lo[0]) * scale).astype(int)
    py = (30 + ((hi[1] - b) if flip_b else (b - lo[1])) * scale).astype(int)
    acc = np.zeros((size, size), dtype=np.int32)
    np.add.at(acc, (np.clip(py, 0, size - 1), np.clip(px, 0, size - 1)), 1)
    dens = np.clip(np.log1p(acc) / np.log1p(acc.max() + 1) * 255, 0, 255).astype(np.uint8)
    col = 255 - dens  # dark where dense
    mask = acc > 0
    arr = np.array(img)
    arr[mask] = np.stack([col[mask]] * 3, axis=1)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    d.text((34, 8), f"{title}  span {hi[0]-lo[0]:.1f} x {hi[1]-lo[1]:.1f} mm, grid 10mm", fill="black")
    img.save(fname)
    print("wrote", fname, "a-range", round(lo[0], 1), round(hi[0], 1), "b-range", round(lo[1], 1), round(hi[1], 1))

project(0, 1, r"E:\code\raspberrypi\mech\work\view_top_XY.png", title="XY (top view)")
project(1, 2, r"E:\code\raspberrypi\mech\work\view_side_YZ.png", title="YZ (side view)")
project(0, 2, r"E:\code\raspberrypi\mech\work\view_front_XZ.png", title="XZ (front view)")
