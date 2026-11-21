# -*- coding: utf-8 -*-
"""Extract EDGE_CURVE segments/arcs from the STEP and render true wireframe
layer drawings (top view) per Z band. Also list long horizontal edges."""
import re, sys, collections
import numpy as np
from PIL import Image, ImageDraw

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
SRC = r"E:\开发资料库\RASPBOT-V2 AI视觉小车\20.硬件资料\3D模型\raspbot v2麦轮版.STEP"
t = open(SRC, "rb").read().decode("gbk")

pts = {}
for m in re.finditer(r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'([^']*)'\s*,\s*\(([^)]*)\)\s*\)", t, re.S):
    pts[m.group(1)] = [float(v) for v in m.group(3).split(",")]

vp = {}
for m in re.finditer(r"#(\d+)\s*=\s*VERTEX_POINT\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*\)", t):
    vp[m.group(1)] = pts.get(m.group(3))

circ = {}
for m in re.finditer(r"#(\d+)\s*=\s*CIRCLE\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*,\s*([-\d.E+]+)\s*\)", t):
    circ[m.group(1)] = (m.group(3), float(m.group(4)))
ap = {}
for m in re.finditer(r"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)", t):
    ap[m.group(1)] = m.group(3)

edges = []
for m in re.finditer(
        r"#(\d+)\s*=\s*EDGE_CURVE\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*\.(T|F)\.\s*\)", t):
    a = vp.get(m.group(3)); b = vp.get(m.group(4))
    if a is None or b is None:
        continue
    cid = m.group(5)
    if cid in circ:
        loc, r = circ[cid]
        cen = pts.get(ap.get(loc, ""), None)
        edges.append((a, b, r, cen))
    else:
        edges.append((a, b, 0.0, None))
print("edges:", len(edges))

# ---- long horizontal straight edges -------------------------------------
print("\n=== straight edges longer than 25 mm, |dz| < 0.2 ===")
segs = []
for a, b, r, cen in edges:
    if r > 0:
        continue
    a = np.array(a); b = np.array(b)
    L = np.linalg.norm(b - a)
    if L > 25 and abs(b[2] - a[2]) < 0.2:
        segs.append((a, b, L))
segs.sort(key=lambda s: -s[2])
for a, b, L in segs[:40]:
    print(f"  Z={a[2]:7.2f}  L={L:6.1f}  ({a[0]:7.1f},{a[1]:7.1f}) -> ({b[0]:7.1f},{b[1]:7.1f})")
np.save(r"E:\code\raspberrypi\mech\work\edges_straight.npy",
        np.array([[a[0], a[1], a[2], b[0], b[1], b[2]] for a, b, L in segs], dtype=float))

# ---- wireframe layer drawings -------------------------------------------
X0, X1, Y0, Y1 = -175.0, 75.0, -215.0, 150.0
S = 6.5
W, H = int((X1 - X0) * S) + 100, int((Y1 - Y0) * S) + 80
def xy(px, py):
    return 70 + (px - X0) * S, 55 + (Y1 - py) * S

bands = [("w165_180", 164, 180), ("w180_195", 180, 195), ("w195_212", 195, 212),
         ("w212_225", 212, 225), ("w225_248", 225, 248), ("w248_270", 248, 270),
         ("w270_300", 270, 300), ("w300_400", 300, 400)]
for name, z0, z1 in bands:
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    for g in range(-170, 76, 10):
        x, _ = xy(g, 0)
        d.line([(x, 45), (x, H - 30)], fill=(238, 238, 238) if g % 50 else (208, 208, 208))
        if g % 20 == 0:
            d.text((x - 10, H - 25), ("%d" % g), fill=(70, 70, 70))
    for g in range(-210, 151, 10):
        _, y = xy(0, g)
        d.line([(60, y), (W - 30, y)], fill=(238, 238, 238) if g % 50 else (208, 208, 208))
        if g % 20 == 0:
            d.text((24, y - 4), ("%d" % g), fill=(70, 70, 70))
    x0, y0 = xy(0, 0)
    d.line([(x0, 45), (x0, H - 30)], fill=(255, 150, 150))
    d.line([(60, y0), (W - 30, y0)], fill=(150, 150, 255))
    d.text((72, 14), f"{name}  Z {z0}-{z1} mm  wireframe top view (mm grid)", fill="black")
    nseg = narc = 0
    for a, b, r, cen in edges:
        za = a[2]
        if not (z0 <= za <= z1):
            continue
        pa = xy(a[0], a[1])
        if r > 0 and cen is not None:
            # approximate arc with polyline between endpoints
            c = np.array(cen)
            va = np.array(a)[:2] - c[:2]; vb = np.array(b)[:2] - c[:2]
            ang_a = np.arctan2(va[1], va[0]); ang_b = np.arctan2(vb[1], vb[0])
            dang = (ang_b - ang_a + np.pi) % (2 * np.pi) - np.pi
            steps = max(6, int(abs(dang) * r * S / 2))
            poly = []
            for i in range(steps + 1):
                ang = ang_a + dang * i / steps
                poly.append(xy(c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)))
            d.line(poly, fill=(0, 0, 0), width=1)
            narc += 1
        else:
            pb = xy(b[0], b[1])
            d.line([pa, pb], fill=(0, 0, 0), width=1)
            nseg += 1
    img.save(rf"E:\code\raspberrypi\mech\work\{name}.png")
    print(f"{name}: {nseg} segs, {narc} arcs")
