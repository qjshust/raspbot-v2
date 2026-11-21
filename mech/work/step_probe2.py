# -*- coding: utf-8 -*-
"""Second pass: vertex Z levels (plate faces), Z-normal hole circles with XY pattern."""
import re, sys, collections
import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
SRC = r"E:\开发资料库\RASPBOT-V2 AI视觉小车\20.硬件资料\3D模型\raspbot v2麦轮版.STEP"
t = open(SRC, "rb").read().decode("gbk")

# ---------- points ----------
pts = {}
for m in re.finditer(r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'([^']*)'\s*,\s*\(([^)]*)\)\s*\)", t, re.S):
    xyz = [float(v) for v in m.group(3).split(",")]
    pts[m.group(1)] = xyz
print("points:", len(pts))

# ---------- vertex points ----------
verts = []
for m in re.finditer(r"#(\d+)\s*=\s*VERTEX_POINT\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*\)", t):
    p = pts.get(m.group(3))
    if p:
        verts.append(p)
verts = np.array(verts)
print("vertices:", verts.shape)
np.save(r"E:\code\raspberrypi\mech\work\verts.npy", verts)

# ---------- circles ----------
circ = [(m.group(3), float(m.group(4))) for m in re.finditer(
    r"#(\d+)\s*=\s*CIRCLE\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*,\s*([-\d.E+]+)\s*\)", t)]
print("circles:", len(circ))

ap = {}
for m in re.finditer(r"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'([^']*)'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)", t):
    ap[m.group(1)] = (m.group(3), m.group(4), m.group(5))
dirs = {}
for m in re.finditer(r"#(\d+)\s*=\s*DIRECTION\s*\(\s*'([^']*)'\s*,\s*\(([^)]*)\)\s*\)", t):
    dirs[m.group(1)] = [float(v) for v in m.group(3).split(",")]

rows = []
for loc, r in circ:
    a = ap.get(loc)
    if not a:
        continue
    p = pts.get(a[0]); d = dirs.get(a[1])
    if p is None or d is None or len(p) != 3 or len(d) != 3:
        continue
    rows.append((r, p[0], p[1], p[2], d[0], d[1], d[2]))
hz = np.array(rows, dtype=float)
print("circles with axis:", hz.shape)
np.save(r"E:\code\raspberrypi\mech\work\circles.npy", hz)

zn = hz[np.abs(np.abs(hz[:, 6]) - 1.0) < 0.02]
print("Z-normal circles:", len(zn))
print("\nradius histogram (>=3):")
hist = collections.Counter(np.round(zn[:, 0], 2))
for r, c in sorted(hist.items()):
    if c >= 3:
        print(f"  r={r:5.2f}  n={c:4d}")

def classify(r):
    for name, ref, tol in [("M2", 1.1, 0.07), ("M2.5", 1.35, 0.1), ("M3", 1.6, 0.09),
                           ("M3.2", 1.7, 0.06), ("M4", 2.15, 0.15), ("M5", 2.6, 0.15),
                           ("D6", 3.0, 0.1), ("D8", 4.0, 0.15)]:
        if abs(r - ref) <= tol:
            return name
    return f"r{r:.2f}"

print("\n=== Z levels of hole-like circles (r 1.0-2.3) ===")
sel = zn[(zn[:, 0] > 1.0) & (zn[:, 0] < 2.3)]
zl = collections.Counter(np.round(sel[:, 3], 1))
for z, c in sorted(zl.items()):
    if c >= 4:
        sub = sel[np.abs(sel[:, 3] - z) < 0.06]
        xs, ys = sub[:, 1], sub[:, 2]
        print(f"\nZ={z:7.1f}  n={c:3d}  X[{xs.min():7.1f},{xs.max():7.1f}] Y[{ys.min():7.1f},{ys.max():7.1f}]")
        order = np.lexsort((xs, ys))
        line = [f"({sub[i,1]:6.1f},{sub[i,2]:6.1f}){classify(sub[i,0])}" for i in order]
        for i in range(0, len(line), 4):
            print("    " + "  ".join(line[i:i+4]))

print("\n=== vertex Z histogram (0.5 mm bins, top 25) ===")
hist, edges = np.histogram(verts[:, 2], bins=np.arange(-2, verts[:, 2].max() + 2, 0.5))
for i in np.argsort(hist)[::-1][:25]:
    print(f"  Z {edges[i]:7.1f}  n={hist[i]:6d}")
