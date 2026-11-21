# -*- coding: utf-8 -*-
"""Probe the vendor STEP assembly: bounding box, Z-level structure, product names,
circular edges (holes) grouped by Z band. Pure regex parse, no CAD kernel."""
import re, json, sys, io, collections
import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

SRC = r"E:\开发资料库\RASPBOT-V2 AI视觉小车\20.硬件资料\3D模型\raspbot v2麦轮版.STEP"

raw = open(SRC, "rb").read()
for enc in ("utf-8", "gbk", "latin-1"):
    try:
        t = raw.decode(enc)
        print("decoded with", enc)
        break
    except UnicodeDecodeError:
        continue
print("file chars:", len(t))

# --- product / part names -------------------------------------------------
names = re.findall(r"PRODUCT\s*\(\s*'([^']*)'", t)
print("\nPRODUCT count:", len(names))
for n, c in collections.Counter(names).most_common(60):
    print(f"  {c:3d} x {n}")

# --- all cartesian points -------------------------------------------------
pts = np.array(re.findall(
    r"CARTESIAN_POINT \(\s*'[^']*'\s*,\s*\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)\s*\)", t),
    dtype=float)
print("\ncartesian points:", pts.shape)
if pts.size:
    lo, hi = pts.min(0), pts.max(0)
    print("bbox lo:", np.round(lo, 2), "hi:", np.round(hi, 2), "size:", np.round(hi - lo, 2))

# --- Z histogram (1 mm bins) ---------------------------------------------
z = pts[:, 2]
hist, edges = np.histogram(z, bins=np.arange(np.floor(z.min()), np.ceil(z.max()) + 1, 1.0))
print("\nZ histogram (bin: count) top 40:")
for i in np.argsort(hist)[::-1][:40]:
    print(f"  Z {edges[i]:8.1f}-{edges[i+1]:8.1f} : {hist[i]:7d}")

# --- circles -> holes -----------------------------------------------------
circ = re.findall(r"#(\d+)\s*=\s*CIRCLE\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*,\s*([-\d.E+]+)\s*\)", t)
print("\nCIRCLE count:", len(circ))
place = {}
for m in re.finditer(
        r"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)", t):
    place[m.group(1)] = m.group(2)
pcoords = {}
for m in re.finditer(
        r"#(\d+)\s*=\s*CARTESIAN_POINT \(\s*'[^']*'\s*,\s*\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)\s*\)", t):
    pcoords[m.group(1)] = (float(m.group(2)), float(m.group(3)), float(m.group(4)))

holes = []
for cid, axis, r in circ:
    r = float(r)
    ap = place.get(axis)
    if ap is None or ap not in pcoords:
        continue
    x, y, zz = pcoords[ap]
    holes.append((r, x, y, zz))
holes = np.array(holes)
print("holes with placement:", holes.shape)

# keep hole-sized radii
for rmin, rmax, label in [(1.4, 1.9, "M3 clearance/tap ~1.5-1.75"),
                          (2.0, 2.4, "M4 ~2.0-2.2"),
                          (2.7, 3.3, "M6 / 6mm boss"),
                          (3.4, 4.2, "8mm boss")]:
    sel = holes[(holes[:, 0] >= rmin) & (holes[:, 0] <= rmax)]
    print(f"\nradius {label}: {len(sel)}")
    if len(sel):
        zz = np.round(sel[:, 3], 1)
        for zv, cnt in sorted(collections.Counter(zz).items()):
            print(f"   Z={zv:8.1f}  n={cnt:3d}")

np.save(r"E:\code\raspberrypi\mech\work\pts.npy", pts.astype(np.float32))
np.save(r"E:\code\raspberrypi\mech\work\holes.npy", holes)
