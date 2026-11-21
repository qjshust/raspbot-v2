# -*- coding: utf-8 -*-
"""Extract embedded JPEGs from a PDF (crude but effective for PPT-exported PDFs)."""
import re, sys, os
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
src = r"E:\开发资料库\RASPBOT-V2 AI视觉小车\01.Raspbot V2简介\05.舵机云台安装\舵机云台安装.pdf"
out = r"E:\code\raspberrypi\mech\work\servo_pdf"
os.makedirs(out, exist_ok=True)
d = open(src, "rb").read()
print("pdf bytes", len(d))
n = 0
for m in re.finditer(rb"\xff\xd8\xff", d):
    s = m.start()
    e = d.find(b"\xff\xd9", s)
    if e == -1:
        continue
    if e - s > 20000:
        n += 1
        open(os.path.join(out, f"img{n:02d}.jpg"), "wb").write(d[s:e + 2])
print("jpegs", n)
