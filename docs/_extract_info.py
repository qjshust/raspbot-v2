# -*- coding: utf-8 -*-
"""Extract text from the vendor PDF that may describe battery/status publishing."""
from pathlib import Path

from pypdf import PdfReader

base = Path(r"E:\开发资料库\RASPBOT-V2 AI视觉小车")
targets = [
    base / "13.ROS2-机器人底盘与控制" / "1.机器人信息发布.pdf",
    base / "03.小车基础教程" / "02.按键读取" / "02.按键读取.pdf",
]

out = []
for p in targets:
    out.append("=" * 70)
    out.append(f"{p.name} exists={p.exists()}")
    if not p.exists():
        continue
    r = PdfReader(str(p))
    out.append(f"pages={len(r.pages)}")
    for i, page in enumerate(r.pages):
        t = (page.extract_text() or "").strip()
        if t:
            out.append(f"--- page {i + 1} ---")
            out.append(t)

Path(r"E:\code\raspberrypi\docs\_vendor_info.txt").write_text("\n".join(out), encoding="utf-8")
print("written", sum(len(x) for x in out))
