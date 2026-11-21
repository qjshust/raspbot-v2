# -*- coding: utf-8 -*-
"""Scan notebook sources for battery/power handling and list notebook names."""
import re
import zipfile
from pathlib import Path

ZIP = Path(r"E:\开发资料库\RASPBOT-V2 AI视觉小车\19.程序源码汇总\程序源码汇总.zip")
PAT = re.compile(r"battery|Battery|BATT|voltage|Voltage|电量|电压|power_supply|电池", re.I)

names = []
hits = []
with zipfile.ZipFile(ZIP) as zf:
    for info in zf.infolist():
        if info.is_dir():
            continue
        low = info.filename.lower()
        if not low.endswith((".ipynb", ".py")):
            continue
        names.append(info.filename)
        try:
            data = zf.read(info).decode("utf-8", errors="replace")
        except Exception:
            continue
        for i, line in enumerate(data.splitlines(), 1):
            if PAT.search(line):
                hits.append(f"{info.filename}:{i}: {line.strip()[:150]}")

print(f"notebooks/scripts: {len(names)}")
print("--- names containing battery/power/voltage/bms ---")
for n in names:
    if re.search(r"batt|power|volt|bms|charge|电池|电量", n, re.I):
        print("  ", n)
print(f"--- keyword hits: {len(hits)} ---")
for h in hits[:40]:
    print(h)
