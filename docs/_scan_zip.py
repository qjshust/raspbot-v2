# -*- coding: utf-8 -*-
"""Search the vendor ROS2 workspace zip for any battery/voltage handling."""
import re
import zipfile
from pathlib import Path

ZIP = Path(r"E:\开发资料库\RASPBOT-V2 AI视觉小车\18.ROS2基础教程\配套源码\ros2_ws.zip")
PAT = re.compile(r"battery|Battery|BATT|voltage|Voltage|电量|电压|power_supply", re.I)
EXTS = (".py", ".cpp", ".hpp", ".h", ".c", ".yaml", ".yml", ".xml", ".msg", ".srv", ".txt", ".md")

hits = []
names = 0
with zipfile.ZipFile(ZIP) as z:
    for info in z.infolist():
        if info.is_dir():
            continue
        if not info.filename.lower().endswith(EXTS):
            continue
        names += 1
        try:
            data = z.read(info).decode("utf-8", errors="replace")
        except Exception:
            continue
        for i, line in enumerate(data.splitlines(), 1):
            if PAT.search(line):
                hits.append(f"{info.filename}:{i}: {line.strip()[:160]}")

print(f"scanned {names} text files, {len(hits)} hits")
for h in hits[:60]:
    print(h)
