# -*- coding: utf-8 -*-
"""Inspect the two vendor zips: structure and any battery-related code."""
import re
import zipfile
from pathlib import Path

ZIPS = [
    Path(r"E:\开发资料库\RASPBOT-V2 AI视觉小车\18.ROS2基础教程\配套源码\ros2_ws.zip"),
    Path(r"E:\开发资料库\RASPBOT-V2 AI视觉小车\19.程序源码汇总\程序源码汇总.zip"),
]
PAT = re.compile(r"battery|Battery|BATT|voltage|Voltage|电量|电压|power_supply", re.I)
EXTS = (".py", ".cpp", ".hpp", ".h", ".c", ".yaml", ".yml", ".msg", ".srv")

for z in ZIPS:
    print("=" * 70)
    print(z.name, f"{z.stat().st_size/1024/1024:.1f} MB")
    try:
        with zipfile.ZipFile(z) as zf:
            infos = zf.infolist()
            print("entries:", len(infos))
            for info in infos[:15]:
                print("  ", info.filename)
            text_files = [i for i in infos if not i.is_dir() and i.filename.lower().endswith(EXTS)]
            print("text files:", len(text_files))
            hits = 0
            for info in text_files:
                try:
                    data = zf.read(info).decode("utf-8", errors="replace")
                except Exception:
                    continue
                for i, line in enumerate(data.splitlines(), 1):
                    if PAT.search(line):
                        hits += 1
                        if hits <= 25:
                            print(f"  HIT {info.filename}:{i}: {line.strip()[:140]}")
            print("hits:", hits)
    except Exception as exc:  # noqa: BLE001
        print("  failed:", exc)
