"""pytest 入口：把包根目录加入 sys.path，使离线测试可直接 import raspbot_safety。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
