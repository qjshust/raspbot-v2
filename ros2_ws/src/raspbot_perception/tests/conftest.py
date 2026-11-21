"""pytest 配置：把包源码目录加入 sys.path（源码树内直接跑测试，无需安装）。"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
