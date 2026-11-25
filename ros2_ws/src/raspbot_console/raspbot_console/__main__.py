"""``python -m raspbot_console`` 入口（等价于 ``console_bridge``）。"""

import sys

from .bridge import main

if __name__ == '__main__':
    sys.exit(main())
