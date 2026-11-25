"""测试辅助：临时目录放在包内（离线沙箱可能禁止写系统临时目录）。"""

import atexit
import os
import shutil
import uuid

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_HERE)
_TMP_ROOT = os.path.join(_PACKAGE_ROOT, '.test_tmp')

os.makedirs(_TMP_ROOT, exist_ok=True)

_CREATED = []


def temp_dir(prefix='rbt_'):
    """创建测试用临时目录（atexit 时清理）。

    不用 ``tempfile.mkdtemp``：部分 Windows 沙箱对 mkdtemp 生成的目录有额外
    写限制；``os.makedirs`` 在离线/CI/沙箱环境下行为一致。
    """
    path = os.path.join(_TMP_ROOT, prefix + uuid.uuid4().hex)
    os.makedirs(path, exist_ok=True)
    _CREATED.append(path)
    return path


@atexit.register
def _cleanup():
    for path in _CREATED:
        shutil.rmtree(path, ignore_errors=True)
    try:
        if os.path.isdir(_TMP_ROOT) and not os.listdir(_TMP_ROOT):
            os.rmdir(_TMP_ROOT)
    except OSError:
        pass
