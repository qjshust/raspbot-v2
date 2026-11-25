"""RASPBOT-V2 PC 控制台服务端（WP-C2）。

只使用 Python 标准库。对外可编程入口：

.. code-block:: python

    from console_server import create_server, load_config
    cfg = load_config()
    server = create_server(cfg, demo=True)
    server.start()
    print(server.url)
    ...
    server.stop()
"""

from .config import (
    BROWSER_CHOICES,
    DEFAULT_CONFIG_PATH,
    DEFAULTS,
    VERSION,
    Config,
    ConfigError,
    load_config,
    parse_pi_target,
    save_config,
)
from .http_server import ConsoleServer, create_server
from .upstream import Upstream, UpstreamError, UpstreamTimeout, UpstreamUnreachable

__version__ = VERSION

__all__ = [
    "VERSION",
    "BROWSER_CHOICES",
    "DEFAULT_CONFIG_PATH",
    "DEFAULTS",
    "Config",
    "ConfigError",
    "load_config",
    "parse_pi_target",
    "save_config",
    "ConsoleServer",
    "create_server",
    "Upstream",
    "UpstreamError",
    "UpstreamTimeout",
    "UpstreamUnreachable",
]
