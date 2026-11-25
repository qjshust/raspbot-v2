"""PC 控制台配置：加载 / 校验 / 保存。

优先级：CLI 参数 > ``config.json`` > 内置默认值。

* 默认配置文件：``pc_console/config.json``（首次运行按默认值生成，已在 .gitignore 中忽略）
* 保存采用「同目录临时文件 + ``os.replace``」的原子写，尽量收紧权限（POSIX 0o600）
* 配置只包含可持久化的字段；``--verbose`` 等运行期参数不落盘
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
from dataclasses import dataclass, field
from pathlib import Path

VERSION = "1.0.0"

CONFIG_FILENAME = "config.json"
#: 本包位于 ``pc_console/console_server/config.py``，配置文件在 ``pc_console/config.json``
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / CONFIG_FILENAME

BROWSER_CHOICES = ("auto", "app", "default", "none")

DEFAULTS: dict = {
    "host": "127.0.0.1",
    "port": 8090,
    "pi_host": "192.168.50.98",
    "pi_port": 8080,
    "token": "",
    "browser": "auto",
}

_FORBIDDEN_HOST_CHARS = " \t\r\n/\\@?#\"'<>|"
_HOST_MAX_LEN = 253


class ConfigError(ValueError):
    """配置非法（面向用户的错误，带字段名便于 UI 展示）。"""

    def __init__(self, message: str, field: str | None = None):
        super().__init__(message)
        self.message = message
        self.field = field

    def to_dict(self) -> dict:
        return {"message": self.message, "field": self.field}


def validate_host(value, field: str = "host") -> str:
    """校验主机名 / IPv4 / IPv6 字面量（不允许带协议、路径、空白）。"""
    if not isinstance(value, str):
        raise ConfigError(f"{field} 必须是字符串", field)
    host = value.strip()
    if not host:
        raise ConfigError(f"{field} 不能为空", field)
    if len(host) > _HOST_MAX_LEN:
        raise ConfigError(f"{field} 过长（> {_HOST_MAX_LEN}）", field)
    if "://" in host:
        raise ConfigError(f"{field} 只填主机名或 IP，不要带协议前缀", field)
    for ch in _FORBIDDEN_HOST_CHARS:
        if ch in host:
            raise ConfigError(f"{field} 含非法字符 {ch!r}", field)
    if host.startswith("-") or host.endswith("-."):
        raise ConfigError(f"{field} 格式非法：{host!r}", field)
    return host


def validate_port(value, field: str = "port", allow_zero: bool = False) -> int:
    """校验端口：1..65535（监听端口 ``--port 0`` 允许 0=临时端口）。"""
    if isinstance(value, bool) or not isinstance(value, int):
        # 宽松接受纯数字字符串（CLI / JSON 手写配置）
        if isinstance(value, str) and value.strip().isdigit():
            value = int(value.strip())
        else:
            raise ConfigError(f"{field} 必须是整数端口", field)
    if allow_zero:
        if not (0 <= value <= 65535):
            raise ConfigError(f"{field} 必须在 0..65535 之间", field)
    else:
        if not (1 <= value <= 65535):
            raise ConfigError(f"{field} 必须在 1..65535 之间", field)
    return value


def validate_token(value, field: str = "token") -> str:
    """校验 token：允许空串；不允许换行 / 超长（避免请求头注入）。"""
    if value is None:
        return ""
    if not isinstance(value, str):
        raise ConfigError(f"{field} 必须是字符串", field)
    token = value.strip()
    if len(token) > 200:
        raise ConfigError(f"{field} 过长（> 200）", field)
    if any(ch in token for ch in "\r\n\x00"):
        raise ConfigError(f"{field} 含非法控制字符", field)
    return token


def validate_browser(value) -> str:
    if value is None:
        return DEFAULTS["browser"]
    if not isinstance(value, str):
        raise ConfigError("browser 必须是字符串", "browser")
    mode = value.strip().lower()
    if mode not in BROWSER_CHOICES:
        raise ConfigError(
            "browser 只能是 " + " / ".join(BROWSER_CHOICES), "browser"
        )
    return mode


def parse_pi_target(text: str) -> tuple[str, int]:
    """解析 CLI ``--pi <host>[:<port>]``。

    * ``192.168.50.98:8080`` → (host, 8080)
    * ``192.168.50.98`` → (host, 8080) 默认端口
    * ``[::1]:8080`` → IPv6 带括号
    """
    if not isinstance(text, str):
        raise ConfigError("--pi 必须是 host[:port] 形式", "pi")
    raw = text.strip()
    if not raw:
        raise ConfigError("--pi 不能为空", "pi")
    if raw.startswith("["):
        end = raw.find("]")
        if end < 0:
            raise ConfigError("--pi 的 IPv6 地址缺少 ']'", "pi")
        host = raw[1:end]
        rest = raw[end + 1 :]
        if rest.startswith(":"):
            port = validate_port(rest[1:], "pi_port")
        elif rest == "":
            port = DEFAULTS["pi_port"]
        else:
            raise ConfigError("--pi 格式非法", "pi")
        return validate_host(host, "pi_host"), port
    if raw.count(":") == 1:
        host, _, port_text = raw.partition(":")
        return validate_host(host, "pi_host"), validate_port(port_text, "pi_port")
    if raw.count(":") > 1:
        # 未加括号的 IPv6，不含端口
        return validate_host(raw, "pi_host"), DEFAULTS["pi_port"]
    return validate_host(raw, "pi_host"), DEFAULTS["pi_port"]


@dataclass
class Config:
    """控制台配置。可直接修改字段，再调用 :func:`save_config` 落盘。"""

    host: str = DEFAULTS["host"]
    port: int = DEFAULTS["port"]
    pi_host: str = DEFAULTS["pi_host"]
    pi_port: int = DEFAULTS["pi_port"]
    token: str = DEFAULTS["token"]
    browser: str = DEFAULTS["browser"]
    config_path: str = field(default=str(DEFAULT_CONFIG_PATH), repr=False)
    verbose: bool = field(default=False, compare=False, repr=False)

    # ---- 派生属性 -------------------------------------------------
    @property
    def target(self) -> dict:
        return {"host": self.pi_host, "port": self.pi_port}

    @property
    def target_tuple(self) -> tuple[str, int]:
        return self.pi_host, self.pi_port

    # ---- 序列化 ---------------------------------------------------
    def to_file_dict(self) -> dict:
        """可持久化字段（不包含 config_path / verbose）。"""
        return {
            "host": self.host,
            "port": self.port,
            "pi_host": self.pi_host,
            "pi_port": self.pi_port,
            "token": self.token,
            "browser": self.browser,
        }

    def copy(self) -> "Config":
        cfg = Config(
            host=self.host,
            port=self.port,
            pi_host=self.pi_host,
            pi_port=self.pi_port,
            token=self.token,
            browser=self.browser,
            config_path=self.config_path,
        )
        cfg.verbose = self.verbose
        return cfg

    # ---- 更新 / 校验 ----------------------------------------------
    def apply(self, updates: dict) -> "Config":
        """校验并就地更新字段。允许的键：host/port/pi_host/pi_port/token/browser。"""
        if not isinstance(updates, dict):
            raise ConfigError("配置更新必须是 JSON 对象")
        if "host" in updates and updates["host"] is not None:
            self.host = validate_host(updates["host"], "host")
        if "port" in updates and updates["port"] is not None:
            self.port = validate_port(updates["port"], "port", allow_zero=True)
        if "pi_host" in updates and updates["pi_host"] is not None:
            self.pi_host = validate_host(updates["pi_host"], "pi_host")
        if "pi_port" in updates and updates["pi_port"] is not None:
            self.pi_port = validate_port(updates["pi_port"], "pi_port")
        if "token" in updates:
            self.token = validate_token(updates["token"], "token")
        if "browser" in updates and updates["browser"] is not None:
            self.browser = validate_browser(updates["browser"])
        return self


def save_config(config: Config, path: str | os.PathLike | None = None) -> str:
    """原子写入配置文件，返回实际路径。

    写法：同目录 ``tempfile.mkstemp`` → 写 → fsync → ``os.replace``；
    POSIX 上把文件权限收紧到 0o600。
    """
    target = Path(path) if path else Path(config.config_path or DEFAULT_CONFIG_PATH)
    target = target.expanduser().resolve()
    payload = json.dumps(config.to_file_dict(), ensure_ascii=False, indent=2) + "\n"
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=target.name + ".", suffix=".tmp", dir=str(target.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(payload)
            fh.flush()
            os.fsync(fh.fileno())
        try:
            os.chmod(tmp_name, 0o600)
        except OSError:  # Windows 上 chmod 语义有限，忽略
            pass
        os.replace(tmp_name, str(target))
    except BaseException:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise
    return str(target)


def load_config(
    path: str | os.PathLike | None = None,
    overrides: dict | None = None,
    create: bool = True,
) -> Config:
    """加载配置。``overrides`` 为 CLI 覆盖项（值为 None 的忽略）。"""
    target = Path(path) if path else DEFAULT_CONFIG_PATH
    target = target.expanduser().resolve()
    config = Config(config_path=str(target))

    data: dict = {}
    if target.is_file():
        try:
            raw = target.read_text(encoding="utf-8")
        except OSError as exc:
            raise ConfigError(f"读取配置文件失败：{exc}", "config_path") from exc
        try:
            data = json.loads(raw) if raw.strip() else {}
        except json.JSONDecodeError as exc:
            raise ConfigError(
                f"配置文件不是合法 JSON（{target}）：{exc}", "config_path"
            ) from exc
        if not isinstance(data, dict):
            raise ConfigError("配置文件顶层必须是 JSON 对象", "config_path")
        # 兼容嵌套写法 {"pi": {"host":..}}，以及扁平写法
        nested = data.get("pi")
        if isinstance(nested, dict):
            data.setdefault("pi_host", nested.get("host"))
            data.setdefault("pi_port", nested.get("port"))
        web = data.get("web")
        if isinstance(web, dict):
            data.setdefault("host", web.get("host"))
            data.setdefault("port", web.get("port"))
    elif create:
        # 首次运行：按默认值生成 config.json（失败不影响启动）
        try:
            save_config(config, target)
        except OSError:
            pass

    config.apply(data)

    if overrides:
        clean = {k: v for k, v in overrides.items() if v is not None}
        if "demo" in clean:
            clean.pop("demo")
        config.apply(clean)

    return config


# /local/config 的一次性测试辅助（多线程环境下避免重复写）
_SAVE_LOCK = threading.Lock()


def save_config_locked(config: Config, path: str | os.PathLike | None = None) -> str:
    """串行化保存（HTTP 并发 POST /local/config 时使用）。"""
    with _SAVE_LOCK:
        return save_config(config, path)
