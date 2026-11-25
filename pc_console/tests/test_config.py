"""配置模块单元测试：默认值、优先级、校验、原子保存、CLI 解析。

注意：本环境的文件沙箱不允许向 ``tempfile.mkdtemp`` 创建的 0o700 目录写文件，
因此测试统一在 ``pc_console/_tmp/`` 下用 ``Path.mkdir`` 自建临时目录并清理。
"""

from __future__ import annotations

import json
import os
import shutil
import unittest
import uuid
from pathlib import Path

from console_server.config import (
    BROWSER_CHOICES,
    DEFAULT_CONFIG_PATH,
    DEFAULTS,
    VERSION,
    Config,
    ConfigError,
    load_config,
    parse_pi_target,
    save_config,
    validate_browser,
    validate_host,
    validate_port,
    validate_token,
)

_TMP_ROOT = Path(__file__).resolve().parent.parent / "_tmp"


def make_temp_dir(prefix: str = "cfg_") -> Path:
    path = _TMP_ROOT / (prefix + uuid.uuid4().hex[:10])
    path.mkdir(parents=True, exist_ok=True)
    return path


class ConfigTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = make_temp_dir()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    # ---- 默认值与文件 ---------------------------------------------
    def test_defaults_create_config_file(self) -> None:
        path = self.tmp / "config.json"
        self.assertFalse(path.exists())
        cfg = load_config(path, create=True)
        self.assertTrue(path.exists())
        self.assertEqual(cfg.host, DEFAULTS["host"])
        self.assertEqual(cfg.port, DEFAULTS["port"])
        self.assertEqual(cfg.pi_host, DEFAULTS["pi_host"])
        self.assertEqual(cfg.pi_port, DEFAULTS["pi_port"])
        self.assertEqual(cfg.token, "")
        self.assertEqual(cfg.browser, "auto")
        self.assertEqual(Path(cfg.config_path), path.resolve())
        # 文件内容合法且不多写私有字段
        data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            set(data), {"host", "port", "pi_host", "pi_port", "token", "browser"}
        )

    def test_file_overrides_defaults(self) -> None:
        path = self.tmp / "config.json"
        path.write_text(
            json.dumps(
                {
                    "host": "0.0.0.0",
                    "port": 8091,
                    "pi_host": "10.0.0.5",
                    "pi_port": 9000,
                    "token": "secret",
                    "browser": "none",
                }
            ),
            encoding="utf-8",
        )
        cfg = load_config(path, create=False)
        self.assertEqual(cfg.host, "0.0.0.0")
        self.assertEqual(cfg.port, 8091)
        self.assertEqual(cfg.pi_host, "10.0.0.5")
        self.assertEqual(cfg.pi_port, 9000)
        self.assertEqual(cfg.token, "secret")
        self.assertEqual(cfg.browser, "none")

    def test_cli_overrides_file(self) -> None:
        path = self.tmp / "config.json"
        path.write_text(
            json.dumps({"pi_host": "10.0.0.5", "pi_port": 9000, "token": "filetoken"}),
            encoding="utf-8",
        )
        cfg = load_config(
            path,
            overrides={"pi_host": "192.168.1.9", "pi_port": 8080, "port": 0},
            create=False,
        )
        self.assertEqual(cfg.pi_host, "192.168.1.9")
        self.assertEqual(cfg.pi_port, 8080)
        self.assertEqual(cfg.port, 0)
        # 未覆盖的字段保留配置文件值
        self.assertEqual(cfg.token, "filetoken")

    def test_nested_pi_format_compatible(self) -> None:
        path = self.tmp / "config.json"
        path.write_text(
            json.dumps({"pi": {"host": "172.16.0.2", "port": 8081}, "web": {"port": 8099}}),
            encoding="utf-8",
        )
        cfg = load_config(path, create=False)
        self.assertEqual(cfg.pi_host, "172.16.0.2")
        self.assertEqual(cfg.pi_port, 8081)
        self.assertEqual(cfg.port, 8099)

    def test_invalid_json_raises(self) -> None:
        path = self.tmp / "config.json"
        path.write_text("{not json", encoding="utf-8")
        with self.assertRaises(ConfigError):
            load_config(path, create=False)

    # ---- 校验 -----------------------------------------------------
    def test_validate_host(self) -> None:
        self.assertEqual(validate_host(" 192.168.50.98 "), "192.168.50.98")
        self.assertEqual(validate_host("raspbot.local"), "raspbot.local")
        self.assertEqual(validate_host("::1", "pi_host"), "::1")
        for bad in ("", "   ", "http://192.168.0.1", "a b", "a/b", "a?b", None, 123):
            with self.assertRaises(ConfigError, msg=f"host={bad!r}"):
                validate_host(bad)

    def test_validate_port(self) -> None:
        self.assertEqual(validate_port(8080), 8080)
        self.assertEqual(validate_port("8090"), 8090)
        self.assertEqual(validate_port(0, allow_zero=True), 0)
        for bad in (0, -1, 65536, "abc", None, True):
            with self.assertRaises(ConfigError, msg=f"port={bad!r}"):
                validate_port(bad)

    def test_validate_token_and_browser(self) -> None:
        self.assertEqual(validate_token(""), "")
        self.assertEqual(validate_token(" abc "), "abc")
        with self.assertRaises(ConfigError):
            validate_token("bad\ntoken")
        with self.assertRaises(ConfigError):
            validate_token(123)
        for mode in BROWSER_CHOICES:
            self.assertEqual(validate_browser(mode), mode)
        self.assertEqual(validate_browser("APP"), "app")
        with self.assertRaises(ConfigError):
            validate_browser("ie")
        with self.assertRaises(ConfigError):
            validate_browser(5)

    def test_apply_rejects_bad_values(self) -> None:
        cfg = Config()
        with self.assertRaises(ConfigError) as ctx:
            cfg.apply({"pi_port": 99999})
        self.assertEqual(ctx.exception.field, "pi_port")
        with self.assertRaises(ConfigError) as ctx2:
            cfg.apply({"pi_host": "http://x"})
        self.assertEqual(ctx2.exception.field, "pi_host")
        # 非法更新不改变原对象之前的字段（apply 先校验后赋值语义按字段）
        self.assertEqual(cfg.pi_host, DEFAULTS["pi_host"])

    def test_config_error_payload(self) -> None:
        err = ConfigError("坏端口", "pi_port")
        self.assertEqual(err.to_dict(), {"message": "坏端口", "field": "pi_port"})

    # ---- 保存 -----------------------------------------------------
    def test_save_roundtrip_and_atomic(self) -> None:
        path = self.tmp / "config.json"
        cfg = Config(host="127.0.0.1", port=8092, pi_host="1.2.3.4", pi_port=8080, token="t0", browser="default")
        cfg.config_path = str(path)
        saved = save_config(cfg)
        self.assertEqual(Path(saved), path.resolve())
        data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data["port"], 8092)
        self.assertEqual(data["token"], "t0")
        # 无残留临时文件
        leftovers = [p.name for p in self.tmp.iterdir() if p.name.endswith(".tmp")]
        self.assertEqual(leftovers, [])
        # 覆盖写入
        cfg.port = 8093
        save_config(cfg)
        data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data["port"], 8093)
        reloaded = load_config(path, create=False)
        self.assertEqual(reloaded.port, 8093)
        self.assertEqual(reloaded.pi_host, "1.2.3.4")

    def test_save_creates_parent_dir(self) -> None:
        path = self.tmp / "sub" / "config.json"
        cfg = Config()
        cfg.config_path = str(path)
        save_config(cfg)
        self.assertTrue(path.is_file())

    # ---- CLI 解析 -------------------------------------------------
    def test_parse_pi_target(self) -> None:
        self.assertEqual(parse_pi_target("192.168.50.98:8080"), ("192.168.50.98", 8080))
        self.assertEqual(parse_pi_target("192.168.50.98"), ("192.168.50.98", 8080))
        self.assertEqual(parse_pi_target(" [::1]:9090 "), ("::1", 9090))
        self.assertEqual(parse_pi_target("::1"), ("::1", 8080))
        with self.assertRaises(ConfigError):
            parse_pi_target("")
        with self.assertRaises(ConfigError):
            parse_pi_target("host:notaport")
        with self.assertRaises(ConfigError):
            parse_pi_target("host:70000")

    # ---- 仓库内示例文件 -------------------------------------------
    def test_example_config_file(self) -> None:
        example = Path(__file__).resolve().parent.parent / "config.example.json"
        self.assertTrue(example.is_file(), "缺少 pc_console/config.example.json")
        data = json.loads(example.read_text(encoding="utf-8"))
        self.assertEqual(set(data), {"host", "port", "pi_host", "pi_port", "token", "browser"})
        cfg = Config()
        cfg.apply(data)
        self.assertGreaterEqual(cfg.port, 1)

    def test_default_config_path_under_pc_console(self) -> None:
        self.assertEqual(DEFAULT_CONFIG_PATH.name, "config.json")
        self.assertEqual(DEFAULT_CONFIG_PATH.parent.name, "pc_console")
        self.assertEqual(VERSION, "1.0.0")


if __name__ == "__main__":
    unittest.main()
