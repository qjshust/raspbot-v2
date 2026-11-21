"""Shared fixtures for the raspbot_base test suite.

- Injects FakeSMBus into ``sys.modules['smbus']`` BEFORE any test imports the
  vendor Raspbot_Lib, so no test ever touches /dev/i2c-1 (SW-G06..G08).
- Provides the ``make_base_node`` factory for the ROS component tests; rclpy
  is imported lazily so the pure unit tests still run on Windows without ROS.
"""

import os
import sys
import types

import pytest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_TEST_DIR)
for _d in (_TEST_DIR, _PKG_ROOT):
    if _d not in sys.path:
        sys.path.insert(0, _d)

from support import FakeSMBus  # noqa: E402  (needs _TEST_DIR on sys.path)

# Inject the fake bus module before anything imports Raspbot_Lib.
_smbus = types.ModuleType('smbus')
_smbus.SMBus = FakeSMBus
sys.modules['smbus'] = _smbus


@pytest.fixture(autouse=True)
def _fresh_fake_bus():
    FakeSMBus.reset()
    yield


def _require_rclpy():
    try:
        import rclpy
    except Exception:
        pytest.skip('rclpy is not available on this machine')
    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception as exc:
        pytest.skip('rclpy.init failed: %s' % exc)
    return rclpy


@pytest.fixture
def make_base_node():
    """Factory that builds a RaspbotBase against the injected FakeSMBus.

    Requires ROS 2; the fixture (and the component tests that use it) are
    skipped on the offline Windows box.
    """
    _require_rclpy()
    from raspbot_base.base_node import RaspbotBase

    nodes = []

    def _make():
        node = RaspbotBase()
        nodes.append(node)
        return node

    yield _make

    for node in nodes:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass


@pytest.fixture(scope='session', autouse=True)
def _shutdown_rclpy_at_end():
    yield
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
