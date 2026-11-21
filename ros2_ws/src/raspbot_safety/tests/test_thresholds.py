"""raspbot_safety.thresholds 纯函数的离线单测（无需 ROS）。"""

import pytest

from raspbot_safety import thresholds

OK, WARN, ERROR = thresholds.OK, thresholds.WARN, thresholds.ERROR


# ---------------------------------------------------------------- stale

def test_is_stale_boundary():
    assert not thresholds.is_stale(1.0, 1.0)   # 恰好等于阈值：不判失联
    assert thresholds.is_stale(1.001, 1.0)
    assert not thresholds.is_stale(0.499, 0.5)
    assert thresholds.is_stale(0.5 + 1e-9, 0.5)


def test_stale_level_never_received():
    level, message = thresholds.stale_level(0.0, 1.0, 0, True)
    assert level == WARN
    assert 'no data' in message


def test_stale_level_fresh_and_stale():
    level, _ = thresholds.stale_level(0.4, 1.0, 42, False)
    assert level == OK
    level, _ = thresholds.stale_level(1.2, 1.0, 42, False)
    assert level == WARN


# ---------------------------------------------------------------- temp

def test_temp_level():
    assert thresholds.temp_level(45.1, 80.0) == (OK, 'SoC 45.1 C')
    level, message = thresholds.temp_level(80.0, 80.0)   # 设计 §7.4：>80 告警
    assert level == WARN
    assert '80.0' in message
    level, _ = thresholds.temp_level(85.0, 80.0)
    assert level == WARN
    level, _ = thresholds.temp_level(None)
    assert level == WARN


# ---------------------------------------------------------------- cpu/mem

def test_cpu_level():
    assert thresholds.cpu_level(42.0, 90.0)[0] == OK
    assert thresholds.cpu_level(90.0, 90.0)[0] == WARN
    assert thresholds.cpu_level(None)[0] == WARN


def test_mem_level():
    assert thresholds.mem_level(42.0, 90.0)[0] == OK
    assert thresholds.mem_level(95.0, 90.0)[0] == WARN
    assert thresholds.mem_level(None)[0] == WARN


# ---------------------------------------------------------------- range

def test_range_level():
    assert thresholds.range_level(None)[0] == OK          # 无数据不告警（mock 模式）
    assert thresholds.range_level(0.349, 0.35)[0] == WARN  # <0.35 m
    assert thresholds.range_level(0.35, 0.35)[0] == OK     # 边界
    assert thresholds.range_level(1.5, 0.35)[0] == OK


# ---------------------------------------------------------------- estop

def test_estop_level():
    assert thresholds.estop_level(False)[0] == OK
    level, message = thresholds.estop_level(True)
    assert level == WARN
    assert 'ESTOP' in message
