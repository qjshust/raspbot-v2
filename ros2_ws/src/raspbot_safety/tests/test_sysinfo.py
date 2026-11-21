"""raspbot_safety.sysinfo 纯函数的离线单测（无需 ROS / 不读真实 /proc）。"""

import pytest

from raspbot_safety import sysinfo


# ---------------------------------------------------------------- loadavg

def test_parse_loadavg_ok():
    assert sysinfo.parse_loadavg('0.15 0.20 0.25 1/234 5678') == pytest.approx(
        (0.15, 0.20, 0.25))


def test_parse_loadavg_garbage():
    assert sysinfo.parse_loadavg('') == (None, None, None)
    assert sysinfo.parse_loadavg('abc') == (None, None, None)
    assert sysinfo.parse_loadavg(None) == (None, None, None)


# ---------------------------------------------------------------- cpu stat

CPU_SAMPLE = ('cpu  100 20 300 4000 500 10 5 0 0 0\n'
              'cpu0 50 10 150 2000 250 5 2 0 0 0\n'
              'intr 12345 0 0 0\n')


def test_parse_cpu_stat_takes_aggregate_line():
    vals = sysinfo.parse_cpu_stat(CPU_SAMPLE)
    assert vals == [100, 20, 300, 4000, 500, 10, 5, 0, 0, 0]


def test_parse_cpu_stat_empty():
    assert sysinfo.parse_cpu_stat('') is None
    assert sysinfo.parse_cpu_stat('intr 1 2 3\n') is None
    assert sysinfo.parse_cpu_stat(None) is None


def test_cpu_percent_idle_only():
    prev = [10, 0, 0, 100, 0, 0, 0, 0, 0, 0]
    cur = [10, 0, 0, 200, 0, 0, 0, 0, 0, 0]
    assert sysinfo.cpu_percent(prev, cur) == pytest.approx(0.0)


def test_cpu_percent_half_busy():
    # 总计 +200，其中 idle +100 → 使用率 50%
    prev = [10, 0, 0, 100, 0, 0, 0, 0, 0, 0]
    cur = [110, 0, 0, 200, 0, 0, 0, 0, 0, 0]
    assert sysinfo.cpu_percent(prev, cur) == pytest.approx(50.0)


def test_cpu_percent_iowait_counts_as_idle():
    prev = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    cur = [0, 0, 0, 0, 100, 0, 0, 0, 0, 0]
    assert sysinfo.cpu_percent(prev, cur) == pytest.approx(0.0)


def test_cpu_percent_invalid():
    assert sysinfo.cpu_percent(None, [1, 2]) is None
    assert sysinfo.cpu_percent([1, 2, 3, 4], [1, 2, 3]) is None
    # 时间没有前进（总差值 <= 0）
    assert sysinfo.cpu_percent([1, 2, 3, 4], [1, 2, 3, 4]) is None


# ---------------------------------------------------------------- meminfo

MEM_SAMPLE = ('MemTotal:        8123456 kB\n'
              'MemFree:         1234567 kB\n'
              'MemAvailable:    3456789 kB\n'
              'Buffers:           12345 kB\n'
              'Cached:           234567 kB\n')


def test_parse_meminfo():
    mem = sysinfo.parse_meminfo(MEM_SAMPLE)
    assert mem['memtotal'] == 8123456
    assert mem['memavailable'] == 3456789


def test_mem_percent():
    mem = sysinfo.parse_meminfo(MEM_SAMPLE)
    expected = 100.0 * (8123456 - 3456789) / 8123456
    assert sysinfo.mem_percent(mem) == pytest.approx(expected)


def test_mem_percent_fallback_without_memavailable():
    mem = {'memtotal': 1000, 'memfree': 600, 'buffers': 100, 'cached': 100}
    assert sysinfo.mem_percent(mem) == pytest.approx(20.0)


def test_mem_percent_empty():
    assert sysinfo.mem_percent({}) is None


# ---------------------------------------------------------------- temp

def test_parse_temp_mc():
    assert sysinfo.parse_temp_mc('45123\n') == pytest.approx(45.123)
    assert sysinfo.parse_temp_mc('  45000  \n') == pytest.approx(45.0)


def test_parse_temp_mc_garbage():
    assert sysinfo.parse_temp_mc('') is None
    assert sysinfo.parse_temp_mc('abc') is None
    assert sysinfo.parse_temp_mc(None) is None


# ---------------------------------------------------------------- uptime

def test_parse_uptime():
    assert sysinfo.parse_uptime('12345.67 54321.00\n') == pytest.approx(12345.67)


def test_parse_uptime_garbage():
    assert sysinfo.parse_uptime('') is None
    assert sysinfo.parse_uptime(None) is None


def test_fmt_uptime():
    assert sysinfo.fmt_uptime(0) == '0s'
    assert sysinfo.fmt_uptime(65) == '1m 5s'
    assert sysinfo.fmt_uptime(3725) == '1h 2m 5s'
    assert sysinfo.fmt_uptime(90065) == '1d 1h 1m 5s'
    assert sysinfo.fmt_uptime(None) == 'unknown'
