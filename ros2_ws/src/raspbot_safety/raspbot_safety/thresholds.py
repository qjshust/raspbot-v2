"""thresholds — 安全与诊断阈值纯函数（离线可测）。

不依赖 ROS：级别用整数常量（0=OK, 1=WARN, 2=ERROR），便于纯函数断言。
注意：Lyrical 新版 rosidl_generator_py 绑定中 DiagnosticStatus.level 是
bytes（b'\\x00' 等），safety_node 构造消息时经 _norm_level() 转成 bytes。
"""

OK = 0
WARN = 1
ERROR = 2

TEMP_WARN_C = 80.0      # 设计 §7.4：>80 °C 降频并告警
CPU_WARN_PCT = 90.0
MEM_WARN_PCT = 90.0
RANGE_WARN_M = 0.35     # 最近 1 s 内最小距离 <0.35 m 告警
IMU_STALE_S = 1.0       # 设计 §7.4：IMU 掉线判定
ODOM_STALE_S = 0.5
SCAN_STALE_S = 0.5      # 设计 §7.4：雷达掉线判定


def is_stale(age_s, limit_s):
    """age_s > limit_s 视为失联（设计 §7.4 口径）。"""
    return age_s > limit_s


def stale_level(age_s, limit_s, count, never):
    """传感器时效 → (level, message)。never=True 表示从未收到消息。"""
    if never:
        return WARN, 'no data received yet'
    if is_stale(age_s, limit_s):
        return WARN, 'stale: no data for %.2f s (limit %.2f s), %d msgs' % (
            age_s, limit_s, count)
    return OK, 'age %.2f s (limit %.2f s), %d msgs' % (age_s, limit_s, count)


def temp_level(temp_c, warn_c=TEMP_WARN_C):
    """SoC 温度 → (level, message)。None 表示读取失败。"""
    if temp_c is None:
        return WARN, 'temperature unreadable'
    if temp_c >= warn_c:
        return WARN, 'SoC %.1f C >= %.1f C (throttle risk)' % (temp_c, warn_c)
    return OK, 'SoC %.1f C' % temp_c


def cpu_level(usage_pct, warn_pct=CPU_WARN_PCT):
    if usage_pct is None:
        return WARN, 'cpu usage unreadable'
    if usage_pct >= warn_pct:
        return WARN, 'cpu %.1f %% >= %.0f %%' % (usage_pct, warn_pct)
    return OK, 'cpu %.1f %%' % usage_pct


def mem_level(used_pct, warn_pct=MEM_WARN_PCT):
    if used_pct is None:
        return WARN, 'memory unreadable'
    if used_pct >= warn_pct:
        return WARN, 'mem %.1f %% >= %.0f %%' % (used_pct, warn_pct)
    return OK, 'mem %.1f %%' % used_pct


def range_level(min_range, warn_m=RANGE_WARN_M):
    """最近窗口内最小距离 → (level, message)。None 表示窗口内无数据。"""
    if min_range is None:
        return OK, 'no range data in window'
    if min_range < warn_m:
        return WARN, 'min range %.3f m < %.2f m' % (min_range, warn_m)
    return OK, 'min range %.3f m' % min_range


def estop_level(active):
    """急停状态 → (level, message)。"""
    if active:
        return WARN, 'ESTOP ENGAGED'
    return OK, 'released'
