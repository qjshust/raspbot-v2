"""sysinfo — /proc 与 /sys 文本解析纯函数（离线可测）。

约定：所有函数只接受字符串/列表输入并返回纯数据，不读真实文件；
文件读取由 safety_node 完成并把原始文本传进来，便于在任意平台离线单测。
"""


def parse_loadavg(text):
    """解析 /proc/loadavg → (load1, load5, load15)；失败返回 (None, None, None)。"""
    try:
        parts = text.split()
        return float(parts[0]), float(parts[1]), float(parts[2])
    except (ValueError, IndexError, AttributeError):
        return None, None, None


def parse_cpu_stat(text):
    """解析 /proc/stat 的聚合 `cpu` 行 → 各计数字段列表；失败返回 None。

    字段顺序：user nice system idle iowait irq softirq steal guest guest_nice。
    """
    if not text:
        return None
    for line in text.splitlines():
        parts = line.split()
        if parts and parts[0] == 'cpu':
            try:
                return [int(v) for v in parts[1:]]
            except ValueError:
                return None
    return None


def cpu_percent(prev, cur):
    """两次 /proc/stat 采样 → CPU 使用率（%）；样本无效返回 None。"""
    if not prev or not cur or len(prev) != len(cur):
        return None
    total_prev = sum(prev)
    total_cur = sum(cur)
    delta_total = total_cur - total_prev
    if delta_total <= 0:
        return None
    # idle(3) + iowait(4) 视为空闲
    idle_prev = prev[3] + prev[4] if len(prev) > 4 else prev[3]
    idle_cur = cur[3] + cur[4] if len(cur) > 4 else cur[3]
    delta_idle = idle_cur - idle_prev
    return 100.0 * max(0.0, (delta_total - delta_idle)) / delta_total


def parse_meminfo(text):
    """解析 /proc/meminfo → {key: 值(kB)}，key 为小写（memtotal 等）。"""
    out = {}
    if not text:
        return out
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            try:
                out[parts[0].rstrip(':').lower()] = int(parts[1])
            except ValueError:
                continue
    return out


def mem_percent(mem):
    """meminfo dict → 已用内存百分比；缺字段返回 None。"""
    total = mem.get('memtotal')
    available = mem.get('memavailable')
    if not total:
        return None
    if available is None:
        # 老内核回退：free + buffers + cached
        available = (mem.get('memfree', 0) + mem.get('buffers', 0)
                     + mem.get('cached', 0))
    return 100.0 * max(0.0, total - available) / total


def parse_temp_mc(text):
    """解析 sysfs 温度文件（m°C 整数文本）→ °C 浮点；失败返回 None。"""
    try:
        return float(text.strip()) / 1000.0
    except (ValueError, AttributeError):
        return None


def parse_uptime(text):
    """解析 /proc/uptime 首字段 → 秒；失败返回 None。"""
    try:
        return float(text.split()[0])
    except (ValueError, IndexError, AttributeError):
        return None


def fmt_uptime(seconds):
    """秒 → '1d 2h 3m 4s' 风格文本；None → 'unknown'。"""
    if seconds is None:
        return 'unknown'
    seconds = int(seconds)
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    parts = []
    if days:
        parts.append('%dd' % days)
    if hours:
        parts.append('%dh' % hours)
    if minutes:
        parts.append('%dm' % minutes)
    parts.append('%ds' % seconds)
    return ' '.join(parts)
