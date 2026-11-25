"""Wire API v1 的纯逻辑：错误模型、字段校验、JSON/SSE 序列化。

本模块只依赖 Python 标准库，导入期不依赖 rclpy，可离线单元测试。
契约：``docs/PC上位机_设计文档_v1.md`` §3.0。

设计要点：

- 统一错误响应 ``{"error": {"code", "message", "detail"}}``；
- 所有 HTTP 状态与错误码集中在此，便于 HTTP 层与 controller 复用；
- JSON 统一 **compact separators**（``(",", ":")``）且不做 ASCII 转义
  （中文日志直接输出 UTF-8，HTTP 侧声明 charset=utf-8）；
- 请求体的所有数值校验都要求 **有限实数**，显式拒绝 bool（bool 是 int 的子类）
  与 NaN/Inf，避免把脏数据带进 ROS 消息。
"""

from __future__ import annotations

import json
import math
import os
import re

#: Wire API 版本
API_VERSION = 1

#: 地图名规则（设计文档 §3.3.2）
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_.-]{1,40}$')

#: 运行模式
MODES = ('base', 'slam', 'nav')

#: JSON 请求体上限（设计文档 §3.0：≤ 64 KB）
MAX_JSON_BODY = 64 * 1024

#: SSE 客户端上限（设计文档 §3.0：≤ 4，超出 503）
MAX_SSE_CLIENTS = 4

#: 网关日志保留上限（GET /api/logs 最多 300 条）
MAX_GATEWAY_LOGS = 300

#: 会话 stdout 环形缓冲上限（设计文档 §5.2：最多 500 行）
MAX_SESSION_LOGS = 500


class ErrorCode:
    """错误码常量（契约里点名的沿用；其余为实现内自洽命名）。"""

    BAD_REQUEST = 'bad_request'
    BAD_TOKEN = 'bad_token'
    NOT_FOUND = 'not_found'
    METHOD_NOT_ALLOWED = 'method_not_allowed'
    PAYLOAD_TOO_LARGE = 'payload_too_large'
    INTERNAL = 'internal_error'

    # 状态冲突（409）
    BUSY = 'busy'
    ESTOP_ACTIVE = 'estop_active'
    EXTERNAL_STACK = 'external_stack'
    ATTACH_MODE = 'attach_mode'
    SESSION_IDLE = 'session_idle'
    SESSION_NOT_RUNNING = 'session_not_running'
    MAP_EXISTS = 'map_exists'
    NO_MAP = 'no_map'
    GOAL_ACTIVE = 'goal_active'
    NAV_NOT_READY = 'nav_not_ready'
    INITIALPOSE_REQUIRED = 'initialpose_required'

    # 503
    ROS_NOT_READY = 'ros_not_ready'
    NAV_REJECTED = 'nav_rejected'
    TOO_MANY_SSE = 'too_many_sse'


#: 各错误码默认 HTTP 状态（显式覆盖优先）
_DEFAULT_STATUS = {
    ErrorCode.BAD_REQUEST: 400,
    ErrorCode.BAD_TOKEN: 401,
    ErrorCode.NOT_FOUND: 404,
    ErrorCode.METHOD_NOT_ALLOWED: 405,
    ErrorCode.PAYLOAD_TOO_LARGE: 400,
    ErrorCode.INTERNAL: 500,
    ErrorCode.BUSY: 409,
    ErrorCode.ESTOP_ACTIVE: 409,
    ErrorCode.EXTERNAL_STACK: 409,
    ErrorCode.ATTACH_MODE: 409,
    ErrorCode.SESSION_IDLE: 409,
    ErrorCode.SESSION_NOT_RUNNING: 409,
    ErrorCode.MAP_EXISTS: 409,
    ErrorCode.NO_MAP: 409,
    ErrorCode.GOAL_ACTIVE: 409,
    ErrorCode.NAV_NOT_READY: 409,
    ErrorCode.INITIALPOSE_REQUIRED: 409,
    ErrorCode.ROS_NOT_READY: 503,
    ErrorCode.NAV_REJECTED: 503,
    ErrorCode.TOO_MANY_SSE: 503,
}


class ApiError(Exception):
    """带 HTTP 状态与契约错误码的异常，HTTP 层直接转成 JSON 错误响应。"""

    def __init__(self, code, message, status=None, detail=None):
        self.code = str(code)
        self.message = str(message)
        self.status = int(status if status is not None
                          else _DEFAULT_STATUS.get(self.code, 400))
        self.detail = detail if detail is None else dict(detail)
        super().__init__(self.message)

    def payload(self):
        """构造契约错误负载（设计文档 §3.0）。"""
        err = {'code': self.code, 'message': self.message}
        if self.detail:
            err['detail'] = self.detail
        return {'error': err}

    def __repr__(self):  # pragma: no cover - 调试用
        return 'ApiError(%r, %r, status=%d)' % (self.code, self.message,
                                                self.status)


def error_payload(code, message, detail=None):
    """不通过异常时构造错误负载。"""
    return ApiError(code, message, detail=detail).payload()


# ---------------------------------------------------------------------------
# JSON 序列化
# ---------------------------------------------------------------------------

def sanitize(value):
    """递归清洗成可 JSON 序列化的数据。

    - NaN / +Inf / -Inf → ``None``（JSON 无对应表示，前端一律按“无数据”处理）；
    - tuple / set → list；bytes → 十六进制字符串（正常路径不会出现，防御性）；
    - 保留 bool/int/str/None。
    """
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return value
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).hex()
    if isinstance(value, dict):
        return {str(k): sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [sanitize(v) for v in value]
    # 兜底：转成字符串，保证 HTTP 层永远能序列化
    return str(value)


def json_dumps(obj):
    """契约要求的 compact JSON（无空格缩进），UTF-8 中文不转义。"""
    return json.dumps(sanitize(obj), separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False)


def json_loads(raw):
    """解析请求体；非法 JSON → ApiError(bad_request)。"""
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = bytes(raw).decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ApiError(ErrorCode.BAD_REQUEST,
                           '请求体不是合法的 UTF-8: %s' % exc)
    if isinstance(raw, str) and not raw.strip():
        return {}
    try:
        return json.loads(raw)
    except (ValueError, TypeError) as exc:
        raise ApiError(ErrorCode.BAD_REQUEST, '请求体不是合法 JSON: %s' % exc)


def sse_frame(event, payload):
    """SSE 单帧（bytes）：``event: <name>\\ndata: <json>\\n\\n``。

    注意：data 使用 compact JSON，保证单行（SSE 以换行分隔字段）。
    """
    data = json_dumps(payload)
    return ('event: %s\ndata: %s\n\n' % (str(event), data)).encode('utf-8')


def sse_retry_frame(millis=2000):
    """SSE 首帧 ``retry: N``（设计文档 §3.2：首帧前发 retry: 2000）。"""
    return ('retry: %d\n\n' % int(millis)).encode('utf-8')


# ---------------------------------------------------------------------------
# 数值 / 字段校验
# ---------------------------------------------------------------------------

def is_number(value):
    """有限实数（显式排除 bool 与 NaN/Inf）。"""
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def finite_or_none(value):
    """把有限数值转 float，其它（含 NaN/Inf/非数值）转 None。"""
    if is_number(value):
        return float(value)
    return None


def clamp(value, low, high):
    """把数值夹到 [low, high]，先转 float。"""
    v = float(value)
    if v < low:
        return float(low)
    if v > high:
        return float(high)
    return v


def require_object(body, what='请求体'):
    """校验 JSON 对象。"""
    if body is None:
        return {}
    if not isinstance(body, dict):
        raise ApiError(ErrorCode.BAD_REQUEST, '%s 必须是 JSON 对象' % what)
    return body


def _missing(field, what):
    return ApiError(ErrorCode.BAD_REQUEST, '%s 缺少字段 %r' % (what, field),
                    detail={'field': field})


def _bad_type(field, expect, what):
    return ApiError(ErrorCode.BAD_REQUEST,
                    '%s 的 %r 必须是 %s' % (what, field, expect),
                    detail={'field': field})


def require_number(body, field, what='请求体', default=None, required=True,
                   low=None, high=None):
    """读取有限数值字段（可选 clamp）。

    ``required=False`` 且字段不存在时返回 ``default``；存在但非法仍报 400。
    ``low``/``high`` 给出时做 clamp（设计文档 §3.3.4：超限不报错，clamp 后执行）。
    """
    if field not in body or body[field] is None:
        if required:
            raise _missing(field, what)
        return default
    value = body[field]
    if not is_number(value):
        raise _bad_type(field, '有限数值', what)
    v = float(value)
    if low is not None or high is not None:
        lo = -math.inf if low is None else float(low)
        hi = math.inf if high is None else float(high)
        v = clamp(v, lo, hi)
    return v


def require_bool(body, field, what='请求体', default=None, required=True):
    """读取布尔字段（只接受 JSON true/false）。"""
    if field not in body or body[field] is None:
        if required:
            raise _missing(field, what)
        return default
    value = body[field]
    if not isinstance(value, bool):
        raise _bad_type(field, '布尔值', what)
    return value


def optional_int(body, field, what='请求体', default=None, low=None, high=None):
    """读取整数（允许 JSON 整数或整值浮点；拒绝 bool）。"""
    if field not in body or body[field] is None:
        return default
    value = body[field]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _bad_type(field, '整数', what)
    as_float = float(value)
    if not math.isfinite(as_float) or as_float != int(as_float):
        raise _bad_type(field, '整数', what)
    iv = int(as_float)
    if low is not None and iv < low:
        raise ApiError(ErrorCode.BAD_REQUEST,
                       '%s 的 %r 不能小于 %d' % (what, field, low),
                       detail={'field': field})
    if high is not None and iv > high:
        raise ApiError(ErrorCode.BAD_REQUEST,
                       '%s 的 %r 不能大于 %d' % (what, field, high),
                       detail={'field': field})
    return iv


def query_int(query, field, default, low=None, high=None):
    """解析查询参数里的整数（如 ``?limit=200``）；非法 → 400。"""
    if query is None or field not in query:
        return default
    raw = query[field]
    if isinstance(raw, (list, tuple)):
        raw = raw[0] if raw else ''
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError):
        raise ApiError(ErrorCode.BAD_REQUEST, '查询参数 %r 必须是整数' % field,
                       detail={'field': field})
    if low is not None and value < low:
        raise ApiError(ErrorCode.BAD_REQUEST,
                       '查询参数 %r 不能小于 %d' % (field, low),
                       detail={'field': field})
    if high is not None and value > high:
        raise ApiError(ErrorCode.BAD_REQUEST,
                       '查询参数 %r 不能大于 %d' % (field, high),
                       detail={'field': field})
    return value


def require_mode(body, field='mode', what='请求体'):
    """校验会话模式：base | slam | nav。"""
    value = body.get(field)
    if not isinstance(value, str) or value not in MODES:
        raise ApiError(ErrorCode.BAD_REQUEST,
                       '%s 的 %r 必须是 %s 之一' % (what, field, '/'.join(MODES)),
                       detail={'field': field})
    return value


def validate_map_name(name):
    """校验地图名（设计文档 §3.3.2：^[A-Za-z0-9_.-]{1,40}$）。"""
    if not isinstance(name, str) or not name:
        raise ApiError(ErrorCode.BAD_REQUEST, '地图名不能为空',
                       detail={'field': 'name'})
    if not MAP_NAME_RE.match(name):
        raise ApiError(
            ErrorCode.BAD_REQUEST,
            '地图名只能包含字母/数字/下划线/点/短横线且长度 1-40：%r' % name,
            detail={'field': 'name'})
    if name in ('.', '..'):
        raise ApiError(ErrorCode.BAD_REQUEST, '地图名不能是 . 或 ..',
                       detail={'field': 'name'})
    return name


def validate_map_path(path):
    """校验导航地图 yaml 路径（绝对路径、存在、可读、.yaml/.yml）。"""
    if not isinstance(path, str) or not path:
        raise ApiError(ErrorCode.BAD_REQUEST, 'nav 模式必须提供 map 绝对路径',
                       detail={'field': 'map'})
    if not os.path.isabs(path):
        raise ApiError(ErrorCode.BAD_REQUEST,
                       'map 必须是绝对路径：%r' % path, detail={'field': 'map'})
    if not path.lower().endswith(('.yaml', '.yml')):
        raise ApiError(ErrorCode.BAD_REQUEST,
                       'map 必须是 .yaml/.yml 文件：%r' % path,
                       detail={'field': 'map'})
    if not os.path.isfile(path):
        raise ApiError(ErrorCode.BAD_REQUEST, '地图文件不存在：%s' % path,
                       detail={'field': 'map'})
    if not os.access(path, os.R_OK):
        raise ApiError(ErrorCode.BAD_REQUEST, '地图文件不可读：%s' % path,
                       detail={'field': 'map'})
    return path


def clamp_velocity(vx, vy, wz, limits):
    """把遥控速度夹到 ``state.limits``。"""
    return (clamp(vx, -limits['max_linear'], limits['max_linear']),
            clamp(vy, -limits['max_lateral'], limits['max_lateral']),
            clamp(wz, -limits['max_angular'], limits['max_angular']))


def yaw_to_quaternion(yaw):
    """平面 yaw → (x, y, z, w)，供 ROS 消息与测试共用。"""
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x, y, z, w):
    """四元数 → 平面 yaw（弧度）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    """把角度归一化到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def downsample_sequence(values, max_count, keep_last=True):
    """等间隔降采样到最多 ``max_count`` 个元素（扫描点/路径共用，§5.4）。

    ``keep_last=True`` 时强制保留最后一个元素（路径终点不应丢）。
    """
    values = list(values)
    max_count = int(max_count)
    if max_count <= 0 or len(values) <= max_count:
        return values
    step = len(values) / float(max_count)
    result = [values[min(len(values) - 1, int(i * step))]
              for i in range(max_count)]
    if keep_last and result:
        result[-1] = values[-1]
    return result
