"""地图文件纯逻辑（仅标准库）：OccupancyGrid → PNG/PGM、YAML、地图列表。

设计文档 §5.5 / §3.3.2 要求：

- PNG：8-bit 灰度；``-1→205（未知）``、``0→254（空闲）``、``100→0（占用）``，
  中间值线性；**行 0 = 世界坐标最大 y**（与 OccupancyGrid 行序相反）。
- PGM：P5，映射相同，行序同为 y 从大到小；配套 YAML：
  ``image/mode/resolution/origin/negate/occupied_thresh/free_thresh``。
- 不得依赖 numpy/Pillow/cv2：PNG 用 ``zlib`` + ``struct`` 手写。
- 地图列表解析 YAML 用安全的最小解析器（不依赖 PyYAML）；坏文件标
  ``valid=false``，不得抛到接口 500。

本模块是纯逻辑，导入期不依赖 rclpy，可离线单元测试。
"""

from __future__ import annotations

import hashlib
import os
import re
import struct
import zlib
from dataclasses import dataclass, field

#: PGM/PNG 灰度映射常量（设计文档 §5.5）
GRAY_UNKNOWN = 205
GRAY_FREE = 254
GRAY_OCCUPIED = 0

#: 地图名规则（与 protocol 保持一致；独立定义避免相互依赖）
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_.-]{1,40}$')

#: 预览图 URL 前缀（设计文档 §3.3.1）
PREVIEW_URL_TEMPLATE = '/api/maps/%s/preview.png'


class MapIOError(Exception):
    """地图文件读取/解析/写入失败（纯逻辑层异常）。"""


class MapExistsError(MapIOError):
    """同名地图已存在且未允许覆盖。"""


# ---------------------------------------------------------------------------
# 灰度映射
# ---------------------------------------------------------------------------

def occupancy_gray(value):
    """OccupancyGrid 单元值 → 8-bit 灰度。

    ``-1``（未知）→ 205；``0``（空闲）→ 254；``100``（占用）→ 0；
    0..100 之间线性；越界值按端点处理。
    """
    v = int(value)
    if v < 0:
        return GRAY_UNKNOWN
    if v >= 100:
        return GRAY_OCCUPIED
    return int(round(254.0 * (1.0 - v / 100.0)))


# ---------------------------------------------------------------------------
# PNG（8-bit 灰度，zlib/struct 手写）
# ---------------------------------------------------------------------------

_PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def _png_chunk(tag, data):
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def encode_png_gray8(width, height, rows):
    """把灰度行（每行 ``width`` 字节、行 0 = 图像顶部）编码为 PNG bytes。"""
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise MapIOError('PNG 尺寸非法: %dx%d' % (width, height))
    rows = list(rows)
    if len(rows) != height:
        raise MapIOError('PNG 行数 %d 与 height %d 不符' % (len(rows), height))
    raw = bytearray()
    for row in rows:
        row = bytes(row)
        if len(row) != width:
            raise MapIOError('PNG 行宽 %d 与 width %d 不符' % (len(row), width))
        raw.append(0)  # filter type 0 (None)
        raw.extend(row)
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return b''.join((
        _PNG_SIGNATURE,
        _png_chunk(b'IHDR', ihdr),
        _png_chunk(b'IDAT', zlib.compress(bytes(raw), 6)),
        _png_chunk(b'IEND', b''),
    ))


def decode_png_gray8(data):
    """解析本模块产生的 8-bit 灰度 PNG（filter 0），返回 (w, h, rows)。

    只支持自己生成的格式（灰度、无隔行、filter=0）；遇到其它格式抛
    ``MapIOError``。主要供离线测试与 HTTP 层自检使用。
    """
    data = bytes(data)
    if not data.startswith(_PNG_SIGNATURE):
        raise MapIOError('不是 PNG 文件（签名不符）')
    pos = len(_PNG_SIGNATURE)
    width = height = None
    idat = bytearray()
    while pos + 8 <= len(data):
        length = struct.unpack('>I', data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        if len(chunk) != length:
            raise MapIOError('PNG chunk 截断: %r' % tag)
        pos += 12 + length
        if tag == b'IHDR':
            width, height, depth, color, comp, filt, interlace = struct.unpack(
                '>IIBBBBB', chunk)
            if depth != 8 or color != 0 or interlace != 0:
                raise MapIOError(
                    '只支持 8-bit 灰度无隔行 PNG：depth=%d color=%d interlace=%d'
                    % (depth, color, interlace))
        elif tag == b'IDAT':
            idat.extend(chunk)
        elif tag == b'IEND':
            break
    if width is None or height is None:
        raise MapIOError('PNG 缺少 IHDR')
    try:
        raw = zlib.decompress(bytes(idat))
    except zlib.error as exc:
        raise MapIOError('PNG IDAT 解压失败: %s' % exc)
    stride = width + 1
    if len(raw) != stride * height:
        raise MapIOError('PNG 像素数据长度不符')
    rows = []
    for y in range(height):
        base = y * stride
        if raw[base] != 0:
            raise MapIOError('只支持 filter=0 的 PNG（行 %d filter=%d）'
                             % (y, raw[base]))
        rows.append(raw[base + 1:base + 1 + width])
    return width, height, rows


# ---------------------------------------------------------------------------
# PGM（P5）
# ---------------------------------------------------------------------------

def encode_pgm_p5(width, height, rows, maxval=255):
    """把灰度行编码为 P5 PGM（行 0 = 世界坐标最大 y）。"""
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise MapIOError('PGM 尺寸非法: %dx%d' % (width, height))
    rows = [bytes(r) for r in rows]
    if len(rows) != height or any(len(r) != width for r in rows):
        raise MapIOError('PGM 行数据与尺寸不符')
    header = b'P5\n%d %d\n%d\n' % (width, height, int(maxval))
    return header + b''.join(rows)


def _pgm_header_tokens(data, count):
    """读取 PGM 头部前 count 个 token（跳过注释与空白），返回 (tokens, pos)。"""
    data = bytes(data)
    pos = 0
    n = len(data)
    tokens = []
    while len(tokens) < count:
        while pos < n and data[pos:pos + 1] in (b' ', b'\t', b'\r', b'\n'):
            pos += 1
        if pos >= n:
            raise MapIOError('PGM 头部不完整')
        if data[pos:pos + 1] == b'#':
            while pos < n and data[pos:pos + 1] not in (b'\r', b'\n'):
                pos += 1
            continue
        start = pos
        while pos < n and data[pos:pos + 1] not in (b' ', b'\t', b'\r', b'\n'):
            pos += 1
        tokens.append(data[start:pos])
    return tokens, pos


@dataclass(frozen=True)
class PgmImage:
    width: int
    height: int
    maxval: int
    rows: tuple  # tuple[bytes, ...]，行 0 = 图像顶部（世界最大 y）
    data: bytes = b''  # 原始灰度像素（连续、行 0 在前的顺序）

    @property
    def gray_rows(self):
        return list(self.rows)


def parse_pgm(data):
    """解析 P5 PGM；支持注释与 ``maxval<=255``。返回 :class:`PgmImage`。"""
    tokens, pos = _pgm_header_tokens(data, 4)
    if tokens[0] != b'P5':
        raise MapIOError('只支持 P5（binary）PGM，实际 %r' % tokens[0])
    try:
        width = int(tokens[1])
        height = int(tokens[2])
        maxval = int(tokens[3])
    except ValueError:
        raise MapIOError('PGM 头部尺寸/最大值不是整数')
    if width <= 0 or height <= 0 or not (0 < maxval <= 255):
        raise MapIOError('PGM 头部数值非法: %dx%d maxval=%d'
                         % (width, height, maxval))
    # 头部最后一个 token 之后必须恰好有一个空白字符，然后是像素数据
    n = len(data)
    if pos >= n or data[pos:pos + 1] not in (b' ', b'\t', b'\r', b'\n'):
        raise MapIOError('PGM 头部与像素数据之间缺少分隔符')
    raster_start = pos + 1
    if data[pos:pos + 2] == b'\r\n':
        raster_start = pos + 2
    raster = bytes(data[raster_start:])
    if len(raster) < width * height:
        raise MapIOError('PGM 像素数据不足：需要 %d，实际 %d'
                         % (width * height, len(raster)))
    raster = raster[:width * height]
    rows = tuple(raster[y * width:(y + 1) * width] for y in range(height))
    return PgmImage(width=width, height=height, maxval=maxval, rows=rows,
                    data=raster)


def pgm_dimensions(data):
    """只解析 PGM 头部并校验数据长度，返回 (width, height, maxval)。"""
    img = parse_pgm(data)
    return img.width, img.height, img.maxval


def scale_to_255(value, maxval):
    """把 maxval 灰度缩放为 0..255（PGM 非 255 时的兜底）。"""
    if maxval == 255:
        return int(value)
    return int(round(int(value) * 255.0 / float(maxval)))


def pgm_to_png(data):
    """P5 PGM → 8-bit 灰度 PNG bytes（供 ``/api/maps/<name>/preview.png``）。"""
    img = parse_pgm(data)
    if img.maxval == 255:
        rows = img.rows
    else:
        rows = [bytes(scale_to_255(v, img.maxval) for v in row)
                for row in img.rows]
    return encode_png_gray8(img.width, img.height, rows)


def read_pgm_file(path):
    with open(path, 'rb') as fh:
        return fh.read()


def pgm_file_to_png(path):
    """读取 PGM 文件并转 PNG；文件不存在/损坏 → MapIOError。"""
    try:
        raw = read_pgm_file(path)
    except OSError as exc:
        raise MapIOError('读取 PGM 失败: %s' % exc)
    return pgm_to_png(raw)


# ---------------------------------------------------------------------------
# OccupancyGrid → MapGrid / PNG / PGM
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MapGrid:
    """与 ROS 解耦的占据栅格快照（值域 -1..100，行 0 = 最小 y）。"""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    stamp: float
    data: tuple

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise MapIOError('地图尺寸非法: %dx%d' % (self.width, self.height))
        if len(self.data) != self.width * self.height:
            raise MapIOError('地图数据长度 %d 与 %dx%d 不符'
                             % (len(self.data), self.width, self.height))
        if not (self.resolution > 0.0):
            raise MapIOError('地图分辨率必须为正数: %r' % (self.resolution,))

    # -- 元数据 ---------------------------------------------------------
    def origin(self):
        return (float(self.origin_x), float(self.origin_y),
                float(self.origin_yaw))

    def meta(self, version):
        """§3.1 的 ``map`` 对象（available=true 分支）。"""
        return {
            'available': True,
            'version': int(version),
            'width': int(self.width),
            'height': int(self.height),
            'resolution': float(self.resolution),
            'origin': {'x': float(self.origin_x),
                       'y': float(self.origin_y),
                       'yaw': float(self.origin_yaw)},
            'frame_id': str(self.frame_id),
            'stamp': float(self.stamp),
        }

    def content_hash(self):
        """内容 hash：尺寸/分辨率/原点/帧 + 栅格数据；用于判断 version 是否递增。"""
        digest = hashlib.sha1()
        digest.update(struct.pack('<II', int(self.width), int(self.height)))
        digest.update(struct.pack('<dddd', float(self.resolution),
                                  float(self.origin_x), float(self.origin_y),
                                  float(self.origin_yaw)))
        digest.update(str(self.frame_id).encode('utf-8'))
        digest.update(bytes((int(v) & 0xFF) for v in self.data))
        return digest.hexdigest()

    # -- 转换 -----------------------------------------------------------
    def gray_rows(self):
        """PNG/PGM 行序：行 0 = 世界坐标最大 y（设计文档 §5.5）。"""
        rows = []
        for y in range(self.height - 1, -1, -1):
            base = y * self.width
            rows.append(bytes(occupancy_gray(self.data[base + i])
                              for i in range(self.width)))
        return rows

    def to_png(self):
        return encode_png_gray8(self.width, self.height, self.gray_rows())

    def to_pgm(self):
        return encode_pgm_p5(self.width, self.height, self.gray_rows())


def make_map_grid(width, height, resolution, origin, data, frame_id='map',
                  stamp=0.0):
    """便捷构造 :class:`MapGrid`（origin 为 ``(x, y, yaw)``）。"""
    ox, oy, oyaw = (float(v) for v in origin)
    return MapGrid(width=int(width), height=int(height),
                   resolution=float(resolution), origin_x=ox, origin_y=oy,
                   origin_yaw=oyaw, frame_id=str(frame_id),
                   stamp=float(stamp), data=tuple(int(v) for v in data))


# ---------------------------------------------------------------------------
# 最小 YAML 解析（地图 YAML 只用简单 ``key: scalar`` / 内联列表）
# ---------------------------------------------------------------------------

def _strip_comment(line):
    """移除引号之外的 ``#`` 注释。"""
    quote = None
    out = []
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
        elif ch in ('"', "'"):
            quote = ch
            out.append(ch)
        elif ch == '#':
            break
        else:
            out.append(ch)
    return ''.join(out).rstrip()


def _parse_scalar(text):
    text = text.strip()
    if not text:
        return ''
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ('"', "'"):
        return text[1:-1]
    low = text.lower()
    if low in ('true', 'false'):
        return low == 'true'
    if low in ('null', '~'):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def parse_minimal_yaml(text):
    """解析简单 YAML：``key: value`` 与 ``key: [a, b, c]``。

    不支持嵌套映射/锚点/多行字符串——地图 YAML 与网关配置文件都不需要。
    解析失败的行直接忽略（地图条目随后按字段缺失标 ``valid=false``）。
    """
    result = {}
    for raw_line in str(text).splitlines():
        line = _strip_comment(raw_line)
        if not line.strip():
            continue
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        key = key.strip().strip('"\'')
        if not key:
            continue
        value = value.strip()
        if value.startswith('[') and value.endswith(']'):
            inner = value[1:-1].strip()
            if not inner:
                result[key] = []
            else:
                result[key] = [_parse_scalar(part)
                               for part in inner.split(',')]
        else:
            result[key] = _parse_scalar(value)
    return result


def _fmt_number(value):
    """YAML 数值格式：``0.05`` / ``-2.4`` / ``0.0``（保留 float 语义）。"""
    return repr(float(value))


def format_map_yaml(image_name, resolution, origin):
    """按设计文档 §3.3.2 生成地图 YAML 文本。"""
    ox, oy, oyaw = (float(v) for v in origin)
    lines = [
        'image: %s' % image_name,
        'mode: trinary',
        'resolution: %s' % _fmt_number(resolution),
        'origin: [%s, %s, %s]' % (_fmt_number(ox), _fmt_number(oy),
                                  _fmt_number(oyaw)),
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
    ]
    return '\n'.join(lines) + '\n'


@dataclass
class MapYaml:
    """解析后的地图 YAML（坏文件也可返回，valid=False）。"""

    path: str
    name: str
    image: str = ''
    resolution: float = 0.0
    origin: tuple = (0.0, 0.0, 0.0)
    mode: str = 'trinary'
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
    valid: bool = False
    error: str = ''
    raw: dict = field(default_factory=dict)

    @property
    def pgm_path(self):
        if not self.image:
            return ''
        if os.path.isabs(self.image):
            return self.image
        return os.path.join(os.path.dirname(os.path.abspath(self.path)),
                            self.image)


def map_name_valid(name):
    """地图名是否满足 ``^[A-Za-z0-9_.-]{1,40}$``。"""
    return isinstance(name, str) and bool(MAP_NAME_RE.match(name)) \
        and name not in ('.', '..')


def load_map_yaml(path):
    """读取并校验地图 YAML；坏文件返回 ``valid=False`` 的 :class:`MapYaml`。"""
    path = os.path.abspath(str(path))
    name = os.path.splitext(os.path.basename(path))[0]
    result = MapYaml(path=path, name=name)
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as fh:
            text = fh.read()
    except OSError as exc:
        result.error = '读取失败: %s' % exc
        return result
    try:
        raw = parse_minimal_yaml(text)
    except Exception as exc:  # pragma: no cover - 解析器防御
        result.error = 'YAML 解析异常: %s' % exc
        return result
    result.raw = raw
    errors = []

    image = raw.get('image')
    if not isinstance(image, str) or not image.strip():
        errors.append('缺少 image')
    else:
        result.image = image.strip()

    resolution = raw.get('resolution')
    if not isinstance(resolution, (int, float)) or isinstance(resolution, bool) \
            or not (float(resolution) > 0.0):
        errors.append('resolution 非法')
    else:
        result.resolution = float(resolution)

    origin = raw.get('origin')
    if isinstance(origin, (list, tuple)) and len(origin) == 3 \
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    for v in origin):
        result.origin = tuple(float(v) for v in origin)
    else:
        errors.append('origin 非法')

    mode = raw.get('mode')
    if isinstance(mode, str) and mode:
        result.mode = mode
    negate = raw.get('negate')
    if isinstance(negate, bool):
        result.negate = 1 if negate else 0
    elif isinstance(negate, int):
        result.negate = negate
    for key in ('occupied_thresh', 'free_thresh'):
        value = raw.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            setattr(result, key, float(value))

    if errors:
        result.error = '；'.join(errors)
        return result
    result.valid = True
    return result


# ---------------------------------------------------------------------------
# 地图列表 / 保存 / 预览
# ---------------------------------------------------------------------------

def scan_map_entry(yaml_path):
    """把一个 yaml 文件扫描成 §3.3.1 的地图条目（坏文件 valid=false）。"""
    info = load_map_yaml(yaml_path)
    entry = {
        'name': info.name,
        'path': info.path,
        'yaml': info.path,
        'pgm': info.pgm_path,
        'mtime': _safe_mtime(info.path),
        'resolution': info.resolution if info.valid else None,
        'width': None,
        'height': None,
        'preview_url': PREVIEW_URL_TEMPLATE % info.name,
        'valid': False,
    }
    if not info.valid:
        return entry
    pgm_path = info.pgm_path
    if not pgm_path or not os.path.isfile(pgm_path):
        return entry
    try:
        width, height, _maxval = pgm_dimensions(read_pgm_file(pgm_path))
    except (MapIOError, OSError):
        return entry
    entry['width'] = width
    entry['height'] = height
    entry['valid'] = True
    return entry


def _safe_mtime(path):
    try:
        return float(os.path.getmtime(path))
    except OSError:
        return 0.0


def list_maps(dirs):
    """按目录优先级扫描地图（同名时先出现的目录优先，§3.3.1）。"""
    result = []
    seen = set()
    for directory in dirs:
        if not directory:
            continue
        directory = os.path.abspath(os.path.expanduser(str(directory)))
        if not os.path.isdir(directory):
            continue
        candidates = []
        for ext in ('*.yaml', '*.yml'):
            try:
                import glob
                candidates.extend(glob.glob(os.path.join(directory, ext)))
            except OSError:
                continue
        for yaml_path in sorted(candidates):
            name = os.path.splitext(os.path.basename(yaml_path))[0]
            if name in seen:
                continue
            seen.add(name)
            try:
                result.append(scan_map_entry(yaml_path))
            except Exception as exc:  # pragma: no cover - 防御
                result.append({
                    'name': name, 'path': os.path.abspath(yaml_path),
                    'yaml': os.path.abspath(yaml_path), 'pgm': '',
                    'mtime': _safe_mtime(yaml_path), 'resolution': None,
                    'width': None, 'height': None,
                    'preview_url': PREVIEW_URL_TEMPLATE % name,
                    'valid': False, 'error': str(exc),
                })
    return result


def existing_map_dirs(dirs):
    """返回实际存在的扫描目录（用于 ``GET /api/maps`` 的 ``dirs`` 字段）。"""
    out = []
    for directory in dirs:
        if not directory:
            continue
        absolute = os.path.abspath(os.path.expanduser(str(directory)))
        if os.path.isdir(absolute) and absolute not in out:
            out.append(absolute)
    return out


def choose_save_dir(primary, fallback):
    """保存目录优先级：primary（存在且可写）→ fallback（自动创建）。"""
    primary = os.path.abspath(os.path.expanduser(str(primary)))
    if os.path.isdir(primary) and os.access(primary, os.W_OK):
        return primary
    fallback = os.path.abspath(os.path.expanduser(str(fallback)))
    os.makedirs(fallback, exist_ok=True)
    return fallback


def _atomic_write(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'wb') as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def save_map(directory, name, grid, overwrite=False):
    """把 :class:`MapGrid` 落盘为 ``name.pgm`` + ``name.yaml``。

    返回 §3.3.2 的响应字段（``ok/name/yaml/pgm/width/height/resolution/mtime``）。
    同名存在且 ``overwrite=False`` → :class:`MapExistsError`。
    """
    if not map_name_valid(name):
        raise MapIOError('非法地图名: %r' % (name,))
    directory = os.path.abspath(os.path.expanduser(str(directory)))
    os.makedirs(directory, exist_ok=True)
    pgm_path = os.path.join(directory, '%s.pgm' % name)
    yaml_path = os.path.join(directory, '%s.yaml' % name)
    if not overwrite and (os.path.exists(pgm_path) or os.path.exists(yaml_path)):
        raise MapExistsError('地图已存在: %s' % os.path.join(directory, name))
    yaml_text = format_map_yaml('%s.pgm' % name, grid.resolution, grid.origin())
    _atomic_write(pgm_path, grid.to_pgm())
    _atomic_write(yaml_path, yaml_text.encode('utf-8'))
    return {
        'ok': True,
        'name': name,
        'yaml': yaml_path,
        'pgm': pgm_path,
        'width': int(grid.width),
        'height': int(grid.height),
        'resolution': float(grid.resolution),
        'mtime': _safe_mtime(yaml_path),
    }


def load_pgm_file(path):
    """读取 PGM 文件（预览/列表校验共用）。"""
    try:
        with open(path, 'rb') as fh:
            return fh.read()
    except OSError as exc:
        raise MapIOError('读取 PGM 失败: %s' % exc)
