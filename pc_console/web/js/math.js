/**
 * math.js —— 纯函数工具库（无 DOM / 无浏览器 API 依赖）。
 *
 * 负责：
 *   - 世界坐标（米，ROS map 坐标系，x 向右、y 向上、yaw 逆时针）与屏幕坐标
 *     （CSS 像素，x 向右、y 向下）的双向换算与 y 轴翻转；
 *   - 地图 origin/resolution/yaw 与图片像素、OccupancyGrid 栅格的换算；
 *   - 缩放/平移/适配视图等视图变换；
 *   - 键盘与虚拟摇杆 → 麦轮速度指令、限幅；
 *   - 网格/比例尺选择与数值格式化。
 *
 * 约定：
 *   view = { scale: 像素/米, centerX, centerY: 视图中心的世界坐标,
 *            width, height: 画布 CSS 像素尺寸 }
 *   meta = §3.1 的 map 对象（available/version/width/height/resolution/origin{...}）
 *
 * 该模块可被 Node 直接 import（node --test，Node ≥24）。
 */

export const EPSILON = 1e-9;

/* ------------------------------------------------------------------------- */
/* 基础数值工具                                                              */
/* ------------------------------------------------------------------------- */

/** 数值限幅；非有限数返回 min。 */
export function clamp(value, min, max) {
  if (!Number.isFinite(value)) return min;
  return Math.min(Math.max(value, min), max);
}

/** 0..1 限幅。 */
export function clamp01(value) {
  return clamp(value, 0, 1);
}

export function isFiniteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

/** 宽松数值解析：非法/缺失时返回 fallback。 */
export function toFiniteNumber(value, fallback = 0) {
  if (typeof value === 'number') return Number.isFinite(value) ? value : fallback;
  if (typeof value === 'string' && value.trim() !== '') {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  }
  return fallback;
}

export function lerp(a, b, t) {
  const k = clamp(t, 0, 1);
  return a + (b - a) * k;
}

/** 角度归一化到 (-π, π]。 */
export function normalizeAngle(rad) {
  if (!Number.isFinite(rad)) return 0;
  const twoPi = Math.PI * 2;
  let angle = rad % twoPi;
  if (angle <= -Math.PI) angle += twoPi;
  if (angle > Math.PI) angle -= twoPi;
  return angle;
}

/** 世界 yaw → canvas 旋转角（屏幕 y 轴向下，故取负）。 */
export function yawToScreenAngle(yaw) {
  return -normalizeAngle(toFiniteNumber(yaw, 0));
}

/** 二维向量按 yaw 旋转（世界坐标系，逆时针为正）。 */
export function rotateVector(dx, dy, yaw) {
  const c = Math.cos(toFiniteNumber(yaw, 0));
  const s = Math.sin(toFiniteNumber(yaw, 0));
  return { x: dx * c - dy * s, y: dx * s + dy * c };
}

export function distance(x1, y1, x2, y2) {
  const dx = toFiniteNumber(x2) - toFiniteNumber(x1);
  const dy = toFiniteNumber(y2) - toFiniteNumber(y1);
  return Math.hypot(dx, dy);
}

/* ------------------------------------------------------------------------- */
/* 视图变换                                                                  */
/* ------------------------------------------------------------------------- */

/** 构造视图，缺省/非法值会被补全。 */
export function makeView(options = {}) {
  return {
    scale: clamp(toFiniteNumber(options.scale, 40), 1, 10000),
    centerX: toFiniteNumber(options.centerX, 0),
    centerY: toFiniteNumber(options.centerY, 0),
    width: Math.max(1, toFiniteNumber(options.width, 800)),
    height: Math.max(1, toFiniteNumber(options.height, 500)),
  };
}

/** 世界坐标 → 屏幕 CSS 像素（y 翻转：世界 y 向上，屏幕 y 向下）。 */
export function worldToScreen(view, wx, wy) {
  const scale = isFiniteNumber(view.scale) && view.scale > 0 ? view.scale : 40;
  const cx = toFiniteNumber(view.centerX, 0);
  const cy = toFiniteNumber(view.centerY, 0);
  const width = toFiniteNumber(view.width, 0);
  const height = toFiniteNumber(view.height, 0);
  return {
    x: width / 2 + (toFiniteNumber(wx, 0) - cx) * scale,
    y: height / 2 - (toFiniteNumber(wy, 0) - cy) * scale,
  };
}

/** 屏幕 CSS 像素 → 世界坐标（worldToScreen 的逆变换）。 */
export function screenToWorld(view, sx, sy) {
  const scale = isFiniteNumber(view.scale) && view.scale > 0 ? view.scale : 40;
  const cx = toFiniteNumber(view.centerX, 0);
  const cy = toFiniteNumber(view.centerY, 0);
  const width = toFiniteNumber(view.width, 0);
  const height = toFiniteNumber(view.height, 0);
  return {
    x: cx + (toFiniteNumber(sx, 0) - width / 2) / scale,
    y: cy - (toFiniteNumber(sy, 0) - height / 2) / scale,
  };
}

/**
 * 以屏幕点 (sx, sy) 为不动点缩放：缩放前后该屏幕点对应的世界坐标不变。
 * 用于滚轮“以鼠标为中心缩放”。
 */
export function zoomAt(view, factor, sx, sy, minScale = 4, maxScale = 400) {
  const safeFactor = Number.isFinite(factor) && factor > 0 ? factor : 1;
  const anchor = screenToWorld(view, sx, sy);
  const scale = clamp(view.scale * safeFactor, minScale, maxScale);
  const width = toFiniteNumber(view.width, 0);
  const height = toFiniteNumber(view.height, 0);
  const centerX = anchor.x - (toFiniteNumber(sx, 0) - width / 2) / scale;
  const centerY = anchor.y + (toFiniteNumber(sy, 0) - height / 2) / scale;
  return { ...view, scale, centerX, centerY };
}

/** 拖拽平移：屏幕位移 (dx, dy) 对应的视图中心变化。 */
export function panView(view, dx, dy) {
  const scale = isFiniteNumber(view.scale) && view.scale > 0 ? view.scale : 40;
  return {
    ...view,
    centerX: toFiniteNumber(view.centerX, 0) - toFiniteNumber(dx, 0) / scale,
    centerY: toFiniteNumber(view.centerY, 0) + toFiniteNumber(dy, 0) / scale,
  };
}

/** 把视图中心移到世界坐标 (wx, wy)。 */
export function centerOn(view, wx, wy) {
  return {
    ...view,
    centerX: toFiniteNumber(wx, 0),
    centerY: toFiniteNumber(wy, 0),
  };
}

/* ------------------------------------------------------------------------- */
/* 地图元数据换算                                                            */
/* ------------------------------------------------------------------------- */

/** 地图是否可用（有 width/height/resolution 且 >0）。 */
export function mapMetaUsable(meta) {
  return Boolean(
    meta &&
      isFiniteNumber(meta.width) &&
      isFiniteNumber(meta.height) &&
      meta.width > 0 &&
      meta.height > 0 &&
      isFiniteNumber(meta.resolution) &&
      meta.resolution > 0,
  );
}

/** 读取地图 origin（缺省 0,0,0）。 */
export function mapOrigin(meta) {
  const origin = (meta && meta.origin) || {};
  return {
    x: toFiniteNumber(origin.x, 0),
    y: toFiniteNumber(origin.y, 0),
    yaw: toFiniteNumber(origin.yaw, 0),
  };
}

/**
 * 图片像素坐标 (px, py) → 世界坐标。
 * 图片约定（与 §5.5 一致）：像素 (0,0) 为图片左上角，像素 y 向下增长，
 * 即图片第 0 行对应世界坐标最大 y；origin 为图片左下角的世界坐标（yaw=0 时）。
 * px/py 允许小数（像素中心 = +0.5）。
 */
export function imagePixelToWorld(meta, px, py) {
  const origin = mapOrigin(meta);
  const resolution = toFiniteNumber(meta && meta.resolution, 0.05);
  const height = toFiniteNumber(meta && meta.height, 0);
  const localX = toFiniteNumber(px, 0) * resolution;
  const localY = (height - toFiniteNumber(py, 0)) * resolution;
  const rotated = rotateVector(localX, localY, origin.yaw);
  return { x: origin.x + rotated.x, y: origin.y + rotated.y };
}

/** 世界坐标 → 图片像素坐标（imagePixelToWorld 的逆变换）。 */
export function worldToImagePixel(meta, wx, wy) {
  const origin = mapOrigin(meta);
  const resolution = toFiniteNumber(meta && meta.resolution, 0.05);
  const height = toFiniteNumber(meta && meta.height, 0);
  const local = rotateVector(
    toFiniteNumber(wx, 0) - origin.x,
    toFiniteNumber(wy, 0) - origin.y,
    -origin.yaw,
  );
  return {
    px: local.x / (resolution || 0.05),
    py: height - local.y / (resolution || 0.05),
  };
}

/**
 * OccupancyGrid 栅格 (col, row) → 单元格中心的世界坐标。
 * 注意与图片行序相反：OccupancyGrid 的 row 0 对应世界最小 y（图片最下一行）。
 */
export function gridCellToWorld(meta, col, row) {
  const origin = mapOrigin(meta);
  const resolution = toFiniteNumber(meta && meta.resolution, 0.05);
  const local = rotateVector(
    (toFiniteNumber(col, 0) + 0.5) * resolution,
    (toFiniteNumber(row, 0) + 0.5) * resolution,
    origin.yaw,
  );
  return { x: origin.x + local.x, y: origin.y + local.y };
}

/** 世界坐标 → OccupancyGrid 栅格 (col, row)（向下取整）。 */
export function worldToGridCell(meta, wx, wy) {
  const origin = mapOrigin(meta);
  const resolution = toFiniteNumber(meta && meta.resolution, 0.05);
  const local = rotateVector(
    toFiniteNumber(wx, 0) - origin.x,
    toFiniteNumber(wy, 0) - origin.y,
    -origin.yaw,
  );
  return {
    col: Math.floor(local.x / (resolution || 0.05)),
    row: Math.floor(local.y / (resolution || 0.05)),
  };
}

/** 地图四角世界坐标（考虑 origin.yaw）。 */
export function mapWorldCorners(meta) {
  const width = toFiniteNumber(meta && meta.width, 0);
  const height = toFiniteNumber(meta && meta.height, 0);
  return [
    imagePixelToWorld(meta, 0, 0),
    imagePixelToWorld(meta, width, 0),
    imagePixelToWorld(meta, width, height),
    imagePixelToWorld(meta, 0, height),
  ];
}

/** 地图世界包围盒 {minX, minY, maxX, maxY}。 */
export function mapWorldBounds(meta) {
  const corners = mapWorldCorners(meta);
  const xs = corners.map((point) => point.x);
  const ys = corners.map((point) => point.y);
  return {
    minX: Math.min(...xs),
    minY: Math.min(...ys),
    maxX: Math.max(...xs),
    maxY: Math.max(...ys),
  };
}

/** 让整张地图适配到画布（留 padding），返回新 view。 */
export function fitView(meta, width, height, padding = 24) {
  const viewWidth = Math.max(1, toFiniteNumber(width, 800));
  const viewHeight = Math.max(1, toFiniteNumber(height, 500));
  if (!mapMetaUsable(meta)) {
    return makeView({ scale: 40, centerX: 0, centerY: 0, width: viewWidth, height: viewHeight });
  }
  const bounds = mapWorldBounds(meta);
  const worldWidth = Math.max(bounds.maxX - bounds.minX, EPSILON);
  const worldHeight = Math.max(bounds.maxY - bounds.minY, EPSILON);
  const pad = Math.max(0, toFiniteNumber(padding, 0));
  const availWidth = Math.max(32, viewWidth - pad * 2);
  const availHeight = Math.max(32, viewHeight - pad * 2);
  const scale = clamp(Math.min(availWidth / worldWidth, availHeight / worldHeight), 1, 800);
  return makeView({
    scale,
    centerX: (bounds.minX + bounds.maxX) / 2,
    centerY: (bounds.minY + bounds.maxY) / 2,
    width: viewWidth,
    height: viewHeight,
  });
}

/* ------------------------------------------------------------------------- */
/* 遥控输入                                                                  */
/* ------------------------------------------------------------------------- */

const KEY_ALIASES = {
  arrowup: 'w',
  arrowdown: 's',
  arrowleft: 'a',
  arrowright: 'd',
};

const KEY_ALIASES_REVERSE = {
  w: ['arrowup'],
  s: ['arrowdown'],
  a: ['arrowleft'],
  d: ['arrowright'],
};

function hasKey(keys, name) {
  if (!keys) return false;
  const key = KEY_ALIASES[name] || name;
  if (typeof keys.has === 'function') {
    if (keys.has(key) || keys.has(name)) return true;
    return (KEY_ALIASES_REVERSE[name] || []).some((alias) => keys.has(alias));
  }
  if (Array.isArray(keys)) {
    return keys.includes(key) || keys.includes(name) ||
      (KEY_ALIASES_REVERSE[name] || []).some((alias) => keys.includes(alias));
  }
  return Boolean(keys[key] || keys[name]);
}

/** 计算遥控速度上限的公共逻辑：滑杆比例 × Shift 加速，且绝不超过 limits。 */
function teleopLimits(options = {}) {
  const maxLinear = Math.max(0, toFiniteNumber(options.maxLinear, 0.3));
  const maxLateral = Math.max(0, toFiniteNumber(options.maxLateral, maxLinear));
  const maxAngular = Math.max(0, toFiniteNumber(options.maxAngular, 1.5));
  const scale = clamp(toFiniteNumber(options.scale, 1), 0, 1);
  const boostFactor = Math.max(1, toFiniteNumber(options.boostFactor, 1.6));
  const boost = options.boost === true;
  return {
    linear: Math.min(maxLinear, maxLinear * scale * (boost ? boostFactor : 1)),
    lateral: Math.min(maxLateral, maxLateral * scale * (boost ? boostFactor : 1)),
    angular: Math.min(maxAngular, maxAngular * scale * (boost ? boostFactor : 1)),
    maxLinear,
    maxLateral,
    maxAngular,
  };
}

/**
 * 键盘按键集合 → 麦轮速度指令。
 *
 * 语义（与 §6.2 一致）：
 *   W/S 前后（vx ±），A/D 左右横移（A=+y 左，D=-y 右），
 *   Q/E 自转（Q=+wz 逆时针，E=-wz 顺时针），Shift 加速（不超过 limits）。
 *
 * @param {Set<string>|string[]|Record<string, boolean>} keys 小写按键集合
 * @param {object} options {maxLinear,maxLateral,maxAngular,scale,boost,boostFactor}
 * @returns {{vx:number,vy:number,wz:number}}
 */
export function keyVelocity(keys, options = {}) {
  const boost = options.boost === true || hasKey(keys, 'shift');
  const limit = teleopLimits({ ...options, boost });
  const forward = (hasKey(keys, 'w') ? 1 : 0) - (hasKey(keys, 's') ? 1 : 0);
  const strafe = (hasKey(keys, 'a') ? 1 : 0) - (hasKey(keys, 'd') ? 1 : 0);
  const turn = (hasKey(keys, 'q') ? 1 : 0) - (hasKey(keys, 'e') ? 1 : 0);
  return {
    vx: clamp(forward * limit.linear, -limit.maxLinear, limit.maxLinear),
    vy: clamp(strafe * limit.lateral, -limit.maxLateral, limit.maxLateral),
    wz: clamp(turn * limit.angular, -limit.maxAngular, limit.maxAngular),
  };
}

/**
 * 摇杆归一化：死区内归零，死区外线性映射到 0..1 模长。
 * @returns {{x:number,y:number,magnitude:number}}
 */
export function normalizeInputVector(jx, jy, deadzone = 0.08) {
  const x = clamp(toFiniteNumber(jx, 0), -1, 1);
  const y = clamp(toFiniteNumber(jy, 0), -1, 1);
  const magnitude = Math.hypot(x, y);
  const dz = clamp(toFiniteNumber(deadzone, 0), 0, 0.9);
  if (magnitude <= dz || magnitude < EPSILON) return { x: 0, y: 0, magnitude: 0 };
  const scaled = clamp((magnitude - dz) / (1 - dz), 0, 1);
  return { x: (x / magnitude) * scaled, y: (y / magnitude) * scaled, magnitude: scaled };
}

/**
 * 虚拟摇杆 → 麦轮速度指令。
 * 屏幕坐标：jx=+1 向右，jy=+1 向下；上推 = 前进，右推 = 右平移。
 */
export function joystickVelocity(jx, jy, options = {}) {
  const deadzone = toFiniteNumber(options.deadzone, 0.08);
  const input = normalizeInputVector(jx, jy, deadzone);
  const limit = teleopLimits(options);
  return {
    vx: clamp(-input.y * limit.linear, -limit.maxLinear, limit.maxLinear) + 0,
    vy: clamp(-input.x * limit.lateral, -limit.maxLateral, limit.maxLateral) + 0,
    wz: 0,
    magnitude: input.magnitude,
  };
}

/** 速度指令按 limits 二次限幅（防御非法/超限输入）。 */
export function clampVelocity(velocity, limits = {}) {
  const maxLinear = Math.max(0, toFiniteNumber(limits.maxLinear, 0.3));
  const maxLateral = Math.max(0, toFiniteNumber(limits.maxLateral, maxLinear));
  const maxAngular = Math.max(0, toFiniteNumber(limits.maxAngular, 1.5));
  return {
    vx: clamp(toFiniteNumber(velocity && velocity.vx, 0), -maxLinear, maxLinear),
    vy: clamp(toFiniteNumber(velocity && velocity.vy, 0), -maxLateral, maxLateral),
    wz: clamp(toFiniteNumber(velocity && velocity.wz, 0), -maxAngular, maxAngular),
  };
}

/* ------------------------------------------------------------------------- */
/* 网格与比例尺                                                              */
/* ------------------------------------------------------------------------- */

/** 选一个“好看”的网格间距（米）：目标像素间距 targetPx 附近取 1/2/5×10^n。 */
export function chooseGridSpacing(scale, targetPx = 90) {
  const safeScale = isFiniteNumber(scale) && scale > 0 ? scale : 40;
  const raw = Math.max(toFiniteNumber(targetPx, 90), 1) / safeScale;
  const power = Math.pow(10, Math.floor(Math.log10(Math.max(raw, EPSILON))));
  const normalized = raw / power;
  let step;
  if (normalized < 1.5) step = 1;
  else if (normalized < 3.5) step = 2;
  else if (normalized < 7.5) step = 5;
  else step = 10;
  return step * power;
}

/** 比例尺：在 maxPx 内选最长的“好看”米数。 */
export function chooseScaleBar(scale, maxPx = 140) {
  const safeScale = isFiniteNumber(scale) && scale > 0 ? scale : 40;
  const raw = Math.max(toFiniteNumber(maxPx, 140), 10) / safeScale;
  const power = Math.pow(10, Math.floor(Math.log10(Math.max(raw, EPSILON))));
  const normalized = raw / power;
  // 取不超过 raw 的最大 1/2/5×10^n，保证比例尺像素长度不超过 maxPx
  let step;
  if (normalized >= 5) step = 5;
  else if (normalized >= 2) step = 2;
  else step = 1;
  const meters = step * power;
  const pixels = meters * safeScale;
  const label = meters >= 1 ? `${formatNumber(meters, 0)} m` : `${formatNumber(meters * 100, 0)} cm`;
  return { meters, pixels, label };
}

/* ------------------------------------------------------------------------- */
/* 数值格式化（UI 统一文案）                                                  */
/* ------------------------------------------------------------------------- */

/** 数值格式化；null/NaN/非数值返回 fallback（默认 em dash）。 */
export function formatNumber(value, digits = 2, fallback = '—') {
  if (!isFiniteNumber(value)) return fallback;
  return value.toFixed(clamp(digits, 0, 6));
}

/** "1.23 m" */
export function formatMeters(value, digits = 2) {
  if (!isFiniteNumber(value)) return '—';
  return `${value.toFixed(clamp(digits, 0, 6))} m`;
}

/** "0.12 m/s" */
export function formatSpeed(value, digits = 2) {
  if (!isFiniteNumber(value)) return '—';
  return `${value.toFixed(clamp(digits, 0, 6))} m/s`;
}

/** "0.30 rad/s" */
export function formatAngularSpeed(value, digits = 2) {
  if (!isFiniteNumber(value)) return '—';
  return `${value.toFixed(clamp(digits, 0, 6))} rad/s`;
}

/** 弧度 → 角度文案 "89.9°"（归一到 (-180, 180]）。 */
export function formatYawDegrees(rad, digits = 1) {
  if (!isFiniteNumber(rad)) return '—';
  const degrees = (normalizeAngle(rad) * 180) / Math.PI;
  return `${degrees.toFixed(clamp(digits, 0, 3))}°`;
}

/** 持续时间："42 s" / "1:05" / "1:01:01"。 */
export function formatDuration(seconds) {
  if (!isFiniteNumber(seconds) || seconds < 0) return '—';
  const value = Math.round(seconds);
  if (value < 60) {
    if (seconds < 10 && Math.abs(seconds - value) > 0.05) {
      return `${seconds.toFixed(1)} s`;
    }
    return `${value} s`;
  }
  const hours = Math.floor(value / 3600);
  const minutes = Math.floor((value % 3600) / 60);
  const secs = value % 60;
  const pad = (n) => String(n).padStart(2, '0');
  return hours > 0 ? `${hours}:${pad(minutes)}:${pad(secs)}` : `${minutes}:${pad(secs)}`;
}

/** 延迟 "12.3 ms"。 */
export function formatLatency(ms) {
  if (!isFiniteNumber(ms)) return '—';
  return `${ms.toFixed(1)} ms`;
}

/** Unix 秒 → 本地时间 "HH:MM:SS"；空值返回 em dash。 */
export function formatTimestamp(ts) {
  if (!isFiniteNumber(ts)) return '—';
  const date = new Date(ts * 1000);
  if (Number.isNaN(date.getTime())) return '—';
  const pad = (n) => String(n).padStart(2, '0');
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
}

/** 数据新鲜度："20 ms" / "1.2 s"。 */
export function formatAge(seconds) {
  if (!isFiniteNumber(seconds) || seconds < 0) return '—';
  if (seconds < 1) return `${Math.round(seconds * 1000)} ms`;
  if (seconds < 10) return `${seconds.toFixed(1)} s`;
  return `${Math.round(seconds)} s`;
}

/** 比例（0..1）→ "42%"；非数值返回 em dash。 */
export function formatPercent(value, digits = 0) {
  if (!isFiniteNumber(value)) return '—';
  return `${(value * 100).toFixed(clamp(digits, 0, 2))}%`;
}
