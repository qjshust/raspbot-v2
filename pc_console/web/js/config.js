/**
 * config.js —— 常量、API 路径、中文文案映射、默认值。
 * 不依赖 DOM，可被测试直接 import。
 *
 * 所有 API 路径严格来自《PC上位机_设计文档_v1.md》§3.3 与 §4.1，
 * 不得新增契约外接口（tests/dom_contract.test.mjs 会校验）。
 */

export const APP = Object.freeze({
  name: 'RASPBOT-V2 控制台',
  shortName: 'RASPBOT',
  version: '1.0.0',
  apiVersion: 1,
});

/** 契约端点（§3.3 / §4.1）。 */
export const API = Object.freeze({
  health: '/api/health',
  state: '/api/state',
  events: '/api/events',
  mapMeta: '/api/map/meta',
  maps: '/api/maps',
  saveMap: '/api/map/save',
  sessionStart: '/api/session/start',
  sessionStop: '/api/session/stop',
  teleop: '/api/teleop',
  teleopStop: '/api/teleop/stop',
  estop: '/api/estop',
  goal: '/api/goal',
  goalCancel: '/api/goal/cancel',
  initialpose: '/api/initialpose',
  navActivate: '/api/nav/activate',
  buzzer: '/api/actuator/buzzer',
  led: '/api/actuator/led',
  servo: '/api/actuator/servo',
  logs: '/api/logs',
  sessionLogs: '/api/session/logs',
  ros: '/api/ros',
  localConfig: '/local/config',
  localPing: '/local/ping',
});

/** 地图图片 URL（版本号变化即换 URL，允许长缓存）。 */
export function mapPngUrl(version) {
  const v = Number.isFinite(version) ? version : 0;
  return `/api/map.png?v=${v}`;
}

/** 地图缩略图 URL（服务端给 preview_url 时优先用服务端的）。 */
export function mapPreviewUrl(name) {
  return `/api/maps/${encodeURIComponent(String(name))}/preview.png`;
}

/* ------------------------------------------------------------------------- */
/* 会话 / 导航 / 诊断文案（§6.4）                                             */
/* ------------------------------------------------------------------------- */

export const SESSION_MODE_TEXT = Object.freeze({
  idle: '空闲',
  base: '手动遥控',
  slam: '建图',
  nav: '导航',
});

export const SESSION_STATE_TEXT = Object.freeze({
  idle: '空闲',
  starting: '正在启动…',
  running: '运行中',
  stopping: '正在停止…',
  error: '启动失败/异常退出',
});

/** 会话/导航状态对应的语义色（CSS class 后缀）：ok | info | warn | error | muted。 */
export const SESSION_STATE_TONE = Object.freeze({
  idle: 'muted',
  starting: 'info',
  running: 'ok',
  stopping: 'warn',
  error: 'error',
});

export const NAV_STATE_TEXT = Object.freeze({
  unavailable: '未进入导航模式',
  waiting_initialpose: '请在地图上设置初始位姿',
  activating: '正在激活导航…',
  ready: '导航就绪，可点击地图设置目的地',
  goal_active: '前往目的地…',
  succeeded: '已到达目的地',
  failed: '导航失败',
  canceled: '导航已取消',
  error: '导航激活失败',
});

export const NAV_STATE_TONE = Object.freeze({
  unavailable: 'muted',
  waiting_initialpose: 'warn',
  activating: 'info',
  ready: 'ok',
  goal_active: 'info',
  succeeded: 'ok',
  failed: 'error',
  canceled: 'muted',
  error: 'error',
});

export const GOAL_STATE_TEXT = Object.freeze({
  idle: '无目标',
  accepted: '目标已受理',
  active: '正在前往',
  succeeded: '已到达',
  canceled: '已取消',
  aborted: '已中止',
});

export const DIAG_LEVEL_TEXT = Object.freeze({
  ok: '正常',
  warn: '警告',
  error: '错误',
  unknown: '未知',
});

export const DIAG_LEVEL_TONE = Object.freeze({
  ok: 'ok',
  warn: 'warn',
  error: 'error',
  unknown: 'muted',
});

export const ROBOT_FRAME_TEXT = Object.freeze({
  map: 'map（已定位）',
  odom: 'odom（未定位）',
});

export const TOOL_TEXT = Object.freeze({
  pointer: '指针/平移',
  goal: '设目的地',
  initialpose: '设初始位姿',
});

/** 图层默认值（可在设置中改，持久化）。 */
export const LAYERS_DEFAULT = Object.freeze({
  grid: true,
  scan: true,
  path: true,
  trail: true,
  robot: true,
});

export const LAYER_TEXT = Object.freeze({
  grid: '网格',
  scan: '激光点',
  path: '路径',
  trail: '轨迹',
  robot: '机器人',
});

/* ------------------------------------------------------------------------- */
/* 错误码 → 中文（§3.0 / §6.2 文案映射）                                      */
/* ------------------------------------------------------------------------- */

export const ERROR_MESSAGES = Object.freeze({
  bad_request: '请求参数有误',
  unauthorized: '访问令牌（token）无效或缺失',
  token_required: '访问令牌（token）无效或缺失',
  forbidden: '没有操作权限',
  not_found: '请求的资源不存在',
  method_not_allowed: '请求方法不被允许',
  conflict: '当前状态不允许该操作',
  busy: '网关正忙，请稍候再试',
  estop_active: '已急停：请先解除急停',
  external_stack: '检测到外部 ROS 栈，网关未托管会话',
  session_idle: '会话未运行',
  no_map: '当前没有可保存的地图',
  map_exists: '同名地图已存在',
  nav_not_ready: '导航尚未就绪：请先设置初始位姿',
  nav_rejected: '导航目标被拒绝',
  goal_active: '已有导航目标，请先取消',
  upstream_unreachable: '无法访问 Pi 网关（网络或服务未启动）',
  ros_not_ready: 'ROS 链路未就绪',
  timeout: '请求超时',
  network: '无法连接本地控制台服务',
  map_load_failed: '地图图片加载失败',
  token_invalid: '访问令牌（token）无效或缺失',
  invalid_request: '请求参数有误',
  demo_mode: '演示模式下不允许该操作',
  internal_error: '网关内部错误',
});

/** 未登记错误码的兜底文案。 */
export const UNKNOWN_ERROR_TEXT = '操作失败';

/* ------------------------------------------------------------------------- */
/* 请求 / SSE / 刷新参数                                                     */
/* ------------------------------------------------------------------------- */

export const REQUEST_TIMEOUT_MS = 5000;
export const SSE_RETRY_BACKOFF_MS = Object.freeze([1000, 2000, 4000, 8000, 15000]);
export const SSE_STALE_MS = 20000;
export const HEALTH_POLL_MS = 2000;
export const DATA_STALE_MS = 5000;
export const CLOCK_TICK_MS = 1000;

export const LOG_LIMIT = 1200;
export const SESSION_LOG_FETCH_LIMIT = 200;
export const GATEWAY_LOG_LIMIT = 200;

/* ------------------------------------------------------------------------- */
/* 遥控                                                                    */
/* ------------------------------------------------------------------------- */

export const TELEOP = Object.freeze({
  hz: 20,
  intervalMs: 50,
  speedScaleDefault: 0.6,
  speedScaleMin: 0.1,
  speedScaleMax: 1,
  boostFactor: 1.6,
  deadzone: 0.08,
  errorToastIntervalMs: 3000,
});

export const LIMITS_FALLBACK = Object.freeze({
  maxLinear: 0.3,
  maxLateral: 0.3,
  maxAngular: 1.5,
});

/** 未运行/急停/未连接时遥控禁用原因。 */
export const TELEOP_DISABLED_TEXT = Object.freeze({
  disconnected: '未连接：无法遥控',
  estop: '已急停：解除后才能遥控',
  session: '会话未运行：请先选择运行模式',
  starting: '会话启动中，暂不可遥控',
});

/* ------------------------------------------------------------------------- */
/* 地图视图                                                                */
/* ------------------------------------------------------------------------- */

export const MAPVIEW = Object.freeze({
  minScale: 4,
  maxScale: 400,
  defaultScale: 40,
  wheelFactor: 1.15,
  fitPadding: 28,
  followLerp: 0.28,
  trailMax: 600,
  trailMinStep: 0.03,
  gridTargetPx: 90,
  scaleBarMaxPx: 140,
  hoverCrosshair: true,
});

export const COLORS = Object.freeze({
  mapBackground: '#0a101c',
  mapBorder: 'rgba(148, 163, 184, 0.35)',
  grid: 'rgba(148, 163, 184, 0.10)',
  gridMajor: 'rgba(148, 163, 184, 0.20)',
  axis: 'rgba(34, 211, 238, 0.35)',
  scan: 'rgba(34, 211, 238, 0.85)',
  scanMuted: 'rgba(94, 234, 212, 0.5)',
  path: '#60a5fa',
  pathGlow: 'rgba(96, 165, 250, 0.25)',
  trail: 'rgba(251, 191, 36, 0.45)',
  robot: '#34d399',
  robotOutline: '#052e22',
  robotOdom: '#fbbf24',
  goal: '#f87171',
  goalFill: 'rgba(248, 113, 113, 0.18)',
  initialpose: '#fbbf24',
  initialposeFill: 'rgba(251, 191, 36, 0.18)',
  hover: 'rgba(226, 232, 240, 0.55)',
  text: '#e6edf7',
  textMuted: 'rgba(226, 232, 240, 0.6)',
  scaleBar: 'rgba(226, 232, 240, 0.85)',
  scaleBarBg: 'rgba(11, 18, 32, 0.72)',
});

/* ------------------------------------------------------------------------- */
/* 本地存储 key                                                            */
/* ------------------------------------------------------------------------- */

export const STORAGE_KEYS = Object.freeze({
  layers: 'raspbot.console.layers.v1',
  speedScale: 'raspbot.console.speedScale.v1',
  selectedMap: 'raspbot.console.selectedMap.v1',
});

/* ------------------------------------------------------------------------- */
/* 快捷键说明（帮助弹窗与 README 对齐）                                        */
/* ------------------------------------------------------------------------- */

export const SHORTCUTS = Object.freeze([
  { keys: 'W / S', desc: '前进 / 后退（麦轮 vx）' },
  { keys: 'A / D', desc: '左 / 右横移（麦轮 vy）' },
  { keys: 'Q / E', desc: '逆时针 / 顺时针自转（wz）' },
  { keys: 'Shift', desc: '按住加速（不超过 limits 上限）' },
  { keys: 'V', desc: '指针工具（平移/查看）' },
  { keys: 'G', desc: '设目的地工具（导航就绪时）' },
  { keys: 'P', desc: '设初始位姿工具（导航模式）' },
  { keys: 'F', desc: '跟随机器人 / 取消跟随' },
  { keys: 'R', desc: '回中：视角移到机器人' },
  { keys: '0 或 H', desc: '整图适配显示' },
  { keys: '+ / -', desc: '放大 / 缩小（以画布中心）' },
  { keys: 'Esc', desc: '取消当前工具 / 关闭弹窗' },
  { keys: '双击地图', desc: '回中到机器人（指针工具下）' },
]);

/** 设置弹窗中“图层默认”的行定义。 */
export const LAYER_SETTINGS = Object.freeze([
  { key: 'grid', label: '网格与坐标轴' },
  { key: 'scan', label: '激光扫描点' },
  { key: 'path', label: '导航路径' },
  { key: 'trail', label: '机器人轨迹' },
  { key: 'robot', label: '机器人位姿箭头' },
]);

/* ------------------------------------------------------------------------- */
/* 状态判定辅助（纯函数，供 app.js 与测试使用）                                */
/* ------------------------------------------------------------------------- */

export function isSessionRunning(session) {
  return Boolean(session && session.state === 'running');
}

/** 会话处于“可以发遥控/外设/保存地图”的稳定运行态。 */
export function isOperational(session) {
  return Boolean(
    session &&
      session.state === 'running' &&
      (session.mode === 'base' || session.mode === 'slam' || session.mode === 'nav'),
  );
}

/** 是否允许在地图上设置目的地（§3.3.6 前置条件）。 */
export function canSetGoal(state) {
  return Boolean(
    state &&
      state.session &&
      state.session.mode === 'nav' &&
      state.session.state === 'running' &&
      state.nav &&
      state.nav.state === 'ready',
  );
}

/** 是否允许在地图上设置初始位姿（§3.3.7）。 */
export function canSetInitialPose(state) {
  return Boolean(
    state &&
      state.session &&
      state.session.mode === 'nav' &&
      state.session.state === 'running',
  );
}

/** 当前是否存在需要取消的导航目标。 */
export function hasActiveGoal(state) {
  if (!state || !state.nav) return false;
  const goalState = state.nav.goal_state;
  return Boolean(
    state.nav.goal &&
      (goalState === 'accepted' || goalState === 'active' || state.nav.state === 'goal_active'),
  );
}

/** 遥控是否可用（§3.3.4：base/slam/nav 的 running 且非急停）。 */
export function canTeleop(state) {
  if (!state) return false;
  if (state.estop && state.estop.active) return false;
  return isOperational(state.session);
}

/** 归一化的 limits（缺失字段用兜底值）。 */
export function normalizeLimits(limits) {
  return {
    maxLinear: Number.isFinite(limits && limits.max_linear) ? limits.max_linear : LIMITS_FALLBACK.maxLinear,
    maxLateral: Number.isFinite(limits && limits.max_lateral) ? limits.max_lateral : LIMITS_FALLBACK.maxLateral,
    maxAngular: Number.isFinite(limits && limits.max_angular) ? limits.max_angular : LIMITS_FALLBACK.maxAngular,
  };
}

/** 导航阶段进度文案（激活步骤 x/total）。 */
export function activationText(nav) {
  const activation = (nav && nav.activation) || {};
  const total = Number.isFinite(activation.total) ? activation.total : 0;
  if (!total) return '正在激活导航…';
  const done = Number.isFinite(activation.done) ? activation.done : 0;
  const phase = typeof activation.phase === 'string' && activation.phase ? activation.phase : '';
  return phase ? `正在激活导航（${done}/${total}）：${phase}` : `正在激活导航（${done}/${total}）…`;
}

/** 默认地图保存名：map_YYYYMMDD_HHMM（后端允许 ^[A-Za-z0-9_.-]{1,40}$）。 */
export function defaultMapName(date = new Date()) {
  const pad = (n) => String(n).padStart(2, '0');
  return `map_${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_${pad(date.getHours())}${pad(date.getMinutes())}`;
}

export const MAP_NAME_PATTERN = /^[A-Za-z0-9_.-]{1,40}$/;

/** 设置弹窗“连接测试”结果的语义 level。 */
export function pingResultLevel(ping) {
  if (!ping) return 'muted';
  return ping.reachable ? 'ok' : 'error';
}
