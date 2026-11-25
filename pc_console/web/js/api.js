/**
 * api.js —— 与 pc_console 本地服务的 HTTP / SSE 封装。
 *
 * 前端始终同源访问 /api/* 与 /local/*（由 PC 程序反向代理到 Pi 网关），
 * 因此这里不拼接 Pi 地址、不携带跨域头；Pi 地址/token 由 /local/config 管理。
 *
 * 契约来源：docs/PC上位机_设计文档_v1.md §3。
 */

import {
  API,
  ERROR_MESSAGES,
  UNKNOWN_ERROR_TEXT,
  REQUEST_TIMEOUT_MS,
  SSE_RETRY_BACKOFF_MS,
  SSE_STALE_MS,
} from './config.js';

/** 统一的 API 错误对象：携带 HTTP 状态、契约错误码与 detail。 */
export class ApiError extends Error {
  constructor(message, { status = 0, code = '', detail = null, path = '' } = {}) {
    super(message);
    this.name = 'ApiError';
    this.status = status;
    this.code = code;
    this.detail = detail;
    this.path = path;
  }
}

/** 契约错误码 → 中文文案；未登记的错误码用服务端 message 或兜底文案。 */
export function mapErrorText(code, fallbackMessage) {
  if (code && ERROR_MESSAGES[code]) return ERROR_MESSAGES[code];
  if (fallbackMessage) return fallbackMessage;
  return UNKNOWN_ERROR_TEXT;
}

/** 从错误响应体提取契约错误对象（容错任意结构）。 */
function extractErrorPayload(payload) {
  if (payload && typeof payload === 'object' && payload.error && typeof payload.error === 'object') {
    return {
      code: typeof payload.error.code === 'string' ? payload.error.code : '',
      message: typeof payload.error.message === 'string' ? payload.error.message : '',
      detail: payload.error.detail ?? null,
    };
  }
  return { code: '', message: '', detail: null };
}

function statusFallbackText(status) {
  switch (status) {
    case 400:
      return '请求参数有误（400）';
    case 401:
      return '访问令牌（token）无效或缺失（401）';
    case 404:
      return '请求的资源不存在（404）';
    case 405:
      return '请求方法不被允许（405）';
    case 409:
      return '当前状态不允许该操作（409）';
    case 502:
      return '无法访问 Pi 网关（502）';
    case 503:
      return '网关或 ROS 尚未就绪（503）';
    case 500:
      return '网关内部错误（500）';
    default:
      return `请求失败（HTTP ${status}）`;
  }
}

/**
 * 创建 API 客户端。
 * @param {object} options {fetchImpl, timeoutMs}
 */
export function createApi(options = {}) {
  const fetchImpl = options.fetchImpl || ((...args) => fetch(...args));
  const defaultTimeout = Number.isFinite(options.timeoutMs) ? options.timeoutMs : REQUEST_TIMEOUT_MS;

  /**
   * 发起 JSON 请求；非 2xx 抛 ApiError（已映射为中文）。
   * @returns {Promise<any>} 解析后的 JSON（空响应返回 null）
   */
  async function request(method, path, body, options = {}) {
    const controller = new AbortController();
    const timeoutMs = Number.isFinite(options.timeoutMs) ? options.timeoutMs : defaultTimeout;
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    const hasBody = body !== undefined && body !== null;
    const headers = { Accept: 'application/json' };
    if (hasBody) headers['Content-Type'] = 'application/json; charset=utf-8';

    let response;
    try {
      response = await fetchImpl(path, {
        method,
        headers,
        body: hasBody ? JSON.stringify(body) : undefined,
        signal: controller.signal,
        cache: 'no-store',
      });
    } catch (error) {
      if (error && error.name === 'AbortError') {
        throw new ApiError(`请求超时（${Math.round(timeoutMs / 1000)} s）：${path}`, {
          code: 'timeout',
          path,
        });
      }
      throw new ApiError(mapErrorText('network'), { code: 'network', path });
    } finally {
      clearTimeout(timer);
    }

    const contentType = response.headers.get('content-type') || '';
    let payload = null;
    if (contentType.includes('application/json')) {
      try {
        payload = await response.json();
      } catch {
        payload = null;
      }
    }

    if (!response.ok) {
      const parsed = extractErrorPayload(payload);
      let message;
      if (parsed.code && ERROR_MESSAGES[parsed.code]) message = ERROR_MESSAGES[parsed.code];
      else if (parsed.message) message = parsed.message;
      else message = statusFallbackText(response.status);
      throw new ApiError(message, {
        status: response.status,
        code: parsed.code || `http_${response.status}`,
        detail: parsed.detail,
        path,
      });
    }

    return payload;
  }

  const get = (path, options) => request('GET', path, undefined, options);
  const post = (path, body, options) => request('POST', path, body ?? {}, options);

  return {
    request,
    get,
    post,

    /* ---- §3.3 网关端点 ---- */
    /** 存活探针（免 token）。 */
    health: () => get(API.health),
    /** 完整状态快照（§3.1）。 */
    state: () => get(API.state),
    /** 地图元数据。 */
    mapMeta: () => get(API.mapMeta),
    /** 已保存地图列表。 */
    maps: () => get(API.maps),
    /** 保存当前地图；冲突时抛 409 map_exists。 */
    saveMap: (name, overwrite = false) => post(API.saveMap, { name, overwrite }),
    /** 启动/切换会话模式（202 受理）。 */
    startSession: (payload) => post(API.sessionStart, payload),
    /** 停止托管会话。 */
    stopSession: () => post(API.sessionStop),
    /** 遥控速度指令（前端 20 Hz 节流）。 */
    teleop: (vx, vy, wz, seq) => post(API.teleop, { vx, vy, wz, seq }),
    /** 立即零速并退出遥控。 */
    teleopStop: () => post(API.teleopStop),
    /** 急停锁存/解除。 */
    estop: (active) => post(API.estop, { active: Boolean(active) }),
    /** 导航目的地；yaw 可传 null 表示使用机器人当前朝向。 */
    goal: (x, y, yaw = null) => post(API.goal, { x, y, yaw }),
    /** 取消当前导航目标。 */
    cancelGoal: () => post(API.goalCancel),
    /** 发布初始位姿。 */
    initialpose: (x, y, yaw) => post(API.initialpose, { x, y, yaw }),
    /** 手动重试 Nav2 激活。 */
    navActivate: () => post(API.navActivate),
    /** 蜂鸣器开关。 */
    buzzer: (on) => post(API.buzzer, { on: Boolean(on) }),
    /** RGB 灯（0..1）。 */
    led: (r, g, b) => post(API.led, { r, g, b }),
    /** 云台角度（deg，pan 0–180，tilt 0–110）。 */
    servo: (pan, tilt) => post(API.servo, { pan, tilt }),
    /** 网关事件日志。 */
    logs: (limit = 200) => get(`${API.logs}?limit=${encodeURIComponent(limit)}`),
    /** 当前/上次 bringup 的 stdout 尾部。 */
    sessionLogs: (limit = 200) => get(`${API.sessionLogs}?limit=${encodeURIComponent(limit)}`),
    /** 节点/话题调试信息。 */
    ros: () => get(API.ros),

    /* ---- §4.1 PC 本地端点 ---- */
    /** 本地配置（demo/目标/端口等）。 */
    localConfig: () => get(API.localConfig),
    /** 修改 Pi 目标；参数 {pi_host, pi_port, token}（演示模式 409）。 */
    saveLocalConfig: (payload) => post(API.localConfig, payload),
    /** 连通性探测（返回 reachable/latency_ms/target/error）。 */
    localPing: () => get(API.localPing),
  };
}

/**
 * EventSource 封装：自动重连（服务端 retry + 指数退避）与数据新鲜度看门狗。
 *
 * 用法：
 *   const stream = new EventStream({ onEvent(type, data), onStatus(status) });
 *   stream.start(); ... stream.stop();
 */
export class EventStream {
  constructor(options = {}) {
    this.url = options.url || API.events;
    this.onEvent = typeof options.onEvent === 'function' ? options.onEvent : () => {};
    this.onStatus = typeof options.onStatus === 'function' ? options.onStatus : () => {};
    this.backoff = Array.isArray(options.backoff) && options.backoff.length
      ? options.backoff
      : SSE_RETRY_BACKOFF_MS;
    this.staleMs = Number.isFinite(options.staleMs) ? options.staleMs : SSE_STALE_MS;
    this.eventSourceFactory =
      options.eventSourceFactory || ((url) => new EventSource(url));
    this.source = null;
    this.attempt = 0;
    this.connected = false;
    this.stopped = false;
    this.reconnectTimer = null;
    this.watchdogTimer = null;
    this.lastEventAt = 0;
    this.eventTypes = ['state', 'scan', 'path', 'map', 'session', 'log'];
    this._handleMessage = this._handleMessage.bind(this);
    this._handleOpen = this._handleOpen.bind(this);
    this._handleError = this._handleError.bind(this);
  }

  start() {
    this.stopped = false;
    this.attempt = 0;
    this._connect();
    this._startWatchdog();
  }

  stop() {
    this.stopped = true;
    this._clearReconnect();
    this._stopWatchdog();
    this._closeSource();
    this._setConnected(false);
  }

  /** 供 app 在页面恢复可见时主动探测：若已断开则立即重连。 */
  reconnectNow() {
    if (this.stopped || this.connected) return;
    this._clearReconnect();
    this.attempt = 0;
    this._connect();
  }

  _connect() {
    if (this.stopped || this.source) return;
    this._closeSource();
    let source;
    try {
      source = this.eventSourceFactory(this.url);
    } catch {
      this._scheduleReconnect();
      return;
    }
    this.source = source;
    source.onopen = this._handleOpen;
    source.onerror = this._handleError;
    for (const type of this.eventTypes) {
      source.addEventListener(type, this._handleMessage);
    }
  }

  _handleOpen() {
    this.attempt = 0;
    this.lastEventAt = Date.now();
    this._setConnected(true);
  }

  _handleError() {
    if (this.stopped) return;
    this._closeSource();
    this._setConnected(false);
    this._scheduleReconnect();
  }

  _handleMessage(event) {
    this.lastEventAt = Date.now();
    let data = null;
    try {
      data = event && event.data ? JSON.parse(event.data) : null;
    } catch {
      return; // 单帧损坏不影响后续流
    }
    if (data === null) return;
    try {
      this.onEvent(event.type, data);
    } catch (error) {
      // 渲染异常不能中断 SSE 连接；交给上层日志捕获
      if (typeof this.onStatus === 'function') {
        this.onStatus({
          connected: this.connected,
          attempt: this.attempt,
          error: error instanceof Error ? error : new Error(String(error)),
        });
      }
    }
  }

  _scheduleReconnect() {
    if (this.stopped || this.reconnectTimer) return;
    const delay = this.backoff[Math.min(this.attempt, this.backoff.length - 1)];
    this.attempt += 1;
    this.onStatus({ connected: false, attempt: this.attempt, retryInMs: delay });
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this._connect();
    }, delay);
  }

  _startWatchdog() {
    this._stopWatchdog();
    this.watchdogTimer = setInterval(() => {
      if (!this.connected) return;
      if (Date.now() - this.lastEventAt <= this.staleMs) return;
      // 连接看似在但 20 s 无事件：主动重建，避免半开连接造成“假在线”
      this._closeSource();
      this._setConnected(false);
      this._scheduleReconnect();
    }, Math.max(1000, Math.min(5000, this.staleMs)));
  }

  _stopWatchdog() {
    if (this.watchdogTimer) {
      clearInterval(this.watchdogTimer);
      this.watchdogTimer = null;
    }
  }

  _clearReconnect() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  _closeSource() {
    if (!this.source) return;
    try {
      this.source.onopen = null;
      this.source.onerror = null;
      for (const type of this.eventTypes) {
        this.source.removeEventListener(type, this._handleMessage);
      }
      this.source.close();
    } catch {
      /* 关闭失败忽略 */
    }
    this.source = null;
  }

  _setConnected(connected) {
    if (this.connected === connected) return;
    this.connected = connected;
    this.onStatus({ connected, attempt: this.attempt });
  }
}
