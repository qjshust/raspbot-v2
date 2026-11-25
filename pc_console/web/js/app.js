/**
 * app.js —— 控制台启动、全局 store、SSE 接入、状态渲染、交互动作与快捷键。
 *
 * 契约：docs/PC上位机_设计文档_v1.md §3 / §6。
 * 所有状态字段来自 GET /api/state 与 SSE state 事件；前端不发明后端字段。
 */

import {
  DATA_STALE_MS,
  DIAG_LEVEL_TEXT,
  DIAG_LEVEL_TONE,
  GOAL_STATE_TEXT,
  HEALTH_POLL_MS,
  LAYERS_DEFAULT,
  LAYER_SETTINGS,
  MAP_NAME_PATTERN,
  NAV_STATE_TEXT,
  NAV_STATE_TONE,
  SESSION_MODE_TEXT,
  SESSION_STATE_TEXT,
  SESSION_STATE_TONE,
  SHORTCUTS,
  STORAGE_KEYS,
  TELEOP,
  TELEOP_DISABLED_TEXT,
  activationText,
  canSetGoal,
  canSetInitialPose,
  defaultMapName,
  hasActiveGoal,
  isOperational,
  mapPngUrl,
  normalizeLimits,
} from './config.js';
import { ApiError, EventStream, createApi } from './api.js';
import { MapView } from './mapview.js';
import { TeleopController } from './teleop.js';
import {
  $el,
  LogPanel,
  ModalManager,
  ToastManager,
  fillShortcutList,
  readLayerSettings,
  renderMapList,
  setDisabled,
  setHidden,
  setPressed,
  setText,
  setTone,
  syncLayerSettings,
} from './ui.js';
import {
  clamp,
  formatAge,
  formatAngularSpeed,
  formatDuration,
  formatLatency,
  formatMeters,
  formatNumber,
  formatSpeed,
  formatTimestamp,
  formatYawDegrees,
  toFiniteNumber,
} from './math.js';

/* ========================================================================= */
/* 全局 store                                                                */
/* ========================================================================= */

const store = {
  api: null,
  mapView: null,
  teleop: null,
  modals: null,
  toast: null,
  logPanel: null,
  stream: null,

  state: null,
  prevSession: null,
  maps: [],
  selectedMapName: null,
  localConfig: null,
  demo: false,

  connection: {
    sse: false,
    health: false,
    online: false,
    everConnected: false,
    latencyMs: null,
    lastStateAt: 0,
    lastHealthAt: 0,
    lastSseStatusAt: 0,
  },

  healthTimer: null,
  clockTimer: null,
  healthInFlight: false,
  pending: new Set(),

  mapVersion: null,
  mapImageStatus: null,
  layerDefaults: { ...LAYERS_DEFAULT },
  speedScale: TELEOP.speedScaleDefault,
  tool: 'pointer',
  diagExpanded: false,
  diagSignature: '',
  errorBannerText: '',
  logKeys: new Set(),
  logWarningShown: false,
  lastSilentWarnAt: 0,
  initialBootstrapped: false,
};

/* ========================================================================= */
/* 启动                                                                      */
/* ========================================================================= */

function boot() {
  restoreLocalSettings();
  store.api = createApi();
  store.toast = new ToastManager($el('toast-root'));
  store.modals = new ModalManager({ onChange: handleModalChange });
  store.modals.attach();
  initLogPanel();
  initMapView();
  initTeleop();

  bindHeader();
  bindSessionPanel();
  bindMapToolbar();
  bindNavPanel();
  bindPeripherals();
  bindSettingsDialog();
  bindHelpDialog();
  bindKeyboardShortcuts();
  bindPageLifecycle();

  fillShortcutList($el('help-shortcuts'), SHORTCUTS);
  syncLayerSettings(store.layerDefaults);
  applyLayerCheckboxState();

  renderAllStatic();
  startConnection();
  store.clockTimer = setInterval(onClockTick, 1000);
}

/* ========================================================================= */
/* 初始化各个子模块                                                          */
/* ========================================================================= */

function initLogPanel() {
  store.logPanel = new LogPanel({
    panelEl: $el('log-panel'),
    bodyEl: $el('log-body'),
    listEl: $el('log-list'),
    countEl: $el('log-count'),
    filterEl: $el('log-filter'),
    pauseBtn: $el('btn-log-pause'),
    clearBtn: $el('btn-log-clear'),
    downloadBtn: $el('btn-log-download'),
    toggleBtn: $el('btn-log-toggle'),
    emptyEl: $el('log-empty'),
  });
}

function initMapView() {
  store.mapView = new MapView($el('map-canvas'), {
    viewportEl: $el('map-viewport'),
    onGoal: (goal) => sendGoalFromMap(goal),
    onInitialPose: (pose) => sendInitialPose(pose.x, pose.y, pose.yaw, { fromMap: true }),
    onHover: (hover) => updateCoordReadout(hover),
    onViewChange: (view) => updateZoomReadout(view),
    onFollowChange: (follow) => syncFollowUI(follow),
    onToolChange: (tool) => syncToolUI(tool),
    onToolCancel: () => {
      syncToolUI(store.mapView.tool);
      updateToolHint();
    },
    onError: (message) => {
      store.toast.error(message);
      addLocalLog('error', message);
    },
    onMapImageStatus: (status) => handleMapImageStatus(status),
  });
}

function initTeleop() {
  store.teleop = new TeleopController(store.api, {
    joystickEl: $el('joystick'),
    knobEl: $el('joystick-knob'),
    speedSliderEl: $el('speed-slider'),
    speedValueEl: $el('speed-value'),
    stopBtnEl: $el('btn-teleop-stop'),
    keyEls: {
      w: $el('key-w'),
      a: $el('key-a'),
      s: $el('key-s'),
      d: $el('key-d'),
      q: $el('key-q'),
      e: $el('key-e'),
      shift: $el('key-shift'),
    },
    speedScale: store.speedScale,
    getContext: getTeleopContext,
    confirmNavCancel: () =>
      store.modals.openConfirm({
        title: '取消导航目标？',
        message: '当前有导航目标正在执行。\n开始遥控将取消该目标并由你手动驾驶。\n\n确认继续遥控吗？',
        okText: '取消目标并遥控',
        cancelText: '保持导航',
        danger: true,
      }),
    onActiveChange: () => renderTeleopStatus(),
    onVelocity: () => {},
    onSpeedChange: (scale) => {
      store.speedScale = scale;
      persistLocalSettings();
    },
    onError: (message) => {
      store.toast.error(message);
      addLocalLog('error', message);
    },
    onBlocked: (reason) => store.toast.warn(reason),
    isInputBlocked: () => store.modals.isOpen(),
  });
  store.teleop.setLimits(normalizeLimits(null));
  store.teleop.start();
}

/* ========================================================================= */
/* 本地设置持久化                                                            */
/* ========================================================================= */

function restoreLocalSettings() {
  try {
    const rawLayers = localStorage.getItem(STORAGE_KEYS.layers);
    if (rawLayers) {
      const parsed = JSON.parse(rawLayers);
      if (parsed && typeof parsed === 'object') {
        for (const item of LAYER_SETTINGS) {
          if (typeof parsed[item.key] === 'boolean') store.layerDefaults[item.key] = parsed[item.key];
        }
        if (typeof parsed.robot === 'boolean') store.layerDefaults.robot = parsed.robot;
      }
    }
    const rawSpeed = localStorage.getItem(STORAGE_KEYS.speedScale);
    if (rawSpeed !== null) {
      const speed = Number(rawSpeed);
      if (Number.isFinite(speed)) {
        store.speedScale = clamp(speed, TELEOP.speedScaleMin, TELEOP.speedScaleMax);
      }
    }
    store.selectedMapName = localStorage.getItem(STORAGE_KEYS.selectedMap) || null;
  } catch {
    // localStorage 不可用时使用默认值（隐私模式等）
  }
}

function persistLocalSettings() {
  try {
    localStorage.setItem(STORAGE_KEYS.layers, JSON.stringify(store.layerDefaults));
    localStorage.setItem(STORAGE_KEYS.speedScale, String(store.speedScale));
    if (store.selectedMapName) localStorage.setItem(STORAGE_KEYS.selectedMap, store.selectedMapName);
    else localStorage.removeItem(STORAGE_KEYS.selectedMap);
  } catch {
    /* 忽略存储失败 */
  }
}

/* ========================================================================= */
/* 连接管理（SSE + /api/health 轮询）                                        */
/* ========================================================================= */

function startConnection() {
  store.stream = new EventStream({
    onEvent: handleStreamEvent,
    onStatus: handleStreamStatus,
  });
  store.stream.start();
  pollHealth();
  store.healthTimer = setInterval(pollHealth, HEALTH_POLL_MS);
  refreshSnapshot({ silent: true });
  refreshMaps({ silent: true });
  loadLocalConfig({ silent: true });
}

async function pollHealth() {
  if (store.healthInFlight) return;
  store.healthInFlight = true;
  const startedAt = performance.now();
  try {
    const health = await store.api.health();
    store.connection.health = Boolean(health && health.ok);
    store.connection.latencyMs = performance.now() - startedAt;
    store.connection.lastHealthAt = Date.now();
    if (!store.connection.sse && store.connection.health) {
      // SSE 尚未恢复但 HTTP 可达：降级拉取快照，保证断流时数据仍能刷新
      refreshSnapshot({ silent: true });
    }
  } catch {
    store.connection.health = false;
    store.connection.latencyMs = null;
    if (!store.connection.sse && store.connection.everConnected) {
      // 保持 lastStateAt 不变，由水印显示最后更新时间
    }
  } finally {
    store.healthInFlight = false;
    syncOnlineState();
  }
}

function syncOnlineState() {
  const online = store.connection.health || store.connection.sse;
  const changed = store.connection.online !== online;
  store.connection.online = online;
  if (changed) {
    store.connection.lastSseStatusAt = Date.now();
    if (online) {
      store.connection.everConnected = true;
      if (store.connection.everConnected) {
        // 重新上线：补拉一次完整状态与地图
        refreshSnapshot({ silent: true });
        refreshMaps({ silent: true });
        loadLocalConfig({ silent: true });
      }
    }
  }
  renderConnection();
  renderHeaderState();
  refreshAvailability();
}

function handleStreamStatus(status) {
  if (!status || typeof status !== 'object') return;
  if (status.error) {
    addLocalLog('error', `SSE 数据处理异常：${status.error.message || status.error}`);
    return;
  }
  if (typeof status.connected === 'boolean') {
    const previous = store.connection.sse;
    store.connection.sse = status.connected;
    store.connection.lastSseStatusAt = Date.now();
    if (status.connected && !previous) {
      syncOnlineState();
      if (store.initialBootstrapped) {
        store.toast.success('连接已恢复');
        addLocalLog('info', 'SSE 连接已恢复');
      }
    } else if (!status.connected && previous) {
      syncOnlineState();
      if (store.connection.everConnected) {
        store.toast.warn('与网关的连接中断，正在自动重连…');
        addLocalLog('warn', `SSE 连接中断，${status.retryInMs ? `${Math.round(status.retryInMs / 1000)} s 后重试` : '自动重连中'}`);
      }
    } else {
      syncOnlineState();
    }
  }
}

function handleStreamEvent(type, data) {
  switch (type) {
    case 'state':
      if (data && data.state) applyState(data.state);
      break;
    case 'session':
      if (data && data.session) applySessionPatch(data.session);
      break;
    case 'map':
      if (data && data.meta) applyMapMeta(data.meta);
      break;
    case 'scan':
      store.mapView?.setScan(data);
      break;
    case 'path':
      store.mapView?.setPath(data);
      break;
    case 'log':
      ingestGatewayLog(data);
      break;
    default:
      break;
  }
}

async function refreshSnapshot({ silent = false } = {}) {
  try {
    const state = await store.api.state();
    if (state && typeof state === 'object') applyState(state);
  } catch (error) {
    if (!silent) reportApiError(error, '读取状态');
    else if (Date.now() - store.lastSilentWarnAt > 10000) {
      store.lastSilentWarnAt = Date.now();
      addLocalLog('warn', `读取状态失败：${error.message || error}`);
    }
  }
}

async function refreshMaps({ silent = false } = {}) {
  try {
    const payload = await store.api.maps();
    const maps = payload && Array.isArray(payload.maps) ? payload.maps : [];
    store.maps = maps;
    if (store.selectedMapName) {
      const found = maps.find((map) => map.name === store.selectedMapName);
      if (!found || found.valid === false) store.selectedMapName = null;
    }
    if (!store.selectedMapName) {
      const firstValid = maps.find((map) => map.valid !== false);
      if (firstValid) store.selectedMapName = firstValid.name;
    }
    renderMaps();
    renderSessionPanel();
  } catch (error) {
    if (!silent) reportApiError(error, '读取地图列表');
    else if (!store.logWarningShown) {
      store.logWarningShown = true;
      addLocalLog('warn', `读取地图列表失败：${error.message || error}`);
    }
  }
}

async function loadLocalConfig({ silent = false } = {}) {
  try {
    const config = await store.api.localConfig();
    if (!config || typeof config !== 'object') return;
    store.localConfig = config;
    store.demo = Boolean(config.demo);
    renderDemoState();
    renderSettingsConfigInfo();
  } catch (error) {
    if (!silent) reportApiError(error, '读取本地配置');
  }
}

/* ========================================================================= */
/* 状态应用与渲染                                                            */
/* ========================================================================= */

function applyState(state) {
  const previous = store.state;
  store.state = state;
  store.connection.lastStateAt = Date.now();

  const session = state.session || null;
  const prevSession = previous && previous.session ? previous.session : store.prevSession;
  if (session && prevSession && session.mode !== prevSession.mode) {
    store.mapView?.clearDynamic();
    store.mapView?.setGoal(null);
    store.mapView?.setInitialPose(null);
  }
  if (session) store.prevSession = session;

  if (state.map) applyMapMeta(state.map);
  else if (previous && previous.map && previous.map.available) applyMapMeta(null);

  store.mapView?.setRobot(state.robot || null);
  store.mapView?.setGoal(state.nav && state.nav.goal ? state.nav.goal : null);
  store.mapView?.setInitialPose(state.nav && state.nav.initialpose ? state.nav.initialpose : null);
  if (!state.nav || !state.nav.goal) store.mapView?.setPath(null);

  if (state.limits) {
    store.teleop?.setLimits(normalizeLimits(state.limits));
  }

  ingestLogs(state.logs);

  if (state.estop && state.estop.active) store.teleop?.stop();

  renderConnection();
  renderHeaderState();
  renderSessionPanel();
  renderTelemetry();
  renderNavPanel();
  renderDiagnostics();
  renderPeripherals();
  renderMapChips();
  renderTeleopStatus();
  refreshAvailability();
  store.initialBootstrapped = true;
}

function applySessionPatch(session) {
  if (!session || typeof session !== 'object') return;
  if (!store.state) store.state = { session };
  else store.state = { ...store.state, session: { ...(store.state.session || {}), ...session } };
  renderSessionPanel();
  renderHeaderState();
  refreshAvailability();
}

function applyMapMeta(meta) {
  if (!meta || !meta.available) {
    if (store.mapVersion !== null) {
      store.mapVersion = null;
      store.mapImageStatus = null;
      store.mapView?.setMap(null);
      renderMapChips();
    }
    return;
  }
  store.mapView?.setMap(meta, mapPngUrl(meta.version));
  store.mapVersion = meta.version;
  renderMapChips();
}

function ingestLogs(logs) {
  if (!Array.isArray(logs)) return;
  for (const entry of logs) ingestGatewayLog(entry);
}

function ingestGatewayLog(entry) {
  if (!entry || typeof entry.msg !== 'string' || !entry.msg) return;
  const ts = Number.isFinite(entry.ts) ? entry.ts : Date.now() / 1000;
  const key = `${ts.toFixed(3)}|${entry.msg}`;
  if (store.logKeys.has(key)) return;
  store.logKeys.add(key);
  if (store.logKeys.size > 4000) store.logKeys.clear();
  store.logPanel?.add({ ts, level: entry.level, msg: entry.msg });
}

function addLocalLog(level, msg) {
  store.logPanel?.add({ ts: Date.now() / 1000, level, msg, source: '控制台' });
}

function reportApiError(error, context) {
  const message = error && error.message ? error.message : '操作失败';
  const text = context ? `${context}：${message}` : message;
  store.toast.error(text);
  addLocalLog('error', text);
  return text;
}

function handleModalChange(open) {
  // 弹窗打开时立即停发遥控（导航取消确认弹窗除外：确认后用户可继续按住键遥控）
  if (open && store.teleop && !store.teleop.navConfirmPending) {
    store.teleop.stopInputs({ sendStop: true });
  }
}

/* ------------------------------------------------------------------------- */
/* 连接 / 顶栏                                                               */
/* ------------------------------------------------------------------------- */

function renderConnection() {
  const c = store.connection;
  const now = Date.now();
  const stale = c.online && c.everConnected && c.lastStateAt > 0 && now - c.lastStateAt > DATA_STALE_MS;
  const dot = $el('conn-dot');
  const text = $el('conn-text');
  const latency = $el('latency-text');

  if (dot) dot.className = `status-dot ${c.online ? 'ok' : c.everConnected ? 'bad' : 'warn'}`;
  if (c.online) {
    setText(text, c.sse ? '已连接' : '已连接（HTTP 轮询）');
    setText(latency, c.latencyMs === null ? '—' : formatLatency(c.latencyMs));
  } else {
    setText(text, c.everConnected ? '未连接：无法访问 Pi 网关' : '正在连接…');
    setText(latency, '—');
  }

  const offline = !c.online || stale;
  $el('map-viewport')?.classList.toggle('offline', offline);
  const watermark = $el('map-watermark');
  setHidden(watermark, !offline);
  if (offline) {
    setText($el('map-watermark-title'), !c.online ? '未连接' : '数据延迟');
    setText(
      $el('map-watermark-time'),
      c.lastStateAt ? `最后更新 ${formatTimestamp(c.lastStateAt / 1000)}` : '最后更新 —',
    );
  }
  renderMapChips();
}

function renderHeaderState() {
  const state = store.state;
  const session = state && state.session;
  const estopActive = Boolean(state && state.estop && state.estop.active);
  const online = store.connection.online;

  // 演示模式徽章
  setHidden($el('badge-demo'), !store.demo);

  // 模式/会话徽章
  const modeText = session ? SESSION_MODE_TEXT[session.mode] || session.mode : '未连接';
  const stateText = session ? SESSION_STATE_TEXT[session.state] || session.state : '—';
  const badge = $el('badge-mode');
  setText(badge, session ? `${modeText} · ${stateText}` : online ? '未知' : '未连接');
  setTone(badge, session ? SESSION_STATE_TONE[session.state] || 'muted' : 'muted');

  const sessionHeader = $el('session-header-text');
  setText(sessionHeader, session ? `${modeText} · ${stateText}` : online ? '未知' : '未连接');
  const sessionDot = $el('session-dot');
  if (sessionDot) {
    sessionDot.className = `status-dot ${
      session && session.state === 'running' ? 'ok' : session && session.state === 'error' ? 'bad' : session && session.state !== 'idle' ? 'warn' : 'info'
    }`;
  }

  // ROS 健康
  const ros = state && state.ros;
  const rosDot = $el('ros-dot');
  const rosText = $el('ros-text');
  const rosItem = $el('ros-item');
  if (!ros) {
    if (rosDot) rosDot.className = 'status-dot';
    setText(rosText, online ? '未知' : '—');
  } else {
    const ready = ros.ready || {};
    const readyNames = [
      ['base', '底盘'],
      ['teleop', '遥控链路'],
      ['map', '地图'],
      ['localization', '定位'],
      ['nav2', 'Nav2'],
      ['diagnostics', '诊断'],
    ];
    const readyText = readyNames.map(([key, label]) => `${label}${ready[key] ? '✓' : '✗'}`).join(' ');
    const tone = ros.ok ? 'ok' : ready.base ? 'warn' : 'bad';
    if (rosDot) rosDot.className = `status-dot ${tone}`;
    setText(rosText, ros.ok ? '正常' : ready.base ? '部分就绪' : '未就绪');
    if (rosItem) {
      const rates = ros.rates || {};
      rosItem.title = [
        `节点数：${ros.nodes ?? '—'}`,
        `就绪：${readyText}`,
        `速率：scan ${formatNumber(rates.scan_hz, 1)} Hz / odom ${formatNumber(rates.odom_hz, 1)} Hz`,
        `地图年龄：${formatAge(rates.map_age_s)} / 诊断年龄：${formatAge(rates.diagnostics_age_s)}`,
      ].join('\n');
    }
  }

  // 急停
  const estopBtn = $el('btn-estop');
  if (estopBtn) {
    estopBtn.classList.toggle('active', estopActive);
    estopBtn.setAttribute('aria-pressed', estopActive ? 'true' : 'false');
    setText($el('estop-text'), estopActive ? '已急停' : '急停');
    estopBtn.title = estopActive ? '急停已锁存：点击解除（需二次确认）' : '软件急停（立即锁存）';
  }
  setHidden($el('estop-banner'), !estopActive);

  // ERROR 横幅
  const diag = state && state.diagnostics;
  let bannerText = '';
  if (diag && (diag.level === 'error' || (diag.counts && diag.counts.error > 0))) {
    const errorItem = Array.isArray(diag.items) ? diag.items.find((item) => item && item.level === 'error') : null;
    bannerText = errorItem ? `${errorItem.name || '诊断'}：${errorItem.message || '错误'}` : '诊断出现 ERROR，请查看右栏诊断面板';
  }
  if (session && session.state === 'error' && session.error) {
    bannerText = `会话启动失败：${session.error}`;
  }
  const banner = $el('error-banner');
  if (bannerText !== store.errorBannerText) {
    store.errorBannerText = bannerText;
    setHidden(banner, !bannerText);
    setText($el('error-banner-text'), bannerText);
  } else {
    setHidden(banner, !bannerText);
  }
}

/* ------------------------------------------------------------------------- */
/* 左栏：会话 / 地图列表 / 外设                                               */
/* ------------------------------------------------------------------------- */

function renderSessionPanel() {
  const session = store.state && store.state.session;
  const mode = session ? session.mode : 'idle';
  const sessionState = session ? session.state : 'idle';

  setText($el('session-mode-badge'), SESSION_MODE_TEXT[mode] || mode);
  setTone($el('session-mode-badge'), mode === 'idle' ? 'muted' : 'info');

  const showIdle = mode === 'idle' && sessionState !== 'error';
  setHidden($el('session-idle'), !showIdle);
  setHidden($el('session-active'), showIdle);

  const selected = getSelectedMap();
  setText($el('mode-nav-map'), selected ? selected.name : '未选择地图');

  if (session) {
    setText($el('session-mode-text'), `${SESSION_MODE_TEXT[mode] || mode}（${SESSION_STATE_TEXT[sessionState] || sessionState}）`);
    const stateBadge = $el('session-state-badge');
    setText(stateBadge, SESSION_STATE_TEXT[sessionState] || sessionState);
    setTone(stateBadge, SESSION_STATE_TONE[sessionState] || 'muted');

    const progressWrap = $el('session-progress-wrap');
    const showProgress = sessionState === 'starting' || sessionState === 'stopping';
    setHidden(progressWrap, !showProgress);
    if (showProgress) {
      const progress = clamp(toFiniteNumber(session.progress, 0), 0, 1);
      progressWrap.classList.toggle('indeterminate', progress <= 0.01);
      const bar = $el('session-progress-bar');
      if (bar) bar.style.width = `${Math.round(progress * 100)}%`;
    }
    setText($el('session-message'), session.message || '—');
    const errorEl = $el('session-error');
    const hasError = Boolean(session.error);
    setHidden(errorEl, !hasError);
    setText(errorEl, hasError ? session.error : '');
  }
}

function renderMaps() {
  renderMapList({
    listEl: $el('map-list'),
    emptyEl: $el('map-list-empty'),
    maps: store.maps,
    selectedName: store.selectedMapName,
    onSelect: handleMapSelect,
  });
}

function handleMapSelect(map) {
  if (!map || map.valid === false) {
    store.toast.warn('该地图文件不完整（缺少 pgm 或字段损坏），不能用于导航');
    return;
  }
  store.selectedMapName = map.name;
  persistLocalSettings();
  renderMaps();
  renderSessionPanel();
  const session = store.state && store.state.session;
  if (session && session.mode === 'nav' && session.state === 'running') {
    store.toast.info(`已选择地图 ${map.name}；当前导航会话不受影响，新会话将使用该地图`);
  } else {
    store.toast.info(`已选择地图 ${map.name}，点击「开始导航」启动`);
  }
}

function getSelectedMap() {
  if (!store.selectedMapName) return null;
  return store.maps.find((map) => map.name === store.selectedMapName) || null;
}

function renderPeripherals() {
  const state = store.state;
  const periph = state && state.peripherals;
  const online = store.connection.online;
  const operational = Boolean(state && isOperational(state.session));
  const usable = online && operational;

  // 蜂鸣器
  const buzzerOn = Boolean(periph && periph.buzzer);
  const buzzerBtn = $el('btn-buzzer');
  if (buzzerBtn) {
    setPressed(buzzerBtn, buzzerOn);
    setText($el('buzzer-state'), buzzerOn ? '开' : '关');
  }

  // RGB 灯
  const ledColor = $el('led-color');
  if (ledColor && periph && periph.led && document.activeElement !== ledColor) {
    const led = periph.led;
    ledColor.value = rgbToHex(led.r, led.g, led.b);
  }

  // 云台
  const pan = periph && periph.servo && Number.isFinite(periph.servo.pan) ? periph.servo.pan : 90;
  const tilt = periph && periph.servo && Number.isFinite(periph.servo.tilt) ? periph.servo.tilt : 45;
  const panSlider = $el('servo-pan');
  const tiltSlider = $el('servo-tilt');
  if (panSlider && document.activeElement !== panSlider) panSlider.value = String(Math.round(pan));
  if (tiltSlider && document.activeElement !== tiltSlider) tiltSlider.value = String(Math.round(tilt));
  setText($el('servo-pan-value'), `${Math.round(Number(panSlider ? panSlider.value : pan))}°`);
  setText($el('servo-tilt-value'), `${Math.round(Number(tiltSlider ? tiltSlider.value : tilt))}°`);

  setDisabled($el('btn-buzzer'), !usable);
  setDisabled($el('btn-led-apply'), !usable);
  setDisabled($el('btn-led-off'), !usable);
  setDisabled($el('btn-servo-apply'), !usable);
  setDisabled(panSlider, !usable);
  setDisabled(tiltSlider, !usable);
  setDisabled($el('led-color'), !usable);
  setText(
    $el('periph-hint'),
    usable ? '外设控制可用（会话运行中）' : '会话运行且连接正常时可控制外设',
  );
}

function rgbToHex(r, g, b) {
  const toByte = (value) => {
    const v = clamp(toFiniteNumber(value, 0), 0, 1);
    return Math.round(v * 255).toString(16).padStart(2, '0');
  };
  return `#${toByte(r)}${toByte(g)}${toByte(b)}`;
}

function hexToRgb(hex) {
  const match = /^#?([0-9a-f]{6})$/i.exec(String(hex || ''));
  if (!match) return { r: 0, g: 0, b: 0 };
  const value = parseInt(match[1], 16);
  return {
    r: ((value >> 16) & 0xff) / 255,
    g: ((value >> 8) & 0xff) / 255,
    b: (value & 0xff) / 255,
  };
}

/* ------------------------------------------------------------------------- */
/* 右栏：遥测 / 导航 / 诊断                                                  */
/* ------------------------------------------------------------------------- */

function renderTelemetry() {
  const state = store.state;
  const robot = state && state.robot;
  const frameBadge = $el('tele-frame');
  if (!robot) {
    for (const id of ['tele-x', 'tele-y', 'tele-yaw', 'tele-vx', 'tele-vy', 'tele-wz', 'tele-imu', 'tele-scan-front', 'tele-scan-min']) {
      setText($el(id), '—');
    }
    setText(frameBadge, '无数据');
    setTone(frameBadge, 'muted');
    setText($el('tele-age'), store.connection.lastStateAt ? formatAge((Date.now() - store.connection.lastStateAt) / 1000) : '—');
    return;
  }

  setText($el('tele-x'), formatMeters(robot.x));
  setText($el('tele-y'), formatMeters(robot.y));
  setText($el('tele-yaw'), formatYawDegrees(robot.yaw));
  const speed = robot.speed || {};
  setText($el('tele-vx'), formatSpeed(speed.vx));
  setText($el('tele-vy'), formatSpeed(speed.vy));
  setText($el('tele-wz'), formatAngularSpeed(speed.wz));
  const imu = robot.imu || {};
  setText(
    $el('tele-imu'),
    Number.isFinite(imu.yaw_deg) ? `${formatNumber(imu.yaw_deg, 1)}°` : '—',
  );
  const scan = robot.scan || {};
  setText($el('tele-scan-front'), formatMeters(scan.front_m));
  setText($el('tele-scan-min'), formatMeters(scan.min_m));

  const frame = robot.frame || '—';
  setText(frameBadge, frame === 'map' ? 'map 已定位' : frame === 'odom' ? 'odom 未定位' : String(frame));
  setTone(frameBadge, frame === 'map' ? 'ok' : 'warn');
  setText(
    $el('tele-age'),
    store.connection.lastStateAt ? formatAge((Date.now() - store.connection.lastStateAt) / 1000) : '—',
  );
}

function renderNavPanel() {
  const state = store.state;
  const session = state && state.session;
  const nav = state && state.nav;
  const navStateEl = $el('nav-state-text');
  const toneChip = $el('nav-tone-chip');

  if (!nav) {
    setText(navStateEl, '未进入导航模式');
    setTone(navStateEl, 'muted');
    setText(toneChip, '—');
    setTone(toneChip, 'muted');
    setHidden($el('nav-activation-wrap'), true);
    setText($el('nav-activation-text'), '');
    for (const id of ['nav-distance', 'nav-time', 'nav-recoveries', 'nav-goal-coord', 'nav-initialpose-coord']) {
      setText($el(id), '—');
    }
    setText($el('nav-map-name'), session && session.map ? basename(session.map) : '—');
    return;
  }

  let text = NAV_STATE_TEXT[nav.state] || nav.state || '未知';
  let tone = NAV_STATE_TONE[nav.state] || 'muted';
  if (session && session.mode !== 'nav') {
    text = '未进入导航模式（当前非导航会话）';
    tone = 'muted';
  } else if (session && session.state !== 'running' && session.state !== 'starting') {
    text = `导航会话未运行：${SESSION_STATE_TEXT[session.state] || session.state}`;
    tone = session.state === 'error' ? 'error' : 'muted';
  }
  const reason = nav.message && nav.message !== text ? nav.message : '';
  if (reason && (nav.state === 'failed' || nav.state === 'error' || nav.state === 'canceled')) {
    text = `${text}：${reason}`;
  }
  // §6.4 文案：前往目的地时同时给出剩余距离与已用时间
  if (nav.state === 'goal_active' && session && session.mode === 'nav' && session.state === 'running') {
    const parts = [];
    if (Number.isFinite(nav.distance_remaining)) parts.push(`剩余 ${formatMeters(nav.distance_remaining)}`);
    if (Number.isFinite(nav.navigation_time_s)) parts.push(`已用 ${formatDuration(nav.navigation_time_s)}`);
    if (parts.length) text = `前往目的地：${parts.join(' / ')}`;
  }
  setText(navStateEl, text);
  setTone(navStateEl, tone);

  const goalState = nav.goal_state || 'idle';
  setText(toneChip, GOAL_STATE_TEXT[goalState] || goalState);
  setTone(
    toneChip,
    goalState === 'succeeded'
      ? 'ok'
      : goalState === 'aborted'
        ? 'error'
        : goalState === 'active' || goalState === 'accepted'
          ? 'info'
          : 'muted',
  );

  setText($el('nav-distance'), formatMeters(nav.distance_remaining));
  setText($el('nav-time'), formatDuration(nav.navigation_time_s));
  setText(
    $el('nav-recoveries'),
    Number.isFinite(nav.recoveries) ? String(nav.recoveries) : '—',
  );
  setText($el('nav-goal-coord'), formatPose(nav.goal));
  setText($el('nav-initialpose-coord'), formatPose(nav.initialpose));
  setText($el('nav-map-name'), session && session.map ? basename(session.map) : '—');

  const activating = nav.state === 'activating';
  const activation = nav.activation || {};
  const total = Number.isFinite(activation.total) ? activation.total : 0;
  setHidden($el('nav-activation-wrap'), !activating);
  if (activating) {
    const done = Number.isFinite(activation.done) ? activation.done : 0;
    const bar = $el('nav-activation-bar');
    if (bar) bar.style.width = `${total > 0 ? Math.round((done / total) * 100) : 0}%`;
    setText($el('nav-activation-text'), activationText(nav));
  } else if (nav.state === 'waiting_initialpose') {
    setText($el('nav-activation-text'), '提示：用「初始位姿」工具在地图上标定，或使用右栏数值输入');
  } else {
    setText($el('nav-activation-text'), '');
  }
}

function formatPose(pose) {
  if (!pose) return '—';
  const x = formatMeters(pose.x);
  const y = formatMeters(pose.y);
  if (!Number.isFinite(pose.yaw)) return `${x}, ${y}`;
  return `${x}, ${y}, ${formatYawDegrees(pose.yaw)}`;
}

function basename(path) {
  if (typeof path !== 'string' || !path) return '—';
  const parts = path.split(/[\\/]/);
  return parts[parts.length - 1] || path;
}

function renderDiagnostics() {
  const state = store.state;
  const diag = state && state.diagnostics;
  const okEl = $el('diag-ok');
  const warnEl = $el('diag-warn');
  const errorEl = $el('diag-error');
  const summaryEl = $el('diag-summary');

  if (!diag) {
    setText(okEl, '0');
    setText(warnEl, '0');
    setText(errorEl, '0');
    setText($el('diag-age'), '—');
    setText(summaryEl, '等待诊断数据…');
    setTone(summaryEl, 'muted');
    renderDiagList([]);
    return;
  }

  const counts = diag.counts || {};
  setText(okEl, String(Number.isFinite(counts.ok) ? counts.ok : 0));
  setText(warnEl, String(Number.isFinite(counts.warn) ? counts.warn : 0));
  setText(errorEl, String(Number.isFinite(counts.error) ? counts.error : 0));
  setText($el('diag-age'), formatAge(diag.age_s));

  const level = DIAG_LEVEL_TEXT[diag.level] ? diag.level : 'unknown';
  const items = Array.isArray(diag.items) ? diag.items : [];
  const badItems = items.filter((item) => item && (item.level === 'warn' || item.level === 'error'));
  let summary = '所有诊断项正常';
  if (level === 'error') {
    summary = badItems.length ? badItems.slice(0, 3).map((item) => `${item.name || '诊断'}：${item.message || 'ERROR'}`).join('；') : '诊断出现 ERROR';
  } else if (level === 'warn') {
    summary = badItems.length ? badItems.slice(0, 3).map((item) => `${item.name || '诊断'}：${item.message || 'WARN'}`).join('；') : '诊断出现警告';
  } else if (level === 'unknown') {
    summary = '诊断状态未知（等待 /diagnostics）';
  }
  setText(summaryEl, summary);
  setTone(summaryEl, DIAG_LEVEL_TONE[level] || 'muted');
  renderDiagList(items);
}

function renderDiagList(items) {
  const listEl = $el('diag-list');
  const expandBtn = $el('btn-diag-expand');
  if (!listEl) return;
  const signature = `${store.diagExpanded}|${JSON.stringify(items)}`;
  if (signature !== store.diagSignature) {
    store.diagSignature = signature;
    listEl.textContent = '';
    if (items.length === 0) {
      if (store.diagExpanded) {
        const empty = document.createElement('li');
        empty.className = 'diag-item';
        empty.textContent = '无诊断条目';
        listEl.append(empty);
      }
    } else {
      for (const item of items) {
        const li = document.createElement('li');
        const level = item && item.level ? item.level : 'unknown';
        li.className = `diag-item tone-${DIAG_LEVEL_TONE[level] || 'muted'}`;
        const name = document.createElement('span');
        name.className = 'diag-item-name';
        name.textContent = `${DIAG_LEVEL_TEXT[level] || '未知'} · ${item && item.name ? item.name : '诊断项'}`;
        const message = document.createElement('span');
        message.className = 'diag-item-message';
        message.textContent = (item && item.message) || '—';
        li.append(name, message);
        listEl.append(li);
      }
    }
  }
  setHidden(listEl, !store.diagExpanded);
  if (expandBtn) {
    expandBtn.setAttribute('aria-expanded', store.diagExpanded ? 'true' : 'false');
    setText(expandBtn, store.diagExpanded ? '收起' : '展开全部');
  }
}

/* ------------------------------------------------------------------------- */
/* 地图 HUD / 工具链                                                         */
/* ------------------------------------------------------------------------- */

function renderMapChips() {
  const state = store.state;
  const map = state && state.map;
  const frame = (state && state.robot && state.robot.frame) || (map && map.frame_id) || null;
  setText($el('map-frame-chip'), `帧 ${frame || '—'}`);
  if (map && map.available) {
    setText(
      $el('map-size-chip'),
      `${map.width ?? '—'}×${map.height ?? '—'} · ${formatNumber(map.resolution, 3)} m`,
    );
    setText($el('map-version-chip'), `v${map.version ?? '—'}`);
  } else {
    setText($el('map-size-chip'), '—');
    setText($el('map-version-chip'), 'v—');
  }

  // 地图空态提示 / 底图加载状态
  const emptyEl = $el('map-empty');
  const hasMap = Boolean(map && map.available);
  if (!hasMap) {
    const session = state && state.session;
    const sessionRunning = session && (session.state === 'starting' || session.state === 'running');
    setHidden(emptyEl, false);
    setText(
      emptyEl,
      sessionRunning ? '等待首帧地图…' : '暂无地图数据（启动建图 / 导航会话后显示）',
    );
  } else if (store.mapImageStatus === 'loading') {
    setHidden(emptyEl, false);
    setText(emptyEl, '地图底图加载中…');
  } else if (store.mapImageStatus === 'failed') {
    setHidden(emptyEl, false);
    setText(emptyEl, '地图底图加载失败，请检查网关 /api/map.png');
  } else {
    setHidden(emptyEl, true);
  }
}

function handleMapImageStatus(status) {
  store.mapImageStatus = status;
  renderMapChips();
}

function updateZoomReadout(view) {
  if (!view) return;
  setText($el('zoom-level'), `${Math.round(view.scale)} px/m`);
}

function updateCoordReadout(hover) {
  if (!hover || !hover.inside) {
    setText($el('map-coord'), 'x — / y —');
    return;
  }
  const x = toFiniteNumber(hover.world && hover.world.x, 0);
  const y = toFiniteNumber(hover.world && hover.world.y, 0);
  setText($el('map-coord'), `x ${x.toFixed(2)} m / y ${y.toFixed(2)} m`);
}

function syncToolUI(tool) {
  store.tool = tool;
  for (const [id, value] of [
    ['tool-pointer', 'pointer'],
    ['tool-goal', 'goal'],
    ['tool-initialpose', 'initialpose'],
  ]) {
    const btn = $el(id);
    if (!btn) continue;
    btn.classList.toggle('active', tool === value);
    btn.setAttribute('aria-pressed', tool === value ? 'true' : 'false');
  }
  updateToolHint();
}

function updateToolHint() {
  const hint = $el('map-tool-hint');
  if (!hint) return;
  const tool = store.tool;
  if (tool === 'goal') {
    setText(hint, '点击地图设置目的地 · Esc 取消');
    setHidden(hint, false);
  } else if (tool === 'initialpose') {
    setText(hint, '按住地图确定位置，拖拽确定朝向，松开发送 · Esc 取消');
    setHidden(hint, false);
  } else {
    setHidden(hint, true);
  }
}

function syncFollowUI(follow) {
  const btn = $el('btn-follow');
  if (btn) {
    btn.classList.toggle('active', Boolean(follow));
    btn.setAttribute('aria-pressed', follow ? 'true' : 'false');
  }
  setHidden($el('map-follow-flag'), !follow);
  if (follow) store.toast.info('已开启跟随：视角将跟随机器人（拖拽地图会关闭）');
}

function selectTool(tool) {
  if (!store.mapView) return;
  if (tool === 'goal') {
    if (!goalAllowed()) {
      store.toast.warn(goalBlockedReason());
      return;
    }
  } else if (tool === 'initialpose') {
    if (!initialPoseAllowed()) {
      store.toast.warn(initialPoseBlockedReason());
      return;
    }
  }
  store.mapView.setTool(tool);
}

function toggleFollow() {
  if (!store.mapView) return;
  if (!store.mapView.follow) {
    const robot = store.state && store.state.robot;
    if (!robot) {
      store.toast.warn('机器人位姿未知，暂无法跟随');
      return;
    }
    store.mapView.setFollow(true);
  } else {
    store.mapView.setFollow(false);
    store.toast.info('已关闭跟随');
  }
}

/* ------------------------------------------------------------------------- */
/* 可用性 / 按钮禁用                                                          */
/* ------------------------------------------------------------------------- */

function goalAllowed() {
  const state = store.state;
  return Boolean(store.connection.online) && !(state && state.estop && state.estop.active) && canSetGoal(state);
}

function goalBlockedReason() {
  const state = store.state;
  if (!store.connection.online) return '未连接：无法设置目的地';
  if (state && state.estop && state.estop.active) return '已急停：解除后才能设置目的地';
  if (!state || !state.session || state.session.mode !== 'nav' || state.session.state !== 'running') {
    return '未进入导航模式：请先启动导航会话';
  }
  if (!state.nav || state.nav.state === 'waiting_initialpose') return '导航尚未就绪：请先设置初始位姿';
  if (state.nav.state === 'activating') return '导航正在激活中，请稍候…';
  if (state.nav.state === 'goal_active') return '已有导航目标，请先取消或到达';
  return '导航尚未就绪，请稍候…';
}

function initialPoseAllowed() {
  const state = store.state;
  return Boolean(store.connection.online) && canSetInitialPose(state);
}

function initialPoseBlockedReason() {
  const state = store.state;
  if (!store.connection.online) return '未连接：无法设置初始位姿';
  if (!state || !state.session || state.session.mode !== 'nav' || state.session.state !== 'running') {
    return '仅在导航会话运行时可设置初始位姿';
  }
  return '当前不可设置初始位姿';
}

function refreshAvailability() {
  const state = store.state;
  const c = store.connection;
  const online = c.online;
  const session = state && state.session;
  const estop = Boolean(state && state.estop && state.estop.active);
  const running = Boolean(session && session.state === 'running');
  const operational = Boolean(session && isOperational(session));
  const idle = !session || (session.mode === 'idle' && session.state !== 'error');
  const busy = store.pending.size > 0;

  // 会话模式卡
  const modesAllowed = online && !estop && idle && !busy;
  setDisabled($el('mode-base'), !modesAllowed);
  setDisabled($el('mode-slam'), !modesAllowed);
  setDisabled($el('mode-nav'), !modesAllowed);

  // 会话内操作
  setDisabled($el('btn-save-map'), !online || !operational || store.pending.has('save-map'));
  setDisabled(
    $el('btn-stop-session'),
    !online || !session || session.mode === 'idle' || store.pending.has('session-stop'),
  );

  // 急停
  setDisabled($el('btn-estop'), !online || store.pending.has('estop'));

  // 地图工具
  const goalOk = goalAllowed();
  const poseOk = initialPoseAllowed();
  setDisabled($el('tool-goal'), !goalOk);
  setDisabled($el('tool-initialpose'), !poseOk);
  setDisabled($el('btn-follow'), !online);
  setDisabled($el('btn-recenter'), !online);
  setDisabled($el('btn-fit'), !online);
  setDisabled($el('btn-zoom-in'), !online);
  setDisabled($el('btn-zoom-out'), !online);

  if ((store.tool === 'goal' && !goalOk) || (store.tool === 'initialpose' && !poseOk)) {
    store.mapView?.cancelTool();
    syncToolUI(store.mapView ? store.mapView.tool : 'pointer');
  }

  // 导航按钮
  const nav = state && state.nav;
  const navModeRunning = Boolean(session && session.mode === 'nav' && running);
  setDisabled($el('btn-cancel-goal'), !online || !hasActiveGoal(state) || store.pending.has('cancel-goal'));
  setDisabled(
    $el('btn-nav-activate'),
    !online ||
      !navModeRunning ||
      !(nav && (nav.state === 'error' || nav.state === 'unavailable')) ||
      store.pending.has('nav-activate'),
  );
  setDisabled($el('btn-set-initialpose'), !poseOk);
  setDisabled($el('btn-send-goal'), !goalOk || store.pending.has('goal'));
  setDisabled($el('btn-send-initialpose'), !poseOk || store.pending.has('initialpose'));

  for (const id of ['goal-x', 'goal-y', 'goal-yaw']) setDisabled($el(id), !goalOk);
  for (const id of ['init-x', 'init-y', 'init-yaw']) setDisabled($el(id), !poseOk);

  // 遥控可用性（与 getTeleopContext 使用同一判定，保证状态文案一致）
  const teleopContext = getTeleopContext();
  store.teleop?.setEnabled(teleopContext.enabled, teleopContext.reason);

  setDisabled($el('btn-save-map-confirm'), store.pending.has('save-map'));

  renderTeleopStatus();
}

function getTeleopContext() {
  const state = store.state;
  const online = store.connection.online;
  const session = state && state.session;
  let enabled = false;
  let reason = '';
  if (!online) reason = TELEOP_DISABLED_TEXT.disconnected;
  else if (state && state.estop && state.estop.active) reason = TELEOP_DISABLED_TEXT.estop;
  else if (session && (session.state === 'starting' || session.state === 'stopping')) {
    reason = TELEOP_DISABLED_TEXT.starting;
  } else if (!state || !isOperational(session)) reason = TELEOP_DISABLED_TEXT.session;
  else enabled = true;
  return {
    enabled,
    reason,
    limits: normalizeLimits(state && state.limits),
    navHasGoal: hasActiveGoal(state),
  };
}

function renderTeleopStatus() {
  const statusEl = $el('teleop-status');
  const dot = $el('teleop-status-dot');
  const text = $el('teleop-status-text');
  if (!statusEl) return;
  const active = Boolean(store.teleop && store.teleop.active);
  const context = getTeleopContext();
  if (active) {
    setText(text, '遥控中');
    setTone(statusEl, 'ok');
    if (dot) dot.className = 'status-dot ok';
    return;
  }
  setText(text, context.enabled ? '遥控就绪' : context.reason);
  setTone(statusEl, context.enabled ? 'ok' : store.connection.online ? 'warn' : 'error');
  if (dot) {
    dot.className = `status-dot ${context.enabled ? 'ok' : store.connection.online ? 'warn' : 'bad'}`;
  }
}

function onClockTick() {
  renderTeleopStatus();
  renderTelemetry();
  const c = store.connection;
  renderConnection();
  if (c.online && c.lastStateAt && Date.now() - c.lastStateAt > DATA_STALE_MS) {
    setHidden($el('map-watermark'), false);
    setText($el('map-watermark-title'), '数据延迟');
    setText($el('map-watermark-time'), `最后更新 ${formatTimestamp(c.lastStateAt / 1000)}`);
  }
}

/* ========================================================================= */
/* 会话 / 导航 / 外设动作                                                    */
/* ========================================================================= */

async function withPending(id, action) {
  if (store.pending.has(id)) return null;
  store.pending.add(id);
  refreshAvailability();
  try {
    return await action();
  } finally {
    store.pending.delete(id);
    refreshAvailability();
  }
}

async function handleStartSession(mode) {
  if (!store.connection.online) {
    store.toast.error('未连接：无法启动会话');
    return;
  }
  const state = store.state;
  if (state && state.estop && state.estop.active) {
    store.toast.error('已急停：请先解除急停再启动会话');
    return;
  }

  const payload = { mode, map: null, use_mock: false, use_camera: false, use_perception: false };
  if (mode === 'nav') {
    if (store.maps.length === 0) {
      store.toast.warn('暂无已保存地图：请先启动建图会话，保存地图后再开始导航');
      return;
    }
    const map = getSelectedMap();
    if (!map) {
      store.toast.warn('请先在「地图列表」中选择一张地图，再开始导航');
      return;
    }
    if (map.valid === false) {
      store.toast.error('所选地图无效（缺少 pgm 或 YAML 字段损坏），请重新建图并保存');
      return;
    }
    const ok = await store.modals.openConfirm({
      title: '启动导航',
      message: `将停止当前进程并启动导航模式。\n地图：${map.name}\n\n启动后需要在地图上设置初始位姿，Nav2 才会激活（可能需要 20–60 s）。`,
      okText: '启动导航',
    });
    if (!ok) return;
    payload.map = map.yaml || map.path;
  } else {
    const ok = await store.modals.openConfirm({
      title: `启动${SESSION_MODE_TEXT[mode]}`,
      message: `将启动${SESSION_MODE_TEXT[mode]}会话，bringup 进程需要数秒启动。\n（若当前已有进程，网关会先停止再启动。）\n\n确认开始吗？`,
      okText: '启动会话',
    });
    if (!ok) return;
  }

  await withPending('session-start', async () => {
    try {
      await store.api.startSession(payload);
      store.toast.success(`已受理：正在启动${SESSION_MODE_TEXT[mode]}会话…`);
      addLocalLog('info', `请求启动会话：mode=${mode}${payload.map ? ` map=${payload.map}` : ''}`);
    } catch (error) {
      reportApiError(error, '启动会话');
    }
  });
}

async function handleStopSession() {
  if (!store.connection.online) {
    store.toast.error('未连接：无法停止会话');
    return;
  }
  const ok = await store.modals.openConfirm({
    title: '停止会话',
    message: '将停止托管 bringup 进程，建图/导航会中断，机器人将停止运动。\n确定停止当前会话吗？',
    okText: '停止会话',
    danger: true,
  });
  if (!ok) return;
  await withPending('session-stop', async () => {
    try {
      store.teleop?.stop();
      await store.api.stopSession();
      store.toast.success('已请求停止会话…');
      addLocalLog('info', '请求停止会话');
    } catch (error) {
      reportApiError(error, '停止会话');
    }
  });
}

async function handleEstopClick() {
  if (!store.connection.online) {
    store.toast.error('未连接：无法操作急停');
    return;
  }
  const state = store.state;
  const active = Boolean(state && state.estop && state.estop.active);
  if (!active) {
    await withPending('estop', async () => {
      try {
        await store.api.estop(true);
        store.teleop?.stop();
        store.toast.error('已急停（锁存）');
        addLocalLog('warn', '已触发软件急停');
        renderHeaderState();
      } catch (error) {
        reportApiError(error, '急停');
      }
    });
    return;
  }
  const ok = await store.modals.openConfirm({
    title: '解除急停',
    message: '解除急停会恢复驱动与遥控能力。\n请确认机器人周围安全、无人靠近。\n\n确认解除急停吗？',
    okText: '确认解除',
    danger: true,
  });
  if (!ok) return;
  await withPending('estop', async () => {
    try {
      await store.api.estop(false);
      store.toast.warn('急停已解除，请谨慎操作');
      addLocalLog('warn', '急停已解除');
      renderHeaderState();
    } catch (error) {
      reportApiError(error, '解除急停');
    }
  });
}

function openSaveMapDialog() {
  const state = store.state;
  if (!store.connection.online || !state || !isOperational(state.session)) {
    store.toast.warn('仅在会话运行时可保存地图');
    return;
  }
  const nameInput = $el('save-map-name');
  if (nameInput) nameInput.value = defaultMapName();
  setText($el('save-map-hint'), '名称仅允许字母/数字/下划线/点/短横线（1–40 字符）');
  store.modals.open($el('dialog-save-map'));
  buttonFocus('save-map-name');
}

async function confirmSaveMap(overwrite = false) {
  const nameInput = $el('save-map-name');
  const name = nameInput ? nameInput.value.trim() : '';
  if (!MAP_NAME_PATTERN.test(name)) {
    setText($el('save-map-hint'), '名称不合法：仅允许 A-Z a-z 0-9 _ . -，长度 1–40');
    store.toast.error('地图名称不合法，请检查');
    return;
  }
  if (store.pending.has('save-map')) return;
  store.pending.add('save-map');
  refreshAvailability();
  let duplicate = false;
  try {
    const result = await store.api.saveMap(name, overwrite);
    store.modals.close($el('dialog-save-map'), true);
    store.toast.success(`地图已保存：${(result && result.name) || name}`);
    addLocalLog('info', `地图已保存：${(result && result.name) || name}`);
    if (result && Number.isFinite(result.width) && Number.isFinite(result.height)) {
      addLocalLog('info', `地图尺寸 ${result.width}×${result.height}，分辨率 ${formatNumber(result.resolution, 3)} m`);
    }
    await refreshMaps({ silent: true });
  } catch (error) {
    if (error instanceof ApiError && error.code === 'map_exists') {
      duplicate = true;
    } else {
      reportApiError(error, '保存地图');
    }
  } finally {
    store.pending.delete('save-map');
    refreshAvailability();
  }
  if (!duplicate) return;
  store.modals.close($el('dialog-save-map'), false);
  const ok = await store.modals.openConfirm({
    title: '覆盖同名地图',
    message: `地图「${name}」已存在，是否覆盖？\n覆盖后旧文件将被替换。`,
    okText: '覆盖保存',
    danger: true,
  });
  if (ok) await confirmSaveMap(true);
}

async function sendGoalFromMap(goal) {
  if (!goalAllowed()) {
    store.toast.warn(goalBlockedReason());
    return;
  }
  await withPending('goal', async () => {
    try {
      await store.api.goal(goal.x, goal.y, null);
      store.toast.success('导航目标已发送');
      addLocalLog(
        'info',
        `发送导航目标：x=${formatNumber(goal.x, 2)} y=${formatNumber(goal.y, 2)}（yaw 使用当前朝向）`,
      );
      store.mapView?.setTool('pointer');
    } catch (error) {
      reportApiError(error, '发送导航目标');
    }
  });
}

async function sendInitialPose(x, y, yaw, { fromMap = false } = {}) {
  if (!initialPoseAllowed()) {
    store.toast.warn(initialPoseBlockedReason());
    return;
  }
  await withPending('initialpose', async () => {
    try {
      await store.api.initialpose(x, y, yaw);
      store.toast.success('初始位姿已发布，等待 Nav2 激活…');
      addLocalLog(
        'info',
        `发布初始位姿：x=${formatNumber(x, 2)} y=${formatNumber(y, 2)} yaw=${formatYawDegrees(yaw)}`,
      );
      store.mapView?.setInitialPose({ x, y, yaw, stamp: Date.now() / 1000 });
      if (fromMap) store.mapView?.setTool('pointer');
    } catch (error) {
      reportApiError(error, '发布初始位姿');
    }
  });
}

async function handleCancelGoal() {
  await withPending('cancel-goal', async () => {
    try {
      await store.api.cancelGoal();
      store.toast.info('已请求取消导航目标');
      addLocalLog('info', '请求取消导航目标');
    } catch (error) {
      reportApiError(error, '取消导航目标');
    }
  });
}

async function handleNavActivate() {
  await withPending('nav-activate', async () => {
    try {
      await store.api.navActivate();
      store.toast.info('已请求重新激活 Nav2…');
      addLocalLog('info', '请求重新激活 Nav2');
    } catch (error) {
      reportApiError(error, '重新激活 Nav2');
    }
  });
}

async function sendGoalFromInputs() {
  if (!goalAllowed()) {
    store.toast.warn(goalBlockedReason());
    return;
  }
  const rawX = String($el('goal-x')?.value ?? '').trim();
  const rawY = String($el('goal-y')?.value ?? '').trim();
  const yawInput = $el('goal-yaw')?.value;
  const x = Number(rawX);
  const y = Number(rawY);
  if (rawX === '' || rawY === '' || !Number.isFinite(x) || !Number.isFinite(y)) {
    store.toast.error('请输入有效的目标坐标 x / y（米）');
    return;
  }
  const yaw = yawInput === '' || yawInput === null || yawInput === undefined
    ? null
    : (Number(yawInput) * Math.PI) / 180;
  if (yaw !== null && !Number.isFinite(yaw)) {
    store.toast.error('目标朝向 yaw 不合法');
    return;
  }
  await withPending('goal', async () => {
    try {
      await store.api.goal(x, y, yaw);
      store.toast.success('导航目标已发送');
      addLocalLog('info', `数值输入发送目标：x=${x} y=${y} yaw=${yaw === null ? '当前朝向' : formatYawDegrees(yaw)}`);
    } catch (error) {
      reportApiError(error, '发送导航目标');
    }
  });
}

async function sendInitialPoseFromInputs() {
  if (!initialPoseAllowed()) {
    store.toast.warn(initialPoseBlockedReason());
    return;
  }
  const rawX = String($el('init-x')?.value ?? '').trim();
  const rawY = String($el('init-y')?.value ?? '').trim();
  const yawDeg = $el('init-yaw')?.value;
  const x = Number(rawX);
  const y = Number(rawY);
  if (rawX === '' || rawY === '' || !Number.isFinite(x) || !Number.isFinite(y)) {
    store.toast.error('请输入有效的初始位姿坐标 x / y（米）');
    return;
  }
  const yaw = Number.isFinite(Number(yawDeg)) && yawDeg !== '' ? (Number(yawDeg) * Math.PI) / 180 : 0;
  await sendInitialPose(x, y, yaw);
}

async function handlePeripheralBuzzer() {
  const state = store.state;
  const current = Boolean(state && state.peripherals && state.peripherals.buzzer);
  await withPending('buzzer', async () => {
    try {
      await store.api.buzzer(!current);
      store.toast.info(`蜂鸣器已${!current ? '开启' : '关闭'}`);
      addLocalLog('info', `蜂鸣器：${!current ? 'on' : 'off'}`);
    } catch (error) {
      reportApiError(error, '蜂鸣器');
    }
  });
}

async function handleLed(color) {
  const { r, g, b } = hexToRgb(color);
  await withPending('led', async () => {
    try {
      await store.api.led(r, g, b);
      addLocalLog('info', `RGB 灯：r=${formatNumber(r, 2)} g=${formatNumber(g, 2)} b=${formatNumber(b, 2)}`);
    } catch (error) {
      reportApiError(error, 'RGB 灯');
    }
  });
}

async function handleServoApply() {
  const pan = Number($el('servo-pan')?.value);
  const tilt = Number($el('servo-tilt')?.value);
  await withPending('servo', async () => {
    try {
      await store.api.servo(pan, tilt);
      addLocalLog('info', `云台：pan=${pan}° tilt=${tilt}°`);
    } catch (error) {
      reportApiError(error, '云台');
    }
  });
}

/* ========================================================================= */
/* 设置 / 帮助                                                               */
/* ========================================================================= */

async function openSettingsDialog() {
  syncLayerSettings(store.layerDefaults);
  await loadLocalConfig({ silent: true });
  const config = store.localConfig;
  const target = (config && config.target) || {};
  const hostInput = $el('set-pi-host');
  const portInput = $el('set-pi-port');
  if (hostInput) hostInput.value = target.host || '';
  if (portInput) portInput.value = Number.isFinite(target.port) ? String(target.port) : '';
  const tokenInput = $el('set-token');
  if (tokenInput) tokenInput.value = '';
  setText($el('settings-test-result'), '未测试');
  setTone($el('settings-test-result'), 'muted');
  renderDemoState();
  renderSettingsConfigInfo();
  store.modals.open($el('dialog-settings'));
}

function renderDemoState() {
  setHidden($el('badge-demo'), !store.demo);
  setHidden($el('settings-demo-note'), !store.demo);
  const disabled = store.demo;
  for (const id of ['set-pi-host', 'set-pi-port', 'set-token', 'btn-settings-test', 'btn-settings-save']) {
    setDisabled($el(id), disabled || store.pending.has('settings-save'));
  }
}

function renderSettingsConfigInfo() {
  const config = store.localConfig;
  const path = config && config.config_path ? config.config_path : '—';
  const target = (config && config.target) || {};
  const tokenState = config && config.token_set ? '已设置' : '未设置';
  setText(
    $el('settings-config-path'),
    `配置文件：${path}\n当前目标：${target.host || '—'}:${target.port || '—'} · token：${tokenState}`,
  );
}

async function handleSettingsTest() {
  const resultEl = $el('settings-test-result');
  setText(resultEl, '测试中…');
  setTone(resultEl, 'info');
  try {
    const ping = await store.api.localPing();
    if (ping && ping.reachable) {
      setText(resultEl, `可达 · ${formatLatency(ping.latency_ms)}`);
      setTone(resultEl, 'ok');
    } else {
      setText(resultEl, `不可达：${(ping && ping.error) || '目标无响应'}`);
      setTone(resultEl, 'error');
    }
  } catch (error) {
    setText(resultEl, `测试失败：${error.message || error}`);
    setTone(resultEl, 'error');
  }
}

async function handleSettingsSave() {
  if (store.demo) {
    store.toast.warn('演示模式下不可修改 Pi 目标地址');
    return;
  }
  const host = ($el('set-pi-host')?.value || '').trim();
  const port = Number($el('set-pi-port')?.value);
  const token = $el('set-token')?.value || '';
  if (!host) {
    store.toast.error('Pi 地址不能为空');
    return;
  }
  if (!Number.isFinite(port) || port < 1 || port > 65535) {
    store.toast.error('Pi 端口需为 1–65535 的整数');
    return;
  }
  const payload = { pi_host: host, pi_port: Math.round(port) };
  if (token !== '') payload.token = token;
  else if (!(store.localConfig && store.localConfig.token_set)) payload.token = '';
  await withPending('settings-save', async () => {
    try {
      await store.api.saveLocalConfig(payload);
      store.toast.success('设置已保存，代理目标已切换');
      addLocalLog('info', `更新 Pi 目标：${host}:${port}`);
      await loadLocalConfig({ silent: true });
      store.modals.close($el('dialog-settings'), true);
      store.connection.health = false;
      pollHealth();
    } catch (error) {
      reportApiError(error, '保存设置');
    }
  });
  renderDemoState();
}

function applySettingsLayers() {
  const layers = readLayerSettings();
  store.layerDefaults = { ...store.layerDefaults, ...layers };
  store.mapView?.setLayers(store.layerDefaults);
  persistLocalSettings();
  applyLayerCheckboxState();
}

function applyLayerCheckboxState() {
  for (const [key, id] of [
    ['grid', 'toggle-grid'],
    ['scan', 'toggle-scan'],
    ['path', 'toggle-path'],
    ['trail', 'toggle-trail'],
    ['robot', 'toggle-robot'],
  ]) {
    const btn = $el(id);
    if (!btn) continue;
    const enabled = Boolean(store.layerDefaults[key]);
    btn.classList.toggle('active', enabled);
    btn.setAttribute('aria-pressed', enabled ? 'true' : 'false');
  }
  store.mapView?.setLayers(store.layerDefaults);
}

function toggleLayer(key) {
  store.layerDefaults[key] = !store.layerDefaults[key];
  persistLocalSettings();
  applyLayerCheckboxState();
  syncLayerSettings(store.layerDefaults);
}

function openHelpDialog() {
  fillShortcutList($el('help-shortcuts'), SHORTCUTS);
  store.modals.open($el('dialog-help'));
}

/* ========================================================================= */
/* 事件绑定                                                                  */
/* ========================================================================= */

function bindHeader() {
  $el('btn-estop')?.addEventListener('click', handleEstopClick);
  $el('btn-settings')?.addEventListener('click', openSettingsDialog);
  $el('btn-help')?.addEventListener('click', openHelpDialog);
}

function bindSessionPanel() {
  $el('mode-base')?.addEventListener('click', () => handleStartSession('base'));
  $el('mode-slam')?.addEventListener('click', () => handleStartSession('slam'));
  $el('mode-nav')?.addEventListener('click', () => handleStartSession('nav'));
  $el('btn-save-map')?.addEventListener('click', openSaveMapDialog);
  $el('btn-stop-session')?.addEventListener('click', handleStopSession);
  $el('btn-refresh-maps')?.addEventListener('click', () => refreshMaps({ silent: false }));
}

function bindMapToolbar() {
  $el('tool-pointer')?.addEventListener('click', () => selectTool('pointer'));
  $el('tool-goal')?.addEventListener('click', () => selectTool('goal'));
  $el('tool-initialpose')?.addEventListener('click', () => selectTool('initialpose'));
  $el('btn-follow')?.addEventListener('click', toggleFollow);
  $el('btn-recenter')?.addEventListener('click', () => store.mapView?.recenter());
  $el('btn-fit')?.addEventListener('click', () => store.mapView?.fitToMap());
  $el('btn-zoom-in')?.addEventListener('click', () => store.mapView?.zoomBy(1.25));
  $el('btn-zoom-out')?.addEventListener('click', () => store.mapView?.zoomBy(1 / 1.25));
  $el('toggle-grid')?.addEventListener('click', () => toggleLayer('grid'));
  $el('toggle-scan')?.addEventListener('click', () => toggleLayer('scan'));
  $el('toggle-path')?.addEventListener('click', () => toggleLayer('path'));
  $el('toggle-trail')?.addEventListener('click', () => toggleLayer('trail'));
  $el('toggle-robot')?.addEventListener('click', () => toggleLayer('robot'));
  $el('btn-diag-expand')?.addEventListener('click', () => {
    store.diagExpanded = !store.diagExpanded;
    store.diagSignature = '';
    renderDiagnostics();
  });
}

function bindNavPanel() {
  $el('btn-cancel-goal')?.addEventListener('click', handleCancelGoal);
  $el('btn-nav-activate')?.addEventListener('click', handleNavActivate);
  $el('btn-set-initialpose')?.addEventListener('click', () => selectTool('initialpose'));
  $el('btn-send-goal')?.addEventListener('click', sendGoalFromInputs);
  $el('btn-send-initialpose')?.addEventListener('click', sendInitialPoseFromInputs);
}

function bindPeripherals() {
  $el('btn-buzzer')?.addEventListener('click', handlePeripheralBuzzer);
  $el('btn-led-apply')?.addEventListener('click', () => handleLed($el('led-color')?.value));
  $el('btn-led-off')?.addEventListener('click', () => handleLed('#000000'));
  $el('btn-servo-apply')?.addEventListener('click', handleServoApply);
  for (const id of ['servo-pan', 'servo-tilt']) {
    $el(id)?.addEventListener('input', () => {
      const pan = Number($el('servo-pan')?.value);
      const tilt = Number($el('servo-tilt')?.value);
      setText($el('servo-pan-value'), `${Math.round(Number.isFinite(pan) ? pan : 90)}°`);
      setText($el('servo-tilt-value'), `${Math.round(Number.isFinite(tilt) ? tilt : 45)}°`);
    });
  }
}

function bindSettingsDialog() {
  $el('btn-settings-test')?.addEventListener('click', handleSettingsTest);
  $el('btn-settings-save')?.addEventListener('click', handleSettingsSave);
  $el('btn-settings-cancel')?.addEventListener('click', () => store.modals.close($el('dialog-settings'), false));
  $el('btn-save-map-cancel')?.addEventListener('click', () => store.modals.close($el('dialog-save-map'), false));
  $el('btn-save-map-confirm')?.addEventListener('click', () => confirmSaveMap(false));
  $el('save-map-name')?.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' && !event.isComposing) {
      event.preventDefault();
      confirmSaveMap(false);
    }
  });
  $el('btn-session-logs')?.addEventListener('click', async () => {
    try {
      const payload = await store.api.sessionLogs(200);
      const lines = payload && Array.isArray(payload.logs) ? payload.logs : [];
      if (lines.length === 0) {
        store.toast.info('暂无会话日志');
        return;
      }
      store.logPanel?.addSessionLogs(lines);
      store.toast.success(`已载入 ${lines.length} 行会话日志`);
    } catch (error) {
      reportApiError(error, '读取会话日志');
    }
  });
}

function bindHelpDialog() {
  $el('btn-help-close')?.addEventListener('click', () => store.modals.close($el('dialog-help'), false));
}

function bindKeyboardShortcuts() {
  window.addEventListener('keydown', (event) => {
    if (store.modals.isOpen()) return;
    if (isTypingTarget(event.target)) return;
    if (event.ctrlKey || event.metaKey || event.altKey) return;
    const key = typeof event.key === 'string' ? event.key.toLowerCase() : '';
    switch (key) {
      case 'v':
        selectTool('pointer');
        break;
      case 'g':
        selectTool('goal');
        break;
      case 'p':
        selectTool('initialpose');
        break;
      case 'f':
        toggleFollow();
        break;
      case 'r':
        store.mapView?.recenter();
        break;
      case '0':
      case 'h':
        store.mapView?.fitToMap();
        break;
      case '+':
      case '=':
        store.mapView?.zoomBy(1.25);
        break;
      case '-':
      case '_':
        store.mapView?.zoomBy(1 / 1.25);
        break;
      case 'escape':
        if (store.mapView && store.mapView.tool !== 'pointer') {
          store.mapView.cancelTool();
          store.toast.info('已取消当前工具');
        }
        break;
      default:
        return;
    }
    event.preventDefault();
  });
}

function bindPageLifecycle() {
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible') {
      store.stream?.reconnectNow();
      pollHealth();
      if (store.connection.online) refreshSnapshot({ silent: true });
    }
  });
  window.addEventListener('online', () => {
    store.stream?.reconnectNow();
    pollHealth();
  });
  window.addEventListener('unhandledrejection', (event) => {
    const reason = event.reason;
    const message = reason && reason.message ? reason.message : String(reason || '未知错误');
    addLocalLog('error', `未处理的异常：${message}`);
  });
  window.addEventListener('error', (event) => {
    if (!event || !event.message) return;
    addLocalLog('error', `脚本错误：${event.message}`);
  });
}

function isTypingTarget(target) {
  if (!target || !(target instanceof HTMLElement)) return false;
  const tag = target.tagName;
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || target.isContentEditable;
}

function buttonFocus(id) {
  requestAnimationFrame(() => {
    const el = $el(id);
    if (el && typeof el.focus === 'function') el.focus();
  });
}

/* ========================================================================= */
/* 其他                                                                      */
/* ========================================================================= */

function renderAllStatic() {
  renderConnection();
  renderHeaderState();
  renderSessionPanel();
  renderMaps();
  renderTelemetry();
  renderNavPanel();
  renderDiagnostics();
  renderPeripherals();
  renderMapChips();
  renderTeleopStatus();
  syncToolUI('pointer');
  syncFollowUI(false);
  refreshAvailability();
  updateToolHint();
  updateZoomReadout(store.mapView ? store.mapView.view : null);
}

/* 捕获 boot 期间的致命错误，避免白屏无提示 */
try {
  boot();
} catch (error) {
  console.error('[RASPBOT console] boot failed', error);
  const banner = $el('error-banner');
  const text = $el('error-banner-text');
  if (banner && text) {
    banner.classList.remove('hidden');
    text.textContent = `控制台初始化失败：${error && error.message ? error.message : error}`;
  }
}

/* 调试钩子：浏览器 DevTools 与离线冒烟测试可读取 store（不暴露任何控制接口） */
if (typeof window !== 'undefined') {
  window.__RASPBOT_CONSOLE__ = { store };
}
