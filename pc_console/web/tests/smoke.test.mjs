/**
 * smoke.test.mjs —— 无浏览器的启动/渲染/交互链路冒烟测试。
 *
 * 用最小 DOM / fetch / EventSource / canvas 桩在 Node 中真实执行 `js/app.js`：
 *   - boot 不抛异常；
 *   - 契约 state 快照能渲染到顶栏/遥测/导航/诊断/地图 HUD；
 *   - SSE 事件（state/session/map/scan/path/log）驱动 UI；
 *   - 目的地/初始位姿/急停/遥控键盘（20 Hz 发送 + 松手即停）等动作链路可用；
 *   - 断连/急停等安全禁用逻辑生效。
 *
 * 该测试不依赖浏览器、无头 Chrome 或 C2 服务，可在 `node --test` 中离线运行。
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const TEST_DIR = dirname(fileURLToPath(import.meta.url));
const WEB_DIR = resolve(TEST_DIR, '..');
const readFixture = (name) => JSON.parse(readFileSync(join(TEST_DIR, 'fixtures', name), 'utf8'));
const stateFixture = readFixture('state.nav.json');
const scanFixture = readFixture('scan.json');
const pathFixture = readFixture('path.json');
const mapsFixture = readFixture('maps.json');

const wait = (ms) => new Promise((resolvePromise) => setTimeout(resolvePromise, ms));

/* ========================================================================= */
/* 最小 DOM 桩                                                               */
/* ========================================================================= */

const allListeners = new Map();

class ClassListStub {
  constructor(element) {
    this.element = element;
  }
  add(...names) {
    for (const name of names) this.element._classes.add(name);
  }
  remove(...names) {
    for (const name of names) this.element._classes.delete(name);
  }
  toggle(name, force) {
    const shouldAdd = force === undefined ? !this.element._classes.has(name) : Boolean(force);
    if (shouldAdd) this.element._classes.add(name);
    else this.element._classes.delete(name);
    return shouldAdd;
  }
  contains(name) {
    return this.element._classes.has(name);
  }
}

let elementUid = 0;

class ElementStub {
  constructor(tag = 'div', id = '') {
    this.tagName = String(tag).toUpperCase();
    this.id = id;
    this._uid = (elementUid += 1);
    this._classes = new Set();
    this.classList = new ClassListStub(this);
    this._attrs = new Map();
    this.children = [];
    this.firstElementChild = null;
    this._text = '';
    this.value = '';
    this.checked = false;
    this.disabled = false;
    this.style = {};
    this.dataset = {};
    this.isConnected = true;
    this.offsetParent = {};
    this.width = 800;
    this.height = 500;
    this._ctx = null;
  }
  get textContent() {
    return this._text;
  }
  set textContent(value) {
    this._text = String(value);
    this.children = [];
    this.firstElementChild = null;
  }
  setAttribute(name, value) {
    this._attrs.set(name, String(value));
    if (name === 'class') {
      this._classes = new Set(String(value).split(/\s+/).filter(Boolean));
    }
  }
  getAttribute(name) {
    return this._attrs.has(name) ? this._attrs.get(name) : null;
  }
  removeAttribute(name) {
    this._attrs.delete(name);
  }
  hasAttribute(name) {
    return this._attrs.has(name);
  }
  addEventListener(type, handler) {
    const key = `el:${this._uid}:${type}`;
    if (!allListeners.has(key)) allListeners.set(key, []);
    allListeners.get(key).push(handler);
  }
  removeEventListener() {}
  dispatch(type, event = {}) {
    const key = `el:${this._uid}:${type}`;
    for (const handler of [...(allListeners.get(key) || [])]) handler(event);
  }
  append(...nodes) {
    for (const node of nodes) {
      if (typeof node === 'string') this._text += node;
      else this.appendChild(node);
    }
  }
  appendChild(node) {
    this.children.push(node);
    if (!this.firstElementChild) this.firstElementChild = node;
    node.parentElement = this;
    return node;
  }
  remove() {
    if (this.parentElement && this.parentElement.children) {
      const index = this.parentElement.children.indexOf(this);
      if (index >= 0) this.parentElement.children.splice(index, 1);
      this.parentElement.firstElementChild = this.parentElement.children[0] || null;
    }
    this.isConnected = false;
  }
  replaceWith(node) {
    this.replacedBy = node;
  }
  focus() {
    documentStub.activeElement = this;
  }
  blur() {
    documentStub.activeElement = documentStub.body;
  }
  getBoundingClientRect() {
    return { left: 0, top: 0, width: 800, height: 500, right: 800, bottom: 500 };
  }
  querySelector() {
    return null;
  }
  querySelectorAll() {
    return [];
  }
  closest() {
    return null;
  }
  setPointerCapture() {}
  releasePointerCapture() {}
  getContext() {
    if (!this._ctx) {
      const target = {};
      this._ctx = new Proxy(target, {
        get(object, property) {
          if (property === 'measureText') return () => ({ width: 30 });
          if (property in object) return object[property];
          return () => {};
        },
        set(object, property, value) {
          object[property] = value;
          return true;
        },
      });
    }
    return this._ctx;
  }
}

const htmlSource = readFileSync(join(WEB_DIR, 'index.html'), 'utf8');
const htmlIds = [...htmlSource.matchAll(/\bid\s*=\s*["']([^"']+)["']/g)].map((match) => match[1]);
const dialogIds = htmlIds.filter((id) => id.startsWith('dialog-'));
const canvasIds = new Set(['map-canvas']);
const valueIds = new Set([
  'led-color', 'servo-pan', 'servo-tilt', 'speed-slider', 'goal-x', 'goal-y', 'goal-yaw',
  'init-x', 'init-y', 'init-yaw', 'set-pi-host', 'set-pi-port', 'set-token', 'save-map-name', 'log-filter',
]);
const checkedIds = new Set(['set-layer-grid', 'set-layer-scan', 'set-layer-path', 'set-layer-trail']);

const elementCache = new Map();
for (const id of htmlIds) {
  const tag = canvasIds.has(id)
    ? 'canvas'
    : id === 'log-filter'
      ? 'select'
      : valueIds.has(id) || checkedIds.has(id)
        ? 'input'
        : 'div';
  const element = new ElementStub(tag, id);
  if (valueIds.has(id)) element.value = '';
  if (checkedIds.has(id)) element.checked = true;
  if (dialogIds.includes(id)) element.setAttribute('data-dialog', '');
  elementCache.set(id, element);
}

const documentStub = {
  activeElement: null,
  visibilityState: 'visible',
  body: new ElementStub('body'),
  getElementById(id) {
    return elementCache.get(id) || null;
  },
  createElement(tag) {
    return new ElementStub(tag);
  },
  createElementNS(_namespace, tag) {
    return new ElementStub(tag);
  },
  addEventListener(type, handler) {
    const key = `document:${type}`;
    if (!allListeners.has(key)) allListeners.set(key, []);
    allListeners.get(key).push(handler);
  },
  removeEventListener() {},
  querySelectorAll(selector) {
    if (selector === '[data-dialog]') return dialogIds.map((id) => elementCache.get(id));
    return [];
  },
  querySelector() {
    return null;
  },
};

const windowStub = {
  devicePixelRatio: 2,
  addEventListener(type, handler) {
    const key = `window:${type}`;
    if (!allListeners.has(key)) allListeners.set(key, []);
    allListeners.get(key).push(handler);
  },
  removeEventListener() {},
  requestAnimationFrame: (callback) => setTimeout(() => callback(performance.now()), 16),
  cancelAnimationFrame: (id) => clearTimeout(id),
};

class FakeImage {
  set src(value) {
    this._src = value;
    setTimeout(() => {
      if (typeof this.onload === 'function') this.onload();
    }, 0);
  }
  get src() {
    return this._src;
  }
}

class FakeEventSource {
  constructor(url) {
    this.url = url;
    this.handlers = new Map();
    this.onopen = null;
    this.onerror = null;
    setTimeout(() => {
      if (typeof this.onopen === 'function') this.onopen();
    }, 0);
  }
  addEventListener(type, handler) {
    if (!this.handlers.has(type)) this.handlers.set(type, []);
    this.handlers.get(type).push(handler);
  }
  removeEventListener() {}
  close() {
    this.closed = true;
  }
  emit(type, data) {
    const event = { type, data: JSON.stringify(data) };
    for (const handler of this.handlers.get(type) || []) handler(event);
  }
}

class FakeResizeObserver {
  constructor(callback) {
    this.callback = callback;
  }
  observe() {
    setTimeout(() => this.callback([]), 0);
  }
  disconnect() {}
}

const storage = new Map();
const localStorageStub = {
  getItem: (key) => (storage.has(key) ? storage.get(key) : null),
  setItem: (key, value) => storage.set(key, String(value)),
  removeItem: (key) => storage.delete(key),
};

const fetchCalls = [];
let healthOnline = true;
function jsonResponse(payload, status = 200) {
  return {
    ok: status >= 200 && status < 300,
    status,
    headers: { get: () => 'application/json; charset=utf-8' },
    json: async () => payload,
  };
}

async function fetchStub(url, options = {}) {
  const method = options.method || 'GET';
  const path = String(url).split('?')[0];
  fetchCalls.push({ url: String(url), path, method, body: options.body });
  if (path === '/api/health') {
    if (!healthOnline) {
      return jsonResponse({ error: { code: 'upstream_unreachable', message: 'down' } }, 502);
    }
    return jsonResponse({ ok: true, name: 'raspbot-console', version: '1.0.0', api: 1, server_time: 1, uptime_s: 1 });
  }
  if (path === '/api/state') return jsonResponse(stateFixture);
  if (path === '/api/maps') return jsonResponse(mapsFixture);
  if (path === '/local/config') {
    return jsonResponse({
      demo: true,
      version: '1.0.0',
      web: { host: '127.0.0.1', port: 8090 },
      target: { host: '192.168.50.98', port: 8080 },
      token_set: false,
      config_path: '/tmp/pc_console/config.json',
    });
  }
  if (path === '/local/ping') return jsonResponse({ reachable: true, latency_ms: 3.2, target: {}, error: null });
  if (path === '/api/session/logs') return jsonResponse({ logs: [{ ts: 1764012345, line: 'bringup started' }] });
  if (method === 'POST') return jsonResponse({ ok: true, name: 'demo_map' });
  return jsonResponse({ error: { code: 'not_found', message: 'not found' } }, 404);
}

/* 安装全局桩（必须在 import app.js 之前） */
globalThis.window = windowStub;
globalThis.document = documentStub;
globalThis.localStorage = localStorageStub;
globalThis.Image = FakeImage;
globalThis.EventSource = FakeEventSource;
globalThis.ResizeObserver = FakeResizeObserver;
globalThis.HTMLElement = ElementStub;
globalThis.Element = ElementStub;
globalThis.fetch = fetchStub;
globalThis.requestAnimationFrame = windowStub.requestAnimationFrame;
globalThis.cancelAnimationFrame = windowStub.cancelAnimationFrame;
documentStub.activeElement = documentStub.body;

const runtimeErrors = [];
process.on('uncaughtException', (error) => runtimeErrors.push(error));
process.on('unhandledRejection', (error) => runtimeErrors.push(error));

function byId(id) {
  return elementCache.get(id);
}

function hasClass(id, className) {
  return byId(id)._classes.has(className);
}

function dispatchWindowKey(type, key, extra = {}) {
  const event = {
    key,
    isComposing: false,
    repeat: false,
    ctrlKey: false,
    metaKey: false,
    altKey: false,
    shiftKey: false,
    target: documentStub.body,
    preventDefault() {},
    ...extra,
  };
  for (const handler of allListeners.get(`window:${type}`) || []) handler(event);
}

function makeReadyState() {
  return {
    ...stateFixture,
    nav: {
      ...stateFixture.nav,
      state: 'ready',
      message: '导航就绪',
      goal: null,
      goal_state: 'idle',
      distance_remaining: null,
      navigation_time_s: null,
      recoveries: 0,
    },
  };
}

/* ========================================================================= */
/* 冒烟测试                                                                  */
/* ========================================================================= */

function cleanupApp() {
  const store = globalThis.window && globalThis.window.__RASPBOT_CONSOLE__
    ? globalThis.window.__RASPBOT_CONSOLE__.store
    : null;
  if (!store) return;
  try {
    store.stream?.stop();
  } catch {
    /* 忽略清理异常 */
  }
  try {
    store.mapView?.destroy();
  } catch {
    /* 忽略清理异常 */
  }
  try {
    store.teleop?.destroy();
  } catch {
    /* 忽略清理异常 */
  }
  clearInterval(store.healthTimer);
  clearInterval(store.clockTimer);
}

test('app.js 在最小 DOM 桩下启动、渲染契约状态并跑通交互链路', async () => {
  try {
    await import('../js/app.js');
    await wait(300);

    const store = globalThis.window.__RASPBOT_CONSOLE__.store;
  assert.ok(store, 'app.js 未暴露调试 store');
  const source = store.stream && store.stream.source;
  assert.ok(source, 'SSE 未建立');

  assert.deepEqual(
    runtimeErrors.map((error) => String(error && error.message ? error.message : error)),
    [],
    '启动期间出现未捕获异常',
  );

  /* ---- 1. 初始快照渲染（fixture: nav=goal_active） ---- */
  assert.equal(byId('conn-text').textContent, '已连接');
  assert.match(byId('latency-text').textContent, /ms/);
  assert.equal(byId('tele-x').textContent, '1.23 m');
  assert.equal(byId('tele-y').textContent, '-0.40 m');
  assert.equal(byId('tele-yaw').textContent, '90.0°');
  assert.equal(byId('tele-vx').textContent, '0.12 m/s');
  assert.equal(byId('tele-scan-front').textContent, '1.35 m');
  assert.match(byId('tele-frame').textContent, /map/);
  assert.match(byId('nav-state-text').textContent, /前往目的地：剩余 2\.31 m \/ 已用 43 s/);
  assert.equal(byId('nav-distance').textContent, '2.31 m');
  assert.equal(byId('nav-time').textContent, '43 s');
  assert.equal(byId('nav-recoveries').textContent, '1');
  assert.equal(byId('diag-ok').textContent, '8');
  assert.equal(byId('session-state-badge').textContent, '运行中');
  assert.equal(byId('map-version-chip').textContent, 'v12');
  assert.match(byId('map-size-chip').textContent, /240×160/);
  assert.ok(!hasClass('badge-demo', 'hidden'), '演示模式徽章未显示');
  assert.ok(!hasClass('badge-mode', 'hidden'));
  assert.equal(byId('btn-estop').getAttribute('aria-pressed'), 'false');
  assert.equal(byId('btn-estop').disabled, false);
  assert.equal(byId('teleop-status-text').textContent, '遥控就绪');
  assert.match(byId('speed-value').textContent, /^0\.18 m\/s$/);
  assert.equal(byId('map-list').children.length, 2, '地图列表未渲染');
  assert.ok(hasClass('map-list-empty', 'hidden'), '有地图时「暂无地图」提示应隐藏');
  assert.equal(byId('log-count').textContent, '3', 'state.logs 未进入事件日志');
  assert.equal(byId('tool-initialpose').disabled, false);
  assert.equal(byId('btn-save-map').disabled, false);
  assert.equal(store.connection.sse, true);

  /* ---- 2. 导航就绪：目的地工具解锁、取消目标禁用 ---- */
  source.emit('state', { state: makeReadyState() });
  await wait(40);
  assert.equal(byId('nav-state-text').textContent, '导航就绪，可点击地图设置目的地');
  assert.equal(byId('tool-goal').disabled, false, 'nav=ready 时目的地工具应可用');
  assert.equal(byId('btn-cancel-goal').disabled, true);
  assert.equal(byId('btn-nav-activate').disabled, true);

  /* ---- 3. 急停：锁存、横幅、危险操作禁用 ---- */
  const estopState = makeReadyState();
  estopState.estop = { active: true, since: 1764012400.0 };
  source.emit('state', { state: estopState });
  await wait(40);
  assert.equal(byId('btn-estop').getAttribute('aria-pressed'), 'true');
  assert.equal(byId('estop-text').textContent, '已急停');
  assert.ok(!hasClass('estop-banner', 'hidden'), '急停横幅未显示');
  assert.equal(byId('tool-goal').disabled, true, '急停时应禁用目的地');
  assert.equal(byId('mode-nav').disabled, true, '急停时应禁用会话启动');
  assert.match(byId('teleop-status-text').textContent, /急停/);
  assert.equal(store.teleop.enabled, false);

  /* ---- 4. 恢复非急停/导航就绪，键盘遥控 20 Hz 发送 + 松手即停 ---- */
  fetchCalls.length = 0;
  source.emit('state', { state: makeReadyState() });
  await wait(40);
  dispatchWindowKey('keydown', 'w');
  await wait(120);
  assert.ok(
    fetchCalls.some((call) => call.path === '/api/teleop' && call.method === 'POST'),
    '按键后未发送 /api/teleop',
  );
  const teleopCallCount = fetchCalls.filter((call) => call.path === '/api/teleop').length;
  assert.ok(teleopCallCount >= 1, '未按 20 Hz 周期发送');
  dispatchWindowKey('keydown', 'shift', { shiftKey: true });
  dispatchWindowKey('keyup', 'w');
  dispatchWindowKey('keyup', 'shift', { shiftKey: false });
  await wait(60);
  assert.ok(
    fetchCalls.some((call) => call.path === '/api/teleop/stop' && call.method === 'POST'),
    '松手后未立即发送 /api/teleop/stop',
  );
  assert.equal(store.teleop.isActive(), false);

  // 在数值输入框内按键不应触发遥控（安全）
  fetchCalls.length = 0;
  dispatchWindowKey('keydown', 'w', { target: byId('goal-x') });
  await wait(80);
  assert.equal(
    fetchCalls.filter((call) => call.path === '/api/teleop').length,
    0,
    '输入框聚焦时按键不应发送遥控',
  );
  dispatchWindowKey('keyup', 'w', { target: byId('goal-x') });

  /* ---- 5. 会话启动中：进度条 ---- */
  const startingState = makeReadyState();
  startingState.session = {
    ...startingState.session,
    mode: 'slam',
    state: 'starting',
    progress: 0.5,
    message: '正在启动 SLAM…',
    map: null,
  };
  source.emit('state', { state: startingState });
  await wait(40);
  assert.equal(byId('session-state-badge').textContent, '正在启动…');
  assert.equal(byId('session-progress-bar').style.width, '50%');
  assert.equal(byId('teleop-status-text').textContent, '会话启动中，暂不可遥控');

  /* ---- 6. 诊断 ERROR → 顶栏横幅 + 摘要 ---- */
  const errorState = makeReadyState();
  errorState.diagnostics = {
    level: 'error',
    counts: { ok: 7, warn: 0, error: 1 },
    items: [{ name: 'Nav2', level: 'error', message: 'bt_navigator 崩溃', hardware_id: 'nav2' }],
    age_s: 0.3,
  };
  source.emit('state', { state: errorState });
  await wait(40);
  assert.ok(!hasClass('error-banner', 'hidden'), 'ERROR 横幅未显示');
  assert.match(byId('error-banner-text').textContent, /Nav2：bt_navigator 崩溃/);
  assert.match(byId('diag-summary').textContent, /Nav2：bt_navigator 崩溃/);

  /* ---- 7. scan / path / log / map SSE 事件 ---- */
  source.emit('scan', scanFixture);
  source.emit('path', pathFixture);
  source.emit('log', { ts: 1764012401.5, level: 'error', msg: 'SSE 测试日志' });
  await wait(40);
  assert.ok(store.mapView.scan && store.mapView.scan.points.length === 18);
  assert.ok(store.mapView.path && store.mapView.path.points.length === 10);
  assert.ok(store.logPanel.entries.some((entry) => entry.msg.includes('SSE 测试日志')));
  assert.ok(store.logPanel.entries.some((entry) => entry.level === 'error'));

  /* ---- 8. 地图选择：点击有效地图项 ---- */
  const listEl = byId('map-list');
  const firstButton = listEl.children[0].children[0];
  firstButton.dispatch('click');
  await wait(20);
  assert.equal(store.selectedMapName, 'room');
  assert.match(byId('mode-nav-map').textContent, /room/);

  /* ---- 8b. 动作链路：发送目的地 / 急停与解除 / 启动会话 / 保存地图 / 取消目标 ---- */
  const lastBody = (path) => {
    const call = [...fetchCalls].reverse().find((item) => item.path === path && item.method === 'POST');
    return call && call.body ? JSON.parse(call.body) : null;
  };

  // 设目的地（地图工具回调 → POST /api/goal）
  source.emit('state', { state: makeReadyState() });
  await wait(40);
  store.mapView.options.onGoal({ x: 1.25, y: -0.5 });
  await wait(80);
  assert.deepEqual(lastBody('/api/goal'), { x: 1.25, y: -0.5, yaw: null }, '目的地请求体不符合契约');
  assert.equal(store.mapView.tool, 'pointer', '发送目的地后应切回指针工具');

  // 急停：点击立即锁存
  byId('btn-estop').dispatch('click');
  await wait(80);
  assert.deepEqual(lastBody('/api/estop'), { active: true }, '急停请求体不符合契约');

  // 解除急停：先弹二次确认，确认后 POST active=false
  const estopActiveState = makeReadyState();
  estopActiveState.estop = { active: true, since: 1764012400.0 };
  source.emit('state', { state: estopActiveState });
  await wait(30);
  byId('btn-estop').dispatch('click');
  await wait(30);
  assert.ok(!hasClass('dialog-confirm', 'hidden'), '解除急停未弹出二次确认');
  byId('btn-confirm-ok').dispatch('click');
  await wait(80);
  assert.deepEqual(lastBody('/api/estop'), { active: false }, '解除急停请求体不符合契约');

  // 启动会话：idle 点击建图卡 → 确认 → POST /api/session/start
  const idleState = {
    ...makeReadyState(),
    session: {
      ...stateFixture.session,
      mode: 'idle',
      state: 'idle',
      progress: 0,
      message: '空闲',
      map: null,
    },
    nav: { ...stateFixture.nav, state: 'unavailable', goal: null, goal_state: 'idle', initialpose: null },
    robot: null,
  };
  source.emit('state', { state: idleState });
  await wait(30);
  byId('mode-slam').dispatch('click');
  await wait(30);
  assert.ok(!hasClass('dialog-confirm', 'hidden'), '启动会话未弹出确认');
  byId('btn-confirm-ok').dispatch('click');
  await wait(100);
  assert.equal(lastBody('/api/session/start').mode, 'slam', '启动会话请求体缺少 mode');

  // 保存地图：会话运行中打开弹窗，填名后 POST /api/map/save
  source.emit('state', { state: makeReadyState() });
  await wait(30);
  byId('btn-save-map').dispatch('click');
  await wait(20);
  assert.ok(!hasClass('dialog-save-map', 'hidden'), '保存地图弹窗未打开');
  byId('save-map-name').value = 'smoke_map';
  byId('btn-save-map-confirm').dispatch('click');
  await wait(100);
  assert.deepEqual(lastBody('/api/map/save'), { name: 'smoke_map', overwrite: false });
  assert.ok(hasClass('dialog-save-map', 'hidden'), '保存成功后弹窗应关闭');

  // 取消目标
  source.emit('state', { state: stateFixture });
  await wait(30);
  byId('btn-cancel-goal').dispatch('click');
  await wait(80);
  assert.ok(
    fetchCalls.some((call) => call.path === '/api/goal/cancel' && call.method === 'POST'),
    '取消目标未发送 /api/goal/cancel',
  );

  /* ---- 9. 断连：水印出现、危险按钮禁用 ---- */
  const offlineSource = store.stream;
  healthOnline = false;
  store.connection.health = false;
  offlineSource._handleError();
  await wait(40);
  assert.equal(store.connection.online, false);
  assert.ok(!hasClass('map-watermark', 'hidden'), '断连水印未显示');
  assert.equal(store.teleop.enabled, false);
  assert.equal(byId('tool-goal').disabled, true);
  assert.equal(byId('btn-stop-session').disabled, true);
  assert.equal(byId('btn-estop').disabled, true);
  assert.equal(byId('conn-text').textContent, '未连接：无法访问 Pi 网关');
  assert.equal(runtimeErrors.length, 0, runtimeErrors.map((error) => String(error)).join('; '));
  } finally {
    // 无论断言是否失败都停止所有 timer / rAF，避免测试进程挂起
    cleanupApp();
  }
});
