/**
 * dom_contract.test.mjs —— 静态契约测试（无需浏览器）：
 *   1) 交付文件清单与 UTF-8 编码；
 *   2) HTML 引用的本地 js/css 均存在，且无任何外部 http(s) 资源 / @import；
 *   3) HTML id 唯一；label/aria 引用有效；
 *   4) JS 中 getElementById('…') / $el('…') 引用的 id 都存在于 index.html；
 *   5) 无 alert/confirm/prompt、eval、document.write、内联事件；
 *   6) JS 中出现的 /api/*、/local/* 端点严格属于设计文档 §3.3 / §4.1 白名单；
 *   7) tests/fixtures 下的 state/scan/path/maps JSON 符合契约 schema（§3.1/§3.2/§3.3.1）。
 *
 * 运行：在仓库根执行 `node --test pc_console/web/tests`
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, extname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const TEST_DIR = dirname(fileURLToPath(import.meta.url));
const WEB_DIR = resolve(TEST_DIR, '..');
const HTML_PATH = join(WEB_DIR, 'index.html');
const JS_DIR = join(WEB_DIR, 'js');
const CSS_DIR = join(WEB_DIR, 'css');
const FIXTURE_DIR = join(TEST_DIR, 'fixtures');

const readText = (path) => readFileSync(path, 'utf8');
const html = readText(HTML_PATH);

function listFiles(dir) {
  if (!existsSync(dir)) return [];
  return readdirSync(dir)
    .filter((name) => statSync(join(dir, name)).isFile())
    .sort();
}

const jsFiles = listFiles(JS_DIR).filter((name) => name.endsWith('.js'));
const cssFiles = listFiles(CSS_DIR).filter((name) => name.endsWith('.css'));
const jsSources = new Map(jsFiles.map((name) => [name, readText(join(JS_DIR, name))]));

function collectIds(source) {
  const ids = [];
  const regex = /\bid\s*=\s*["']([^"']+)["']/g;
  let match;
  while ((match = regex.exec(source)) !== null) ids.push(match[1]);
  return ids;
}

const htmlIds = new Set(collectIds(html));

/* ------------------------------------------------------------------------- */
/* 1. 交付文件清单与编码                                                      */
/* ------------------------------------------------------------------------- */

test('交付文件清单完整且均为 UTF-8 文本', () => {
  const required = [
    'index.html',
    'css/app.css',
    'js/app.js',
    'js/api.js',
    'js/math.js',
    'js/mapview.js',
    'js/teleop.js',
    'js/ui.js',
    'js/config.js',
    'tests/math.test.mjs',
    'tests/dom_contract.test.mjs',
    'README.md',
  ];
  for (const rel of required) {
    assert.ok(existsSync(join(WEB_DIR, rel)), `缺少交付文件：${rel}`);
  }
  const files = [
    HTML_PATH,
    ...jsFiles.map((name) => join(JS_DIR, name)),
    ...cssFiles.map((name) => join(CSS_DIR, name)),
  ];
  for (const path of files) {
    const text = readText(path);
    const name = relative(WEB_DIR, path).replace(/\\/g, '/');
    assert.ok(text.length > 0, `${name} 为空文件`);
    assert.ok(!text.includes('\uFFFD'), `${name} 不是合法 UTF-8（含替换字符）`);
  }
});

/* ------------------------------------------------------------------------- */
/* 2. HTML 引用的本地资源存在 & 无外部资源 / 无 @import                        */
/* ------------------------------------------------------------------------- */

test('index.html 引用的本地 js/css 存在且无外部 http(s) 资源', () => {
  const tagRegex = /<(script|link)\b[^>]*>/gi;
  let match;
  let moduleScriptSeen = false;
  let stylesheetSeen = false;
  while ((match = tagRegex.exec(html)) !== null) {
    const tag = match[0];
    const tagName = match[1].toLowerCase();
    const isScript = tagName === 'script';
    const attrRegex = isScript ? /\bsrc\s*=\s*["']([^"']+)["']/i : /\bhref\s*=\s*["']([^"']+)["']/i;
    const attr = attrRegex.exec(tag);
    if (!attr) {
      assert.fail(`index.html 中存在缺少 src/href 的 <${tagName}> 标签：${tag}`);
    }
    const url = attr[1].trim();
    assert.ok(!/^https?:\/\//i.test(url), `发现外部 URL 引用：${url}`);
    assert.ok(!url.startsWith('//'), `发现协议相对外部引用：${url}`);
    if (url.startsWith('#') || url.startsWith('data:')) continue;
    const clean = url.split(/[?#]/)[0];
    const target = resolve(WEB_DIR, clean);
    assert.ok(
      target.startsWith(WEB_DIR + '\\') || target.startsWith(WEB_DIR + '/'),
      `引用的资源越出 web/ 目录：${url}`,
    );
    assert.ok(existsSync(target), `引用的本地资源不存在：${url}（解析为 ${target}）`);
    if (isScript) {
      assert.match(tag, /type\s*=\s*["']module["']/i, '前端脚本必须以 type="module" 加载');
      moduleScriptSeen = true;
    } else if (/rel\s*=\s*["']stylesheet["']/i.test(tag)) {
      stylesheetSeen = true;
    }
  }
  assert.ok(moduleScriptSeen, 'index.html 缺少 type="module" 的脚本');
  assert.ok(stylesheetSeen, 'index.html 缺少样式表引用');

  for (const [name, source] of jsSources) {
    // 允许 SVG/XML 命名空间常量（不是网络资源），其余任何 http(s) 地址都禁止
    assert.ok(
      !/https?:\/\/(?!www\.w3\.org\/2000\/svg)/i.test(source),
      `js/${name} 含有外部 http(s) 地址`,
    );
    const importRegex = /\bfrom\s+["']([^"']+)["']/g;
    let importMatch;
    while ((importMatch = importRegex.exec(source)) !== null) {
      const spec = importMatch[1];
      assert.ok(spec.startsWith('./') || spec.startsWith('../'), `js/${name} 存在非相对 import：${spec}`);
      const target = resolve(JS_DIR, spec);
      assert.ok(existsSync(target), `js/${name} 的 import 目标不存在：${spec}`);
    }
  }

  for (const name of cssFiles) {
    const source = readText(join(CSS_DIR, name));
    assert.ok(!/@import/i.test(source), `css/${name} 不得使用 @import`);
    assert.ok(
      !/url\s*\(\s*['"]?\s*(?:https?:)?\/\//i.test(source),
      `css/${name} 不得引用外部/网络 URL`,
    );
  }
});

/* ------------------------------------------------------------------------- */
/* 3. HTML id 唯一 & label/aria 引用有效                                      */
/* ------------------------------------------------------------------------- */

test('index.html 中 id 唯一，label/aria 引用均已定义', () => {
  const ids = collectIds(html);
  const seen = new Set();
  for (const id of ids) {
    assert.ok(/^[A-Za-z][\w-]*$/.test(id), `id 命名不合法：${id}`);
    assert.ok(!seen.has(id), `index.html 中 id 重复：${id}`);
    seen.add(id);
  }
  assert.ok(seen.size >= 100, `index.html 的 id 数量异常偏少（${seen.size}），疑似结构缺失`);

  const forRegex = /\bfor\s*=\s*["']([^"']+)["']/g;
  let match;
  let forCount = 0;
  while ((match = forRegex.exec(html)) !== null) {
    forCount += 1;
    assert.ok(seen.has(match[1]), `label for="${match[1]}" 不存在对应元素`);
  }
  assert.ok(forCount >= 5, '表单控件缺少 label for 关联');

  const ariaRegex = /\baria-(?:labelledby|describedby|controls)\s*=\s*["']([^"']+)["']/g;
  while ((match = ariaRegex.exec(html)) !== null) {
    for (const ref of match[1].split(/\s+/)) {
      assert.ok(seen.has(ref), `aria 引用不存在的 id：${ref}`);
    }
  }

  assert.match(html, /<html\s+lang="zh-CN"/i, 'HTML 根元素应声明 lang="zh-CN"');
  assert.match(html, /<meta\s+charset="utf-8"/i, '缺少 UTF-8 charset 声明');
  assert.match(html, /<meta\s+name="viewport"/i, '缺少 viewport 声明');
  assert.match(html, /<title>[^<]+<\/title>/i, '缺少页面标题');
});

/* ------------------------------------------------------------------------- */
/* 4. JS 引用的 DOM id 必须存在                                              */
/* ------------------------------------------------------------------------- */

test('JS 中 getElementById/$el 引用的 id 都存在于 index.html', () => {
  const references = [];
  for (const [name, source] of jsSources) {
    for (const regex of [
      /getElementById\(\s*["']([^"']+)["']/g,
      /\$el\(\s*["']([^"']+)["']/g,
    ]) {
      let match;
      while ((match = regex.exec(source)) !== null) {
        references.push({ file: name, id: match[1] });
      }
    }
  }
  assert.ok(references.length >= 120, `JS 中 DOM id 引用数量异常偏少（${references.length}）`);
  const missing = references.filter((ref) => !htmlIds.has(ref.id));
  assert.deepEqual(
    missing.map((ref) => `${ref.file} -> #${ref.id}`),
    [],
    'JS 引用了 index.html 中不存在的 id',
  );

  // 关键交互元素必须在 HTML 中静态存在（不能被动态创建替代）
  const critical = [
    'map-canvas', 'map-viewport', 'btn-estop', 'btn-estop', 'conn-dot', 'conn-text', 'ros-dot',
    'mode-base', 'mode-slam', 'mode-nav', 'btn-save-map', 'btn-stop-session', 'map-list',
    'btn-buzzer', 'btn-led-apply', 'btn-servo-apply', 'joystick', 'joystick-knob', 'speed-slider',
    'speed-value', 'btn-teleop-stop', 'tool-pointer', 'tool-goal', 'tool-initialpose', 'btn-follow',
    'btn-recenter', 'btn-fit', 'btn-zoom-in', 'btn-zoom-out', 'log-list', 'log-filter',
    'btn-log-pause', 'btn-log-clear', 'btn-log-download', 'btn-session-logs', 'tele-x', 'tele-y',
    'tele-yaw', 'tele-vx', 'nav-state-text', 'nav-distance', 'nav-time', 'nav-recoveries',
    'btn-cancel-goal', 'btn-nav-activate', 'btn-set-initialpose', 'goal-x', 'goal-y', 'goal-yaw',
    'btn-send-goal', 'init-x', 'init-y', 'init-yaw', 'btn-send-initialpose', 'diag-ok', 'diag-warn',
    'diag-error', 'diag-list', 'btn-diag-expand', 'dialog-save-map', 'save-map-name',
    'btn-save-map-confirm', 'dialog-settings', 'set-pi-host', 'set-pi-port', 'set-token',
    'btn-settings-test', 'btn-settings-save', 'dialog-help', 'help-shortcuts', 'dialog-confirm',
    'confirm-title', 'confirm-message', 'btn-confirm-ok', 'btn-confirm-cancel', 'toast-root',
    'map-watermark', 'badge-demo', 'error-banner', 'estop-banner',
  ];
  for (const id of critical) {
    assert.ok(htmlIds.has(id), `关键元素缺失：#${id}`);
  }
});

/* ------------------------------------------------------------------------- */
/* 5. 禁止项：alert/confirm/prompt、eval、document.write、内联事件             */
/* ------------------------------------------------------------------------- */

test('不使用 alert/confirm/prompt/eval/document.write，无内联事件处理器', () => {
  const forbidden = [
    { regex: /\b(alert|confirm|prompt)\s*\(/i, reason: '必须使用自绘模态框，禁止 alert/confirm/prompt' },
    { regex: /\beval\s*\(/, reason: '禁止 eval' },
    { regex: /new\s+Function\s*\(/, reason: '禁止 new Function' },
    { regex: /document\s*\.\s*write\s*\(/, reason: '禁止 document.write' },
  ];
  for (const [name, source] of jsSources) {
    for (const item of forbidden) {
      assert.ok(!item.regex.test(source), `js/${name} 违反约束：${item.reason}`);
    }
  }
  const inlineHandler = /\son(?:click|dblclick|change|input|submit|load|error|keydown|keyup|mousedown|mouseup|pointerdown|pointerup)\s*=/i;
  assert.ok(!inlineHandler.test(html), 'index.html 不得使用内联事件处理器');
  assert.ok(!/<script\b(?![^>]*\bsrc\s*=)[^>]*>/i.test(html), 'index.html 不得包含内联脚本');
});

/* ------------------------------------------------------------------------- */
/* 6. API 端点白名单                                                         */
/* ------------------------------------------------------------------------- */

test('JS 中出现的 /api/* 与 /local/* 端点严格属于契约白名单', () => {
  const allowed = [
    /^\/api\/(health|state|events|map\/meta|map\.png|maps|map\/save|session\/start|session\/stop|teleop|teleop\/stop|estop|goal|goal\/cancel|initialpose|nav\/activate|actuator\/buzzer|actuator\/led|actuator\/servo|logs|session\/logs|ros)(?:[?/].*)?$/,
    /^\/local\/(config|ping)$/,
  ];
  const collected = [];
  for (const [name, source] of jsSources) {
    const regex = /(["'`])(\/(?:api|local)\/[^"'`\n]*)\1/g;
    let match;
    while ((match = regex.exec(source)) !== null) {
      const literal = match[2];
      const normalized = literal.replace(/\$\{[^}]*\}/g, '');
      collected.push({ file: name, literal, normalized });
    }
  }
  assert.ok(collected.length >= 25, `提取到的端点数量异常偏少（${collected.length}）`);
  const illegal = collected.filter((item) => !allowed.some((regex) => regex.test(item.normalized)));
  assert.deepEqual(
    illegal.map((item) => `${item.file}: ${item.literal}`),
    [],
    '发现契约之外的后端接口',
  );

  // 契约端点均应被声明（至少出现一次）
  const expected = [
    '/api/health', '/api/state', '/api/events', '/api/map/meta', '/api/map.png',
    '/api/maps', '/api/map/save', '/api/session/start', '/api/session/stop', '/api/teleop',
    '/api/teleop/stop', '/api/estop', '/api/goal', '/api/goal/cancel', '/api/initialpose',
    '/api/nav/activate', '/api/actuator/buzzer', '/api/actuator/led', '/api/actuator/servo',
    '/api/logs', '/api/session/logs', '/api/ros', '/local/config', '/local/ping',
  ];
  const normalizedSet = new Set(collected.map((item) => item.normalized));
  for (const endpoint of expected) {
    const found = [...normalizedSet].some(
      (value) => value === endpoint || value.startsWith(`${endpoint}?`) || value.startsWith(`${endpoint}/`),
    );
    assert.ok(found, `契约端点未在 js/ 中声明：${endpoint}`);
  }
});

/* ------------------------------------------------------------------------- */
/* 7. fixtures schema 校验（§3.1 / §3.2 / §3.3.1）                            */
/* ------------------------------------------------------------------------- */

function assertFiniteNumber(value, name) {
  assert.equal(typeof value, 'number', `${name} 应为 number`);
  assert.ok(Number.isFinite(value), `${name} 应为有限数值`);
}

function assertEnum(value, allowed, name) {
  assert.ok(allowed.includes(value), `${name} 取值非法：${JSON.stringify(value)}`);
}

function assertPose(pose, name, { yawNullable = false } = {}) {
  assert.equal(typeof pose, 'object', `${name} 应为对象`);
  assert.notEqual(pose, null, `${name} 不应为 null`);
  assertFiniteNumber(pose.x, `${name}.x`);
  assertFiniteNumber(pose.y, `${name}.y`);
  if (!(yawNullable && pose.yaw === null)) assertFiniteNumber(pose.yaw, `${name}.yaw`);
  assertFiniteNumber(pose.stamp, `${name}.stamp`);
}

test('fixtures：state 快照符合 §3.1 schema', () => {
  const state = JSON.parse(readText(join(FIXTURE_DIR, 'state.nav.json')));
  assertFiniteNumber(state.api, 'api');
  assertFiniteNumber(state.server_time, 'server_time');
  assertFiniteNumber(state.uptime_s, 'uptime_s');

  const session = state.session;
  assert.ok(session, '缺少 session');
  assertEnum(session.mode, ['idle', 'base', 'slam', 'nav'], 'session.mode');
  assertEnum(session.state, ['idle', 'starting', 'running', 'stopping', 'error'], 'session.state');
  for (const key of ['mock', 'managed', 'external']) {
    assert.equal(typeof session[key], 'boolean', `session.${key} 应为 boolean`);
  }
  assert.ok(Array.isArray(session.available_modes), 'session.available_modes 应为数组');
  assertFiniteNumber(session.since, 'session.since');
  assertFiniteNumber(session.progress, 'session.progress');
  assert.ok(session.progress >= 0 && session.progress <= 1, 'session.progress 应在 0..1');
  assert.equal(typeof session.message, 'string', 'session.message 应为 string');
  assert.ok(session.error === null || typeof session.error === 'string', 'session.error 类型非法');
  assert.ok(session.map === null || typeof session.map === 'string', 'session.map 类型非法');

  const ros = state.ros;
  assert.ok(ros, '缺少 ros');
  assert.equal(typeof ros.ok, 'boolean', 'ros.ok 应为 boolean');
  assertFiniteNumber(ros.nodes, 'ros.nodes');
  for (const key of ['base', 'teleop', 'map', 'localization', 'nav2', 'diagnostics']) {
    assert.equal(typeof ros.ready[key], 'boolean', `ros.ready.${key} 应为 boolean`);
  }
  for (const key of ['scan_hz', 'odom_hz', 'map_age_s', 'diagnostics_age_s']) {
    assertFiniteNumber(ros.rates[key], `ros.rates.${key}`);
  }

  for (const key of ['max_linear', 'max_lateral', 'max_angular']) {
    assertFiniteNumber(state.limits[key], `limits.${key}`);
  }

  const robot = state.robot;
  if (robot !== null) {
    assertEnum(robot.frame, ['map', 'odom'], 'robot.frame');
    for (const key of ['x', 'y', 'yaw', 'stamp']) assertFiniteNumber(robot[key], `robot.${key}`);
    for (const key of ['vx', 'vy', 'wz']) assertFiniteNumber(robot.speed[key], `robot.speed.${key}`);
    assertFiniteNumber(robot.imu.yaw_deg, 'robot.imu.yaw_deg');
    assertFiniteNumber(robot.imu.age_s, 'robot.imu.age_s');
    assertFiniteNumber(robot.scan.front_m, 'robot.scan.front_m');
    assertFiniteNumber(robot.scan.min_m, 'robot.scan.min_m');
    assertFiniteNumber(robot.scan.age_s, 'robot.scan.age_s');
  }

  const nav = state.nav;
  assert.ok(nav, '缺少 nav');
  assertEnum(
    nav.state,
    ['unavailable', 'waiting_initialpose', 'activating', 'ready', 'goal_active', 'succeeded', 'failed', 'canceled', 'error'],
    'nav.state',
  );
  assert.equal(typeof nav.message, 'string', 'nav.message 应为 string');
  if (nav.initialpose !== null) assertPose(nav.initialpose, 'nav.initialpose');
  if (nav.goal !== null) {
    assertPose(nav.goal, 'nav.goal', { yawNullable: true });
    assert.equal(typeof nav.goal.goal_id, 'string', 'nav.goal.goal_id 应为 string');
  }
  assertEnum(nav.goal_state, ['idle', 'accepted', 'active', 'succeeded', 'canceled', 'aborted'], 'nav.goal_state');
  for (const key of ['distance_remaining', 'navigation_time_s', 'recoveries']) {
    assert.ok(nav[key] === null || typeof nav[key] === 'number', `nav.${key} 类型非法`);
  }
  assert.equal(typeof nav.lifecycle, 'object', 'nav.lifecycle 应为对象');
  assertFiniteNumber(nav.activation.done, 'nav.activation.done');
  assertFiniteNumber(nav.activation.total, 'nav.activation.total');
  assert.equal(typeof nav.activation.phase, 'string', 'nav.activation.phase 应为 string');

  const map = state.map;
  assert.equal(typeof map.available, 'boolean', 'map.available 应为 boolean');
  if (map.available) {
    for (const key of ['version', 'width', 'height', 'resolution', 'stamp']) {
      assertFiniteNumber(map[key], `map.${key}`);
    }
    assert.ok(map.width > 0 && map.height > 0 && map.resolution > 0, '地图尺寸/分辨率必须为正');
    for (const key of ['x', 'y', 'yaw']) assertFiniteNumber(map.origin[key], `map.origin.${key}`);
    assert.equal(typeof map.frame_id, 'string', 'map.frame_id 应为 string');
  }

  assert.equal(typeof state.estop.active, 'boolean', 'estop.active 应为 boolean');
  assert.ok(state.estop.since === null || typeof state.estop.since === 'number', 'estop.since 类型非法');
  assert.equal(typeof state.teleop.active, 'boolean', 'teleop.active 应为 boolean');
  for (const key of ['vx', 'vy', 'wz']) assertFiniteNumber(state.teleop[key], `teleop.${key}`);

  const periph = state.peripherals;
  assert.equal(typeof periph.buzzer, 'boolean', 'peripherals.buzzer 应为 boolean');
  for (const key of ['r', 'g', 'b']) {
    assertFiniteNumber(periph.led[key], `peripherals.led.${key}`);
    assert.ok(periph.led[key] >= 0 && periph.led[key] <= 1, `peripherals.led.${key} 应在 0..1`);
  }
  assertFiniteNumber(periph.servo.pan, 'peripherals.servo.pan');
  assertFiniteNumber(periph.servo.tilt, 'peripherals.servo.tilt');
  assert.ok(periph.servo.pan >= 0 && periph.servo.pan <= 180, 'servo.pan 超出机械限位');
  assert.ok(periph.servo.tilt >= 0 && periph.servo.tilt <= 110, 'servo.tilt 超出机械限位');

  assertEnum(state.diagnostics.level, ['ok', 'warn', 'error', 'unknown'], 'diagnostics.level');
  for (const key of ['ok', 'warn', 'error']) {
    assertFiniteNumber(state.diagnostics.counts[key], `diagnostics.counts.${key}`);
  }
  assert.ok(Array.isArray(state.diagnostics.items), 'diagnostics.items 应为数组');
  assert.ok(state.diagnostics.items.length <= 12, 'diagnostics.items 最多 12 条');
  for (const item of state.diagnostics.items) {
    assertEnum(item.level, ['ok', 'warn', 'error', 'unknown'], 'diagnostics.items[].level');
    assert.equal(typeof item.name, 'string', 'diagnostics.items[].name 应为 string');
    assert.equal(typeof item.message, 'string', 'diagnostics.items[].message 应为 string');
  }
  assertFiniteNumber(state.diagnostics.age_s, 'diagnostics.age_s');

  assert.ok(Array.isArray(state.logs), 'logs 应为数组');
  for (const entry of state.logs) {
    assertFiniteNumber(entry.ts, 'logs[].ts');
    assertEnum(entry.level, ['info', 'warn', 'error'], 'logs[].level');
    assert.equal(typeof entry.msg, 'string', 'logs[].msg 应为 string');
  }

  // 几何自洽：机器人位姿应在地图范围内（该 fixture 为地图内的正常状态）
  if (map.available && robot) {
    const minX = map.origin.x;
    const maxX = map.origin.x + map.width * map.resolution;
    const minY = map.origin.y;
    const maxY = map.origin.y + map.height * map.resolution;
    assert.ok(robot.x >= minX && robot.x <= maxX, 'fixture 机器人 x 超出地图范围');
    assert.ok(robot.y >= minY && robot.y <= maxY, 'fixture 机器人 y 超出地图范围');
  }
});

test('fixtures：scan / path / maps 符合 §3.2、§3.3.1', () => {
  const scan = JSON.parse(readText(join(FIXTURE_DIR, 'scan.json')));
  assertFiniteNumber(scan.stamp, 'scan.stamp');
  assertEnum(scan.frame, ['map', 'odom'], 'scan.frame');
  assert.ok(Array.isArray(scan.points), 'scan.points 应为数组');
  assert.equal(scan.points.length % 2, 0, 'scan.points 必须为扁平坐标对');
  assert.ok(scan.points.length / 2 <= 360, 'scan 点数不得超过 360');
  assert.ok(scan.points.every((value) => typeof value === 'number' && Number.isFinite(value)), 'scan.points 含非法数值');

  const path = JSON.parse(readText(join(FIXTURE_DIR, 'path.json')));
  assertFiniteNumber(path.stamp, 'path.stamp');
  assertEnum(path.frame, ['map', 'odom'], 'path.frame');
  assert.ok(Array.isArray(path.points), 'path.points 应为数组');
  assert.equal(path.points.length % 2, 0, 'path.points 必须为扁平坐标对');
  assert.ok(path.points.length / 2 <= 200, 'path 点数不得超过 200');
  assert.ok(path.points.every((value) => typeof value === 'number' && Number.isFinite(value)), 'path.points 含非法数值');

  const maps = JSON.parse(readText(join(FIXTURE_DIR, 'maps.json')));
  assert.ok(Array.isArray(maps.maps), 'maps.maps 应为数组');
  assert.ok(Array.isArray(maps.dirs), 'maps.dirs 应为数组');
  for (const map of maps.maps) {
    assert.equal(typeof map.name, 'string', 'maps[].name 应为 string');
    assert.equal(typeof map.path, 'string', 'maps[].path 应为 string');
    assert.equal(typeof map.yaml, 'string', 'maps[].yaml 应为 string');
    assertFiniteNumber(map.mtime, 'maps[].mtime');
    assertFiniteNumber(map.resolution, 'maps[].resolution');
    assertFiniteNumber(map.width, 'maps[].width');
    assertFiniteNumber(map.height, 'maps[].height');
    assert.equal(typeof map.preview_url, 'string', 'maps[].preview_url 应为 string');
    assert.equal(typeof map.valid, 'boolean', 'maps[].valid 应为 boolean');
  }
});
