/**
 * capture_screens.js —— 用 Edge/Chrome headless + CDP 采集 PC 上位机演示界面截图。
 *
 * 为什么不用 `--screenshot`：页面保持 SSE 长连接，headless 的“页面加载完成”判定会被
 * 长连接拖住，`--screenshot` 可能永远不出图。本脚本通过 DevTools Protocol 显式导航、
 * 等待、`Page.captureScreenshot`，并同时收集 JS 异常/控制台错误。
 *
 * 前置：演示服务已启动（python pc_console/main.py --demo --port 8099 --no-browser）。
 *
 * 用法：
 *   node tools/console/capture_screens.js \
 *     --base http://127.0.0.1:8099 --out docs/evidence \
 *     [--nav-map demo_home] [--initialpose 1.5,1.5,0] [--goal 2.5,1.5,0]
 *
 * 产出：pc_console_00_idle.png / 01_mapping.png / 02_nav_ready.png /
 *       03_navigating.png / 04_estop.png + capture_screens_report.json
 */

'use strict';

const fs = require('fs');
const path = require('path');
const os = require('os');
const { spawn } = require('child_process');

const EDGE_CANDIDATES = [
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',
  'C:\\Program Files (x86)\\Google\\Chrome\\Application\\chrome.exe',
  '/usr/bin/google-chrome', '/usr/bin/chromium', '/usr/bin/chromium-browser',
];

function parseArgs(argv) {
  const args = {
    base: 'http://127.0.0.1:8099',
    out: 'docs/evidence',
    edge: '',
    width: 1600,
    height: 1000,
    navMap: 'demo_home',
    initialpose: [1.5, 1.5, 0],
    goal: [2.5, 1.5, 0],
    keepRunning: false,
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--base') args.base = next();
    else if (a === '--out') args.out = next();
    else if (a === '--edge') args.edge = next();
    else if (a === '--width') args.width = parseInt(next(), 10);
    else if (a === '--height') args.height = parseInt(next(), 10);
    else if (a === '--nav-map') args.navMap = next();
    else if (a === '--initialpose') args.initialpose = next().split(',').map(Number);
    else if (a === '--goal') args.goal = next().split(',').map(Number);
    else if (a === '--keep-running') args.keepRunning = true;
    else throw new Error('未知参数: ' + a);
  }
  return args;
}

function findBrowser(explicit) {
  if (explicit) {
    if (!fs.existsSync(explicit)) throw new Error('浏览器不存在: ' + explicit);
    return explicit;
  }
  if (process.env.RASPBOT_BROWSER && fs.existsSync(process.env.RASPBOT_BROWSER)) {
    return process.env.RASPBOT_BROWSER;
  }
  for (const p of EDGE_CANDIDATES) if (fs.existsSync(p)) return p;
  throw new Error('未找到 Edge/Chrome，请用 --edge 指定');
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function apiFetch(base, method, p, body) {
  const opts = { method, headers: { 'Content-Type': 'application/json' } };
  if (body !== undefined) opts.body = JSON.stringify(body);
  const res = await fetch(base + p, opts);
  let json = null;
  try { json = await res.json(); } catch (_) { /* ignore */ }
  return { status: res.status, body: json };
}

async function waitState(base, predicate, label, timeoutMs = 90000) {
  const t0 = Date.now();
  let last = null;
  while (Date.now() - t0 < timeoutMs) {
    const { status, body } = await apiFetch(base, 'GET', '/api/state');
    if (status === 200 && body) {
      last = body;
      try { if (predicate(body)) return body; } catch (_) { /* ignore */ }
    }
    await sleep(300);
  }
  throw new Error('等待超时: ' + label + '；最后状态=' +
    JSON.stringify(last && last.session) + ' nav=' + JSON.stringify(last && last.nav));
}

class Cdp {
  constructor(wsUrl) {
    this.ws = new WebSocket(wsUrl);
    this.nextId = 1;
    this.pending = new Map();
    this.events = [];
    this.ws.addEventListener('message', (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.id && this.pending.has(msg.id)) {
        const { resolve, reject } = this.pending.get(msg.id);
        this.pending.delete(msg.id);
        if (msg.error) reject(new Error(JSON.stringify(msg.error)));
        else resolve(msg.result);
      } else if (msg.method) {
        this.events.push(msg);
        if (this.onEvent) this.onEvent(msg);
      }
    });
    this.ready = new Promise((resolve, reject) => {
      this.ws.addEventListener('open', resolve);
      this.ws.addEventListener('error', (e) => reject(new Error('CDP WebSocket 错误')));
    });
  }

  send(method, params = {}) {
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
    });
  }

  close() { try { this.ws.close(); } catch (_) { /* ignore */ } }
}

async function main() {
  const args = parseArgs(process.argv);
  const browser = findBrowser(args.edge);
  fs.mkdirSync(args.out, { recursive: true });
  const report = { base: args.base, browser, shots: [], errors: [], notes: [] };
  const log = (m) => { console.log(m); report.notes.push(m); };

  const port = 9333;
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'rbt_cdp_'));
  const child = spawn(browser, [
    '--headless=new', '--disable-gpu', '--hide-scrollbars',
    '--no-first-run', '--no-default-browser-check',
    '--remote-debugging-port=' + port,
    '--user-data-dir=' + profile,
    '--window-size=' + args.width + ',' + args.height,
    'about:blank',
  ], { stdio: 'ignore' });

  let cdp = null;
  try {
    // 等 DevTools HTTP 端就绪
    let version = null;
    for (let i = 0; i < 80; i++) {
      try {
        const r = await fetch('http://127.0.0.1:' + port + '/json/version');
        if (r.ok) { version = await r.json(); break; }
      } catch (_) { /* retry */ }
      await sleep(250);
    }
    if (!version) throw new Error('DevTools 端口未就绪');

    // 找到 about:blank 页 target
    let target = null;
    for (let i = 0; i < 40; i++) {
      const list = await (await fetch('http://127.0.0.1:' + port + '/json/list')).json();
      target = list.find((t) => t.type === 'page');
      if (target && target.webSocketDebuggerUrl) break;
      await sleep(200);
    }
    if (!target) throw new Error('找不到页面 target');

    cdp = new Cdp(target.webSocketDebuggerUrl);
    await cdp.ready;
    cdp.onEvent = (msg) => {
      if (msg.method === 'Runtime.exceptionThrown') {
        report.errors.push({ type: 'exception', detail: msg.params.exceptionDetails.text });
      } else if (msg.method === 'Log.entryAdded' && msg.params.entry.level === 'error') {
        report.errors.push({ type: 'log', detail: msg.params.entry.text });
      } else if (msg.method === 'Runtime.consoleAPICalled' && msg.params.type === 'error') {
        report.errors.push({ type: 'console', detail: (msg.params.args || []).map((a) => a.value || a.description || '').join(' ') });
      }
    };
    await cdp.send('Page.enable');
    await cdp.send('Runtime.enable');
    await cdp.send('Log.enable');
    await cdp.send('Emulation.setDeviceMetricsOverride', {
      width: args.width, height: args.height, deviceScaleFactor: 1, mobile: false,
    });

    const navigate = async () => {
      await cdp.send('Page.navigate', { url: args.base + '/' });
      await sleep(3000);
      const res = await cdp.send('Runtime.evaluate', {
        expression: 'document.readyState + "|" + document.body.innerText.length + "|" + document.querySelectorAll("canvas").length',
        returnByValue: true,
      });
      const val = String((res.result && res.result.value) || '');
      log('页面状态(readyState|文本长度|canvas 数): ' + val);
      if (!val.startsWith('complete|') || parseInt(val.split('|')[1], 10) < 50) {
        throw new Error('页面疑似未渲染: ' + val);
      }
    };

    const shot = async (name) => {
      const file = path.join(args.out, name);
      const res = await cdp.send('Page.captureScreenshot', { format: 'png', fromSurface: true });
      fs.writeFileSync(file, Buffer.from(res.data, 'base64'));
      const size = fs.statSync(file).size;
      report.shots.push({ file, ok: size > 20000, size });
      log((size > 20000 ? '[OK] ' : '[FAIL] ') + name + ' —— ' + Math.round(size / 1024) + ' KB');
      return size > 20000;
    };

    await navigate();

    // ---- 0. idle ----
    await apiFetch(args.base, 'POST', '/api/session/stop', {});
    await waitState(args.base, (s) => s.session && s.session.state === 'idle', 'idle', 90000);
    await sleep(1200);
    await shot('pc_console_00_idle.png');

    // ---- 1. 建图（遥控探测揭示地图） ----
    const start = await apiFetch(args.base, 'POST', '/api/session/start', { mode: 'slam' });
    if (start.status !== 200 && start.status !== 202) throw new Error('启动建图失败: ' + JSON.stringify(start));
    await waitState(args.base, (s) => s.session && s.session.state === 'running', 'slam running', 120000);
    await waitState(args.base, (s) => s.map && s.map.available, '首帧地图', 90000);
    const tEnd = Date.now() + 3000;
    let seq = 0;
    while (Date.now() < tEnd) {
      await apiFetch(args.base, 'POST', '/api/teleop', {
        vx: 0.12, vy: 0.0, wz: 0.25, seq: seq++,
      });
      await sleep(100);
    }
    await apiFetch(args.base, 'POST', '/api/teleop/stop', {});
    await sleep(1200);
    await shot('pc_console_01_mapping.png');
    const saveName = 'evidence_' + Date.now();
    const saved = await apiFetch(args.base, 'POST', '/api/map/save', { name: saveName, overwrite: true });
    log('保存地图: ' + saveName + ' -> ' + JSON.stringify(saved.body && (saved.body.yaml || saved.body.path)));

    // ---- 2. 导航 ----
    await apiFetch(args.base, 'POST', '/api/session/stop', {});
    await waitState(args.base, (s) => s.session && s.session.state === 'idle', '停止建图', 90000);
    const maps = await apiFetch(args.base, 'GET', '/api/maps');
    const list = (maps.body && maps.body.maps) || [];
    let navMap = null;
    for (const m of list) {
      if (m.name === args.navMap && m.valid !== false) { navMap = m.path; break; }
    }
    if (!navMap) navMap = saved.body && (saved.body.yaml || saved.body.path);
    if (!navMap) throw new Error('找不到导航地图（demo_home）');
    log('导航地图: ' + navMap);
    const navStart = await apiFetch(args.base, 'POST', '/api/session/start', { mode: 'nav', map: navMap });
    if (navStart.status !== 200 && navStart.status !== 202) throw new Error('启动导航失败: ' + JSON.stringify(navStart));
    await waitState(args.base, (s) => s.session && s.session.state === 'running', 'nav running', 120000);
    await waitState(args.base, (s) => s.nav && ['waiting_initialpose', 'activating', 'ready'].includes(s.nav.state), '导航入口', 120000);
    await apiFetch(args.base, 'POST', '/api/initialpose', {
      x: args.initialpose[0], y: args.initialpose[1], yaw: args.initialpose[2] || 0,
    });
    await waitState(args.base, (s) => s.nav && s.nav.state === 'ready', 'Nav2 ready', 180000);
    await sleep(1500);
    await shot('pc_console_02_nav_ready.png');

    const goal = await apiFetch(args.base, 'POST', '/api/goal', {
      x: args.goal[0], y: args.goal[1], yaw: args.goal[2] || 0,
    });
    log('发送目标: ' + JSON.stringify(goal.body && goal.body.goal) + ' HTTP ' + goal.status);
    await waitState(args.base, (s) => s.nav && ['goal_active', 'succeeded', 'failed'].includes(s.nav.state), '目标执行', 60000);
    await sleep(2000);
    await shot('pc_console_03_navigating.png');

    // ---- 3. 急停 ----
    await apiFetch(args.base, 'POST', '/api/estop', { active: true });
    await waitState(args.base, (s) => s.estop && s.estop.active, '急停回显', 10000);
    await sleep(1000);
    await shot('pc_console_04_estop.png');
    await apiFetch(args.base, 'POST', '/api/estop', { active: false });
    await waitState(args.base, (s) => s.estop && !s.estop.active, '急停解除', 10000);

    if (!args.keepRunning) {
      await apiFetch(args.base, 'POST', '/api/session/stop', {});
      await waitState(args.base, (s) => s.session && s.session.state === 'idle', '恢复 idle', 90000).catch(() => {});
    }

    report.ok = report.shots.length === 5 && report.shots.every((s) => s.ok) && report.errors.length === 0;
    const reportPath = path.join(args.out, 'capture_screens_report.json');
    fs.writeFileSync(reportPath, JSON.stringify(report, null, 2));
    console.log('\n== 截图 ' + report.shots.filter((s) => s.ok).length + '/' + report.shots.length +
      '，JS 错误 ' + report.errors.length + '，报告: ' + reportPath + ' ==');
    process.exitCode = report.ok ? 0 : 1;
  } finally {
    if (cdp) cdp.close();
    try { child.kill(); } catch (_) { /* ignore */ }
    await sleep(500);
    try { fs.rmSync(profile, { recursive: true, force: true }); } catch (_) { /* ignore */ }
  }
}

main().catch((e) => {
  console.error('[FAIL] ' + (e && e.stack || e));
  process.exit(1);
});
