/**
 * math.test.mjs —— js/math.js 纯函数单测。
 * 运行：在仓库根执行 `node --test pc_console/web/tests`
 */
import test from 'node:test';
import assert from 'node:assert/strict';

import {
  clamp,
  normalizeAngle,
  yawToScreenAngle,
  rotateVector,
  worldToScreen,
  screenToWorld,
  zoomAt,
  panView,
  centerOn,
  makeView,
  fitView,
  imagePixelToWorld,
  worldToImagePixel,
  gridCellToWorld,
  worldToGridCell,
  mapWorldCorners,
  mapWorldBounds,
  mapMetaUsable,
  keyVelocity,
  normalizeInputVector,
  joystickVelocity,
  clampVelocity,
  chooseGridSpacing,
  chooseScaleBar,
  formatNumber,
  formatMeters,
  formatSpeed,
  formatAngularSpeed,
  formatYawDegrees,
  formatDuration,
  formatLatency,
  formatTimestamp,
  formatAge,
  formatPercent,
  distance,
  toFiniteNumber,
} from '../js/math.js';

const EPS = 1e-9;
const MAP = {
  available: true,
  version: 12,
  width: 240,
  height: 160,
  resolution: 0.05,
  origin: { x: -2.4, y: -2.0, yaw: 0 },
  frame_id: 'map',
  stamp: 1764012345.2,
};

function close(actual, expected, epsilon = 1e-6, message = '') {
  assert.ok(
    Math.abs(actual - expected) <= epsilon,
    `${message} 期望 ${expected}，实际 ${actual}`,
  );
}

test('clamp / toFiniteNumber / normalizeAngle 基础', () => {
  assert.equal(clamp(5, 0, 1), 1);
  assert.equal(clamp(-5, 0, 1), 0);
  assert.equal(clamp(0.5, 0, 1), 0.5);
  assert.equal(clamp(NaN, 2, 3), 2);
  assert.equal(toFiniteNumber('0.25'), 0.25);
  assert.equal(toFiniteNumber('abc', 7), 7);
  assert.equal(toFiniteNumber(null, 7), 7);
  close(normalizeAngle(3 * Math.PI), Math.PI, 1e-9);
  close(normalizeAngle(-3 * Math.PI), Math.PI, 1e-9);
  close(normalizeAngle(Math.PI * 2 + 0.1), 0.1, 1e-9);
  close(normalizeAngle(-0.1), -0.1, 1e-9);
});

test('世界↔屏幕：中心点与 y 轴翻转（正/负坐标）', () => {
  const view = makeView({ scale: 50, centerX: 1, centerY: 2, width: 800, height: 600 });
  // 视图中心 = 世界 (1, 2) → 屏幕 (400, 300)
  assert.deepEqual(worldToScreen(view, 1, 2), { x: 400, y: 300 });

  // 世界 y 增加（向上）→ 屏幕 y 变小
  const up = worldToScreen(view, 1, 3);
  assert.equal(up.y, 250);
  assert.equal(up.x, 400);

  // 负坐标：(-1.5, -2.5)
  const neg = worldToScreen(view, -1.5, -2.5);
  assert.equal(neg.x, 400 + (-1.5 - 1) * 50); // 275
  assert.equal(neg.y, 300 - (-2.5 - 2) * 50); // 525
  // y 翻转的符号检查
  assert.ok(worldToScreen(view, 0, 10).y < worldToScreen(view, 0, 0).y);

  // 屏幕 → 世界逆变换
  const world = screenToWorld(view, neg.x, neg.y);
  close(world.x, -1.5, 1e-9, 'x 逆变换');
  close(world.y, -2.5, 1e-9, 'y 逆变换');

  // 点击 → 世界坐标（已知点）
  assert.deepEqual(screenToWorld(view, 400, 300), { x: 1, y: 2 });
  const clicked = screenToWorld(view, 450, 250);
  close(clicked.x, 2, 1e-9);
  close(clicked.y, 3, 1e-9);

  // 双向 round-trip
  for (const [wx, wy] of [[0, 0], [-3.2, 4.7], [12.9, -8.1]]) {
    const screen = worldToScreen(view, wx, wy);
    const back = screenToWorld(view, screen.x, screen.y);
    close(back.x, wx, 1e-9);
    close(back.y, wy, 1e-9);
  }
});

test('yaw：世界 yaw → 屏幕旋转角为取负', () => {
  close(yawToScreenAngle(Math.PI / 2), -Math.PI / 2, 1e-9);
  close(yawToScreenAngle(-Math.PI / 2), Math.PI / 2, 1e-9);
  close(yawToScreenAngle(0), 0, 1e-12);

  const rotated = rotateVector(1, 0, Math.PI / 2);
  close(rotated.x, 0, 1e-9);
  close(rotated.y, 1, 1e-9);
});

test('地图 origin + resolution：图片像素 ↔ 世界 ↔ OccupancyGrid 栅格', () => {
  // 图片左上角 = 世界最大 y（§5.5），origin 为左下角
  const topLeft = imagePixelToWorld(MAP, 0, 0);
  close(topLeft.x, -2.4, 1e-9);
  close(topLeft.y, -2.0 + 160 * 0.05, 1e-9); // 6.0
  const bottomLeft = imagePixelToWorld(MAP, 0, 160);
  close(bottomLeft.x, -2.4, 1e-9);
  close(bottomLeft.y, -2.0, 1e-9);
  // 世界包围盒
  const bounds = mapWorldBounds(MAP);
  close(bounds.minX, -2.4, 1e-9);
  close(bounds.maxX, -2.4 + 240 * 0.05, 1e-9); // 9.6
  close(bounds.minY, -2.0, 1e-9);
  close(bounds.maxY, -2.0 + 160 * 0.05, 1e-9); // 6.0
  assert.equal(mapWorldCorners(MAP).length, 4);
  assert.equal(mapMetaUsable(MAP), true);
  assert.equal(mapMetaUsable(null), false);
  assert.equal(mapMetaUsable({ ...MAP, resolution: 0 }), false);

  // 世界 → 图片像素（round-trip）
  const pixel = worldToImagePixel(MAP, 1.0, 3.0);
  close(pixel.px, (1.0 + 2.4) / 0.05, 1e-9); // 68
  close(pixel.py, 160 - (3.0 + 2.0) / 0.05, 1e-9); // 60
  const back = imagePixelToWorld(MAP, pixel.px, pixel.py);
  close(back.x, 1.0, 1e-9);
  close(back.y, 3.0, 1e-9);

  // OccupancyGrid 栅格（row 0 = 世界最小 y，与图片行序相反）
  const cell = gridCellToWorld(MAP, 0, 0);
  close(cell.x, -2.4 + 0.025, 1e-9);
  close(cell.y, -2.0 + 0.025, 1e-9);
  const cellBack = worldToGridCell(MAP, cell.x, cell.y);
  assert.deepEqual(cellBack, { col: 0, row: 0 });
  assert.deepEqual(worldToGridCell(MAP, -2.39, -1.99), { col: 0, row: 0 });
  assert.deepEqual(worldToGridCell(MAP, 1.0, 3.0), {
    col: Math.floor((1.0 + 2.4) / 0.05),
    row: Math.floor((3.0 + 2.0) / 0.05),
  });
});

test('地图 origin.yaw 非零时旋转换算正确', () => {
  const rotatedMap = {
    available: true,
    width: 4,
    height: 2,
    resolution: 1,
    origin: { x: 0, y: 0, yaw: Math.PI / 2 },
  };
  // 图片像素 (0,0)：local=(0, 2) 旋转 90° → world=(-2, 0)
  const topLeft = imagePixelToWorld(rotatedMap, 0, 0);
  close(topLeft.x, -2, 1e-9);
  close(topLeft.y, 0, 1e-9);
  // 图片像素 (4,0)：local=(4,2) → world=(-2,4)
  const topRight = imagePixelToWorld(rotatedMap, 4, 0);
  close(topRight.x, -2, 1e-9);
  close(topRight.y, 4, 1e-9);
  const bounds = mapWorldBounds(rotatedMap);
  close(bounds.minX, -2, 1e-9);
  close(bounds.maxX, 0, 1e-9);
  close(bounds.minY, 0, 1e-9);
  close(bounds.maxY, 4, 1e-9);
  // round-trip
  const pixel = worldToImagePixel(rotatedMap, -2, 4);
  close(pixel.px, 4, 1e-6);
  close(pixel.py, 0, 1e-6);
});

test('视图：适配、以鼠标为中心缩放、平移、回中', () => {
  const fitted = fitView(MAP, 800, 500, 20);
  // 地图 12m × 8m，可用 760×460 → scale = min(63.33, 57.5) = 57.5
  close(fitted.scale, 57.5, 1e-9);
  close(fitted.centerX, (-2.4 + 9.6) / 2, 1e-9);
  close(fitted.centerY, (-2.0 + 6.0) / 2, 1e-9);
  assert.equal(fitted.width, 800);
  assert.equal(fitted.height, 500);

  // 空地图适配不抛错
  const emptyView = fitView(null, 800, 500);
  assert.ok(emptyView.scale > 0);

  // 缩放不动点：缩放前后鼠标下的世界坐标不变
  const view = makeView({ scale: 50, centerX: 0.5, centerY: -0.5, width: 800, height: 600 });
  const anchorBefore = screenToWorld(view, 610, 180);
  const zoomed = zoomAt(view, 1.5, 610, 180);
  const anchorAfter = screenToWorld(zoomed, 610, 180);
  close(anchorAfter.x, anchorBefore.x, 1e-6);
  close(anchorAfter.y, anchorBefore.y, 1e-6);
  close(zoomed.scale, 75, 1e-9);
  // 缩放限幅
  assert.equal(zoomAt(view, 1000, 400, 300, 4, 400).scale, 400);
  assert.equal(zoomAt(view, 0.0001, 400, 300, 4, 400).scale, 4);
  // 非法 factor 不变
  assert.equal(zoomAt(view, NaN, 400, 300).scale, view.scale);

  // 拖拽平移：向右拖 dx=50 → 视图中心 world x 减小 1（scale=50）
  const panned = panView(view, 50, 0);
  close(panned.centerX, 0.5 - 1, 1e-9);
  close(panned.centerY, -0.5, 1e-9);
  // 向下拖 dy=100 → widget 向下 → 世界 y 增加 2
  const pannedDown = panView(view, 0, 100);
  close(pannedDown.centerY, -0.5 + 2, 1e-9);

  const centered = centerOn(view, 3.2, -1.7);
  close(centered.centerX, 3.2, 1e-9);
  close(centered.centerY, -1.7, 1e-9);
  // 原 view 不被修改
  assert.equal(view.centerX, 0.5);
});

test('按键 → 麦轮速度：WASD/QE/Shift/滑杆/限幅/对向抵消', () => {
  const limits = { maxLinear: 0.3, maxLateral: 0.3, maxAngular: 1.5 };
  const base = keyVelocity(new Set(['w']), limits);
  close(base.vx, 0.3, 1e-12);
  close(base.vy, 0, 1e-12);
  close(base.wz, 0, 1e-12);

  assert.ok(keyVelocity(new Set(['s']), limits).vx < 0);
  assert.ok(keyVelocity(new Set(['a']), limits).vy > 0, 'A = 左 = +y');
  assert.ok(keyVelocity(new Set(['d']), limits).vy < 0, 'D = 右 = -y');
  assert.ok(keyVelocity(new Set(['q']), limits).wz > 0, 'Q = 逆时针 +wz');
  assert.ok(keyVelocity(new Set(['e']), limits).wz < 0, 'E = 顺时针 -wz');

  // 斜向
  const diagonal = keyVelocity(new Set(['w', 'a']), limits);
  close(diagonal.vx, 0.3, 1e-12);
  close(diagonal.vy, 0.3, 1e-12);

  // 对向抵消
  assert.deepEqual(keyVelocity(new Set(['w', 's']), limits), { vx: 0, vy: 0, wz: 0 });

  // 滑杆 scale=0.5 → 半速；Shift 加速 1.6× 后仍不超过 limits
  const half = keyVelocity(new Set(['w']), { ...limits, scale: 0.5 });
  close(half.vx, 0.15, 1e-12);
  const halfBoost = keyVelocity(new Set(['w']), { ...limits, scale: 0.5, boost: true });
  close(halfBoost.vx, 0.24, 1e-12);
  const fullBoost = keyVelocity(new Set(['w']), { ...limits, scale: 1, boost: true });
  close(fullBoost.vx, 0.3, 1e-12);
  const keyShift = keyVelocity(new Set(['w', 'shift']), { ...limits, scale: 0.5 });
  close(keyShift.vx, 0.24, 1e-12, 'keys 中的 shift 等同 boost');

  // 数组/对象形式
  close(keyVelocity(['w'], limits).vx, 0.3, 1e-12);
  close(keyVelocity({ w: true, a: true }, limits).vy, 0.3, 1e-12);
  // 方向键别名
  close(keyVelocity(new Set(['arrowup']), limits).vx, 0.3, 1e-12);
  // limits 缺失/非法 → 兜底不抛错
  const fallback = keyVelocity(new Set(['w']), {});
  assert.ok(Number.isFinite(fallback.vx) && fallback.vx > 0);
});

test('虚拟摇杆：死区、上推前进、右推右移、限幅', () => {
  const dead = normalizeInputVector(0.02, -0.02, 0.08);
  assert.deepEqual(dead, { x: 0, y: 0, magnitude: 0 });

  const up = normalizeInputVector(0, -1, 0.08);
  assert.equal(up.magnitude, 1);
  close(up.y, -1, 1e-12);

  const limits = { maxLinear: 0.3, maxLateral: 0.2, maxAngular: 1.5, scale: 0.5 };
  const forward = joystickVelocity(0, -1, limits);
  close(forward.vx, 0.15, 1e-12);
  close(forward.vy, 0, 1e-12);
  const right = joystickVelocity(1, 0, limits);
  close(right.vy, -0.1, 1e-12);
  const deadZone = joystickVelocity(0.03, 0.02, limits);
  assert.equal(deadZone.vx, 0);
  assert.equal(deadZone.vy, 0);
  // 斜向不超过上限
  const diagonal = joystickVelocity(1, -1, limits);
  assert.ok(Math.abs(diagonal.vx) <= 0.3 + EPS);
  assert.ok(Math.abs(diagonal.vy) <= 0.2 + EPS);

  // 二次限幅
  const clamped = clampVelocity({ vx: 9, vy: -9, wz: 9 }, { maxLinear: 0.3, maxLateral: 0.3, maxAngular: 1.5 });
  assert.deepEqual(clamped, { vx: 0.3, vy: -0.3, wz: 1.5 });
});

test('网格间距与比例尺选择', () => {
  const spacing = chooseGridSpacing(50);
  assert.ok(spacing > 0);
  // 目标像素间距在 45~180 px 之间（1/2/5 步进的容差）
  const px = spacing * 50;
  assert.ok(px >= 45 && px <= 180, `网格像素间距异常: ${px}`);
  assert.ok(chooseGridSpacing(100) > 0);

  for (const scale of [8, 20, 40, 57.5, 100, 250]) {
    const bar = chooseScaleBar(scale);
    assert.ok(bar.meters > 0);
    assert.ok(bar.pixels > 0 && bar.pixels <= 140 + 1e-6, `比例尺超长: ${bar.pixels}`);
    assert.match(bar.label, /^[\d.]+ (m|cm)$/);
  }
  const meterBar = chooseScaleBar(50);
  close(meterBar.meters, 2, 1e-9); // 140/50=2.8 → 2 m
  assert.equal(meterBar.label, '2 m');
  close(meterBar.pixels, 100, 1e-9);
});

test('格式化：距离/速度/角度/时长/延迟/时间戳/百分比', () => {
  assert.equal(formatNumber(1.234, 2), '1.23');
  assert.equal(formatNumber(null), '—');
  assert.equal(formatMeters(1.2), '1.20 m');
  assert.equal(formatMeters(-0.4), '-0.40 m');
  assert.equal(formatMeters(undefined), '—');
  assert.equal(formatSpeed(0.1), '0.10 m/s');
  assert.equal(formatAngularSpeed(0.5), '0.50 rad/s');
  assert.equal(formatYawDegrees(Math.PI / 2), '90.0°');
  assert.equal(formatYawDegrees(-Math.PI / 2), '-90.0°');
  assert.equal(formatYawDegrees(null), '—');

  assert.equal(formatDuration(42), '42 s');
  assert.equal(formatDuration(65), '1:05');
  assert.equal(formatDuration(3661), '1:01:01');
  assert.equal(formatDuration(0.5), '0.5 s');
  assert.equal(formatDuration(null), '—');
  assert.equal(formatDuration(-1), '—');

  assert.equal(formatLatency(12.34), '12.3 ms');
  assert.equal(formatLatency(null), '—');
  assert.equal(formatAge(0.02), '20 ms');
  assert.equal(formatAge(1.24), '1.2 s');
  assert.equal(formatAge(12.4), '12 s');
  assert.equal(formatPercent(0.426), '43%');
  assert.equal(formatPercent(0.426, 1), '42.6%');

  const ts = 1764012345.1;
  const date = new Date(ts * 1000);
  const pad = (n) => String(n).padStart(2, '0');
  assert.equal(
    formatTimestamp(ts),
    `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`,
  );
  assert.equal(formatTimestamp(null), '—');

  close(distance(0, 0, 3, 4), 5, 1e-12);
});
