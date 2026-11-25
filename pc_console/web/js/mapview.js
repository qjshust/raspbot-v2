/**
 * mapview.js —— 地图画布。
 *
 * 分层绘制：
 *   - 底图层（离屏 canvas）：背景、底图 PNG、网格/坐标轴、比例尺；仅在视图/地图变化时重绘；
 *   - 动态层（离屏 canvas）：激光点、路径、轨迹、机器人箭头、目标、初始位姿（含预览）、
 *     鼠标十字；状态更新时置脏，由 requestAnimationFrame 合并重绘；
 *   - 主 canvas：每帧把两层合成（devicePixelRatio 适配）。
 *
 * 交互（§6.2）：
 *   - 滚轮以鼠标为中心缩放、拖拽平移、双击回中、F 跟随、比例尺/网格/坐标显示；
 *   - 工具：V 指针、G 设目的地（单击）、P 设初始位姿（按下定位置、拖拽定朝向、松开发送）、Esc 取消。
 */

import {
  COLORS,
  LAYERS_DEFAULT,
  MAPVIEW,
} from './config.js';
import {
  chooseGridSpacing,
  chooseScaleBar,
  centerOn,
  clamp,
  fitView,
  imagePixelToWorld,
  makeView,
  mapMetaUsable,
  panView,
  screenToWorld,
  worldToScreen,
  yawToScreenAngle,
  zoomAt,
  toFiniteNumber,
  isFiniteNumber,
  distance,
} from './math.js';

const TOOL_POINTER = 'pointer';
const TOOL_GOAL = 'goal';
const TOOL_INITIALPOSE = 'initialpose';
const TOOLS = new Set([TOOL_POINTER, TOOL_GOAL, TOOL_INITIALPOSE]);

export class MapView {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {object} options 回调：
   *   onGoal({x,y})                    G 工具单击地图
   *   onInitialPose({x,y,yaw})         P 工具松手
   *   onHover({inside, world, screen}) 鼠标坐标（底部 HUD）
   *   onViewChange(view)               缩放/平移变化
   *   onFollowChange(follow)           跟随状态变化（拖拽会自动关闭跟随）
   *   onToolChange(tool)               工具变化（含 Esc/工具完成后的自动切回）
   *   onToolCancel()                   Esc 取消工具
   *   onError(message)                 底图加载失败等
   *   onMapImageStatus(status)         'loading' | 'loaded' | 'failed'
   */
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.options = options;
    this.ctx = canvas ? canvas.getContext('2d') : null;
    this.viewportEl = options.viewportEl || (canvas ? canvas.parentElement : null);

    this.dpr = Math.max(1, Math.min(3, (typeof window !== 'undefined' && window.devicePixelRatio) || 1));
    this.cssWidth = 0;
    this.cssHeight = 0;
    this.pixelWidth = 0;
    this.pixelHeight = 0;

    this.view = makeView({ scale: MAPVIEW.defaultScale, width: 800, height: 500 });
    this.mapMeta = null;
    this.mapImage = null;
    this.mapImageKey = null;
    this.mapImageFailed = false;
    this.mapImageLoading = false;
    this.hasAutoFit = false;
    this.userAdjustedView = false;

    this.scan = null;
    this.path = null;
    this.robot = null;
    this.goal = null;
    this.initialpose = null;
    this.trail = [];

    this.layers = { ...LAYERS_DEFAULT };
    this.tool = TOOL_POINTER;
    this.follow = false;
    this.hover = { inside: false, screen: { x: 0, y: 0 }, world: { x: 0, y: 0 } };
    this.dragging = null;
    this.preview = null;
    this.rafId = null;
    this.baseDirty = true;
    this.dynDirty = true;
    this.destroyed = false;

    this.baseCanvas = document.createElement('canvas');
    this.baseCtx = this.baseCanvas.getContext('2d');
    this.dynCanvas = document.createElement('canvas');
    this.dynCtx = this.dynCanvas.getContext('2d');

    this._bound = {
      pointerdown: (event) => this._onPointerDown(event),
      pointermove: (event) => this._onPointerMove(event),
      pointerup: (event) => this._onPointerUp(event),
      pointercancel: (event) => this._onPointerCancel(event),
      pointerleave: (event) => this._onPointerLeave(event),
      wheel: (event) => this._onWheel(event),
      dblclick: (event) => this._onDoubleClick(event),
      contextmenu: (event) => event.preventDefault(),
    };
    this._loop = this._loop.bind(this);

    this._bindEvents();
    this._observeResize();
    this._resize();
    this._rafId = requestAnimationFrame(this._loop);
  }

  /* ===================================================================== */
  /* 生命周期与尺寸                                                        */
  /* ===================================================================== */

  _bindEvents() {
    if (!this.canvas) return;
    const c = this.canvas;
    c.addEventListener('pointerdown', this._bound.pointerdown);
    c.addEventListener('pointermove', this._bound.pointermove);
    c.addEventListener('pointerup', this._bound.pointerup);
    c.addEventListener('pointercancel', this._bound.pointercancel);
    c.addEventListener('pointerleave', this._bound.pointerleave);
    c.addEventListener('wheel', this._bound.wheel, { passive: false });
    c.addEventListener('dblclick', this._bound.dblclick);
    c.addEventListener('contextmenu', this._bound.contextmenu);
  }

  _observeResize() {
    if (typeof ResizeObserver !== 'undefined' && this.viewportEl) {
      this.resizeObserver = new ResizeObserver(() => this._resize());
      this.resizeObserver.observe(this.viewportEl);
    } else if (typeof window !== 'undefined') {
      this.windowResize = () => this._resize();
      window.addEventListener('resize', this.windowResize);
    }
  }

  _resize() {
    if (!this.canvas || this.destroyed) return;
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width));
    const height = Math.max(1, Math.round(rect.height));
    if (width === this.cssWidth && height === this.cssHeight) return;
    this.cssWidth = width;
    this.cssHeight = height;
    this.pixelWidth = Math.max(1, Math.round(width * this.dpr));
    this.pixelHeight = Math.max(1, Math.round(height * this.dpr));

    this.canvas.width = this.pixelWidth;
    this.canvas.height = this.pixelHeight;
    this.baseCanvas.width = this.pixelWidth;
    this.baseCanvas.height = this.pixelHeight;
    this.dynCanvas.width = this.pixelWidth;
    this.dynCanvas.height = this.pixelHeight;

    this.view = makeView({ ...this.view, width, height });
    this.baseDirty = true;
    this.dynDirty = true;
  }

  destroy() {
    this.destroyed = true;
    if (this.rafId) cancelAnimationFrame(this.rafId);
    this.rafId = null;
    this.resizeObserver?.disconnect();
    if (this.windowResize) window.removeEventListener('resize', this.windowResize);
    if (this.canvas) {
      const c = this.canvas;
      c.removeEventListener('pointerdown', this._bound.pointerdown);
      c.removeEventListener('pointermove', this._bound.pointermove);
      c.removeEventListener('pointerup', this._bound.pointerup);
      c.removeEventListener('pointercancel', this._bound.pointercancel);
      c.removeEventListener('pointerleave', this._bound.pointerleave);
      c.removeEventListener('wheel', this._bound.wheel);
      c.removeEventListener('dblclick', this._bound.dblclick);
      c.removeEventListener('contextmenu', this._bound.contextmenu);
    }
  }

  /* ===================================================================== */
  /* 数据入口                                                              */
  /* ===================================================================== */

  /** 设置/更新地图元数据（§3.1 map 对象）；版本号变化时重载底图。 */
  setMap(meta, imageUrl) {
    if (!mapMetaUsable(meta)) {
      this.mapMeta = null;
      this.mapImage = null;
      this.mapImageKey = null;
      this.mapImageFailed = false;
      this.baseDirty = true;
      return;
    }
    const version = Number.isFinite(meta.version) ? meta.version : 0;
    const key = `${version}:${meta.width}x${meta.height}:${meta.resolution}:${meta.origin?.x},${meta.origin?.y},${meta.origin?.yaw}`;
    this.mapMeta = meta;
    this.dynDirty = true;
    if (this.mapImageKey === key && this.mapImage) {
      // 版本未变：只更新元数据，不重绘底图
      return;
    }
    this.mapImageKey = key;
    this.mapImageFailed = false;
    this._loadMapImage(imageUrl, key);
    if (!this.hasAutoFit && !this.userAdjustedView) {
      this.fitToMap();
      this.hasAutoFit = true;
    }
    this.baseDirty = true;
  }

  _loadMapImage(url, key) {
    if (typeof Image === 'undefined' || !url) return;
    this.mapImageLoading = true;
    this.options.onMapImageStatus?.('loading');
    const image = new Image();
    image.decoding = 'async';
    image.onload = () => {
      if (key !== this.mapImageKey) return;
      this.mapImage = image;
      this.mapImageLoading = false;
      this.mapImageFailed = false;
      this.baseDirty = true;
      this.options.onMapImageStatus?.('loaded');
    };
    image.onerror = () => {
      if (key !== this.mapImageKey) return;
      this.mapImage = null;
      this.mapImageLoading = false;
      this.mapImageFailed = true;
      this.baseDirty = true;
      this.options.onMapImageStatus?.('failed');
      this.options.onError?.('地图底图加载失败：请确认会话是否已产生地图');
    };
    image.src = url;
  }

  /** 激光点云（§3.2 scan 事件）：{stamp, frame, points:[x0,y0,...]}。 */
  setScan(scan) {
    this.scan = scan && Array.isArray(scan.points) ? scan : null;
    this.dynDirty = true;
  }

  /** 导航路径（§3.2 path 事件）。 */
  setPath(path) {
    this.path = path && Array.isArray(path.points) ? path : null;
    this.dynDirty = true;
  }

  /** 机器人位姿（state.robot）；robot 可为 null。 */
  setRobot(robot) {
    const valid = robot && isFiniteNumber(robot.x) && isFiniteNumber(robot.y);
    if (!valid) {
      this.robot = null;
      this.dynDirty = true;
      return;
    }
    this.robot = robot;
    const last = this.trail[this.trail.length - 1];
    if (!last || distance(last.x, last.y, robot.x, robot.y) >= MAPVIEW.trailMinStep) {
      this.trail.push({ x: robot.x, y: robot.y });
      if (this.trail.length > MAPVIEW.trailMax) this.trail.splice(0, this.trail.length - MAPVIEW.trailMax);
    }
    this.dynDirty = true;
    if (this.follow) {
      this._followStep();
    }
  }

  /** 已接受的初始位姿（state.nav.initialpose）。 */
  setInitialPose(pose) {
    this.initialpose = pose && isFiniteNumber(pose.x) && isFiniteNumber(pose.y) ? pose : null;
    this.dynDirty = true;
  }

  /** 当前导航目标（state.nav.goal）。 */
  setGoal(goal) {
    this.goal = goal && isFiniteNumber(goal.x) && isFiniteNumber(goal.y) ? goal : null;
    this.dynDirty = true;
  }

  setLayers(layers) {
    this.layers = { ...this.layers, ...(layers || {}) };
    this.baseDirty = true;
    this.dynDirty = true;
  }

  /** 清空动态数据（会话切换/停止时调用；断连时不要调用，以保留最后画面）。 */
  clearDynamic() {
    this.scan = null;
    this.path = null;
    this.goal = null;
    this.initialpose = null;
    this.trail = [];
    this.preview = null;
    this.dynDirty = true;
  }

  clearTrail() {
    this.trail = [];
    this.dynDirty = true;
  }

  /* ===================================================================== */
  /* 视图操作                                                              */
  /* ===================================================================== */

  setFollow(follow) {
    const next = Boolean(follow);
    if (this.follow === next) return;
    this.follow = next;
    if (next) this._followStep(true);
    this.options.onFollowChange?.(next);
  }

  toggleFollow() {
    this.setFollow(!this.follow);
    return this.follow;
  }

  recenter() {
    if (this.robot && isFiniteNumber(this.robot.x)) {
      this._applyView(centerOn(this.view, this.robot.x, this.robot.y), { user: true });
    } else if (this.mapMeta) {
      this.fitToMap();
    }
  }

  fitToMap() {
    if (!this.mapMeta) return;
    const next = fitView(this.mapMeta, this.cssWidth || this.view.width, this.cssHeight || this.view.height, MAPVIEW.fitPadding);
    this.view = { ...next, width: this.view.width, height: this.view.height };
    this.baseDirty = true;
    this.dynDirty = true;
    this.options.onViewChange?.(this.view);
  }

  zoomBy(factor) {
    const sx = this.cssWidth / 2;
    const sy = this.cssHeight / 2;
    this._applyView(
      zoomAt(this.view, factor, sx, sy, MAPVIEW.minScale, MAPVIEW.maxScale),
      { user: true },
    );
  }

  setTool(tool) {
    if (!TOOLS.has(tool) || tool === this.tool) return;
    this.tool = tool;
    this.preview = null;
    this.dragging = this.dragging && this.dragging.mode !== 'initialpose' ? this.dragging : null;
    this._applyToolClasses();
    this.dynDirty = true;
    this.options.onToolChange?.(tool);
  }

  cancelTool() {
    const was = this.tool;
    this.preview = null;
    this.dragging = null;
    this.dynDirty = true;
    this.tool = TOOL_POINTER;
    this._applyToolClasses();
    if (was !== TOOL_POINTER) this.options.onToolCancel?.();
    this.options.onToolChange?.(this.tool);
  }

  _applyToolClasses() {
    if (!this.viewportEl) return;
    this.viewportEl.classList.toggle('tool-goal', this.tool === TOOL_GOAL);
    this.viewportEl.classList.toggle('tool-initialpose', this.tool === TOOL_INITIALPOSE);
  }

  _applyView(view, { user = false } = {}) {
    this.view = makeView({
      ...view,
      width: this.view.width,
      height: this.view.height,
    });
    if (user) this.userAdjustedView = true;
    this.baseDirty = true;
    this.dynDirty = true;
    this.options.onViewChange?.(this.view);
  }

  _followStep(immediate = false) {
    if (!this.robot) return;
    const targetX = this.robot.x;
    const targetY = this.robot.y;
    const dx = targetX - this.view.centerX;
    const dy = targetY - this.view.centerY;
    const tolerance = 0.5 / this.view.scale;
    if (!immediate && Math.abs(dx) < tolerance && Math.abs(dy) < tolerance) return;
    const k = immediate ? 1 : MAPVIEW.followLerp;
    const next = {
      ...this.view,
      centerX: this.view.centerX + dx * k,
      centerY: this.view.centerY + dy * k,
    };
    this.view = next;
    this.baseDirty = true;
    this.dynDirty = true;
    this.options.onViewChange?.(this.view);
  }

  _stopFollow(reason = 'user') {
    if (!this.follow) return;
    this.follow = false;
    this.options.onFollowChange?.(false, reason);
  }

  /* ===================================================================== */
  /* 指针交互                                                              */
  /* ===================================================================== */

  _localPoint(event) {
    const rect = this.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  _onPointerDown(event) {
    if (!this.canvas || (event.button !== 0 && event.button !== 1)) return;
    event.preventDefault();
    const point = this._localPoint(event);
    try {
      this.canvas.setPointerCapture(event.pointerId);
    } catch {
      /* 部分环境不支持捕获 */
    }
    const world = screenToWorld(this.view, point.x, point.y);
    if (event.button === 1) {
      this.dragging = { mode: 'pan', lastX: point.x, lastY: point.y, pointerId: event.pointerId, moved: 0 };
      this.viewportEl?.classList.add('dragging');
      return;
    }
    if (this.tool === TOOL_INITIALPOSE) {
      const yaw = this.robot && isFiniteNumber(this.robot.yaw) ? this.robot.yaw : 0;
      this.preview = { x: world.x, y: world.y, yaw };
      this.dragging = {
        mode: 'initialpose',
        start: world,
        last: world,
        lastX: point.x,
        lastY: point.y,
        pointerId: event.pointerId,
        moved: 0,
      };
      this.dynDirty = true;
      return;
    }
    this.dragging = {
      mode: this.tool === TOOL_GOAL ? 'goal' : 'pan',
      lastX: point.x,
      lastY: point.y,
      pointerId: event.pointerId,
      moved: 0,
    };
    if (this.dragging.mode === 'pan') this.viewportEl?.classList.add('dragging');
  }

  _onPointerMove(event) {
    const point = this._localPoint(event);
    const world = screenToWorld(this.view, point.x, point.y);
    this.hover = { inside: true, screen: point, world };
    this.options.onHover?.({ inside: true, world, screen: point });

    if (!this.dragging) {
      this.dynDirty = true;
      return;
    }
    const drag = this.dragging;
    if (drag.mode === 'pan') {
      const dx = point.x - drag.lastX;
      const dy = point.y - drag.lastY;
      drag.moved += Math.abs(dx) + Math.abs(dy);
      drag.lastX = point.x;
      drag.lastY = point.y;
      if (drag.moved > 2) this._stopFollow('pan');
      this.view = makeView({
        ...panView(this.view, dx, dy),
        width: this.view.width,
        height: this.view.height,
      });
      this.baseDirty = true;
      this.dynDirty = true;
      this.options.onViewChange?.(this.view);
    } else if (drag.mode === 'goal') {
      drag.moved += Math.abs(point.x - drag.lastX) + Math.abs(point.y - drag.lastY);
      drag.lastX = point.x;
      drag.lastY = point.y;
    } else if (drag.mode === 'initialpose') {
      drag.last = world;
      drag.moved += Math.abs(point.x - drag.lastX) + Math.abs(point.y - drag.lastY);
      drag.lastX = point.x;
      drag.lastY = point.y;
      const dx = world.x - drag.start.x;
      const dy = world.y - drag.start.y;
      const yaw = Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6
        ? (this.robot && isFiniteNumber(this.robot.yaw) ? this.robot.yaw : 0)
        : Math.atan2(dy, dx);
      this.preview = { x: drag.start.x, y: drag.start.y, yaw };
      this.dynDirty = true;
    }
  }

  _onPointerUp(event) {
    if (!this.dragging) return;
    const drag = this.dragging;
    this.dragging = null;
    this.viewportEl?.classList.remove('dragging');
    try {
      this.canvas.releasePointerCapture(event.pointerId);
    } catch {
      /* 忽略 */
    }
    const point = this._localPoint(event);
    const world = screenToWorld(this.view, point.x, point.y);

    if (drag.mode === 'goal') {
      if (drag.moved <= 5) {
        this.options.onGoal?.({ x: world.x, y: world.y });
      }
    } else if (drag.mode === 'initialpose') {
      const preview = this.preview;
      this.preview = null;
      this.dynDirty = true;
      if (drag.moved <= 3) {
        const yaw = this.robot && isFiniteNumber(this.robot.yaw) ? this.robot.yaw : 0;
        this.options.onInitialPose?.({ x: drag.start.x, y: drag.start.y, yaw });
      } else if (preview) {
        this.options.onInitialPose?.({ x: preview.x, y: preview.y, yaw: preview.yaw });
      }
    }
  }

  _onPointerCancel() {
    this.dragging = null;
    this.preview = null;
    this.viewportEl?.classList.remove('dragging');
    this.dynDirty = true;
  }

  _onPointerLeave() {
    this.hover = { ...this.hover, inside: false };
    if (!this.dragging) {
      this.options.onHover?.({ inside: false, world: this.hover.world, screen: this.hover.screen });
      this.dynDirty = true;
    }
  }

  _onWheel(event) {
    event.preventDefault();
    const point = this._localPoint(event);
    const factor = event.deltaY < 0 ? MAPVIEW.wheelFactor : 1 / MAPVIEW.wheelFactor;
    this._applyView(zoomAt(this.view, factor, point.x, point.y, MAPVIEW.minScale, MAPVIEW.maxScale), {
      user: true,
    });
  }

  _onDoubleClick(event) {
    if (this.tool !== TOOL_POINTER) return;
    event.preventDefault();
    this.recenter();
  }

  /* ===================================================================== */
  /* 渲染循环                                                              */
  /* ===================================================================== */

  _loop() {
    if (this.destroyed) return;
    if (this.cssWidth > 0 && this.cssHeight > 0 && (this.baseDirty || this.dynDirty)) {
      if (this.baseDirty) this._drawBase();
      if (this.dynDirty) this._drawDynamic();
      this._composite();
      this.baseDirty = false;
      this.dynDirty = false;
    }
    this.rafId = requestAnimationFrame(this._loop);
  }

  _composite() {
    const ctx = this.ctx;
    if (!ctx) return;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.pixelWidth, this.pixelHeight);
    ctx.drawImage(this.baseCanvas, 0, 0);
    ctx.drawImage(this.dynCanvas, 0, 0);
  }

  _prepareCtx(ctx) {
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.cssWidth, this.cssHeight);
  }

  /* ------------------------------ 底图层 ------------------------------ */

  _drawBase() {
    const ctx = this.baseCtx;
    if (!ctx) return;
    this._prepareCtx(ctx);
    ctx.fillStyle = COLORS.mapBackground;
    ctx.fillRect(0, 0, this.cssWidth, this.cssHeight);

    if (this.layers.grid) this._drawGrid(ctx);
    this._drawMapImage(ctx);
    this._drawMapBorder(ctx);
    this._drawScaleBar(ctx);
  }

  _drawGrid(ctx) {
    const spacing = chooseGridSpacing(this.view.scale, MAPVIEW.gridTargetPx);
    const topLeft = screenToWorld(this.view, 0, 0);
    const bottomRight = screenToWorld(this.view, this.cssWidth, this.cssHeight);
    const minX = Math.min(topLeft.x, bottomRight.x);
    const maxX = Math.max(topLeft.x, bottomRight.x);
    const minY = Math.min(topLeft.y, bottomRight.y);
    const maxY = Math.max(topLeft.y, bottomRight.y);
    const startX = Math.ceil(minX / spacing) * spacing;
    const startY = Math.ceil(minY / spacing) * spacing;
    ctx.save();
    ctx.lineWidth = 1 / this.dpr;
    ctx.font = `${11}px ${'ui-monospace, Consolas, monospace'}`;
    ctx.textBaseline = 'top';
    let count = 0;
    for (let x = startX; x <= maxX && count < 400; x += spacing, count += 1) {
      const screen = worldToScreen(this.view, x, 0);
      const index = Math.round(x / spacing);
      ctx.strokeStyle = Math.abs(x) < spacing / 2 ? COLORS.axis : index % 5 === 0 ? COLORS.gridMajor : COLORS.grid;
      ctx.beginPath();
      ctx.moveTo(Math.round(screen.x) + 0.5, 0);
      ctx.lineTo(Math.round(screen.x) + 0.5, this.cssHeight);
      ctx.stroke();
    }
    count = 0;
    for (let y = startY; y <= maxY && count < 400; y += spacing, count += 1) {
      const screen = worldToScreen(this.view, 0, y);
      const index = Math.round(y / spacing);
      ctx.strokeStyle = Math.abs(y) < spacing / 2 ? COLORS.axis : index % 5 === 0 ? COLORS.gridMajor : COLORS.grid;
      ctx.beginPath();
      ctx.moveTo(0, Math.round(screen.y) + 0.5);
      ctx.lineTo(this.cssWidth, Math.round(screen.y) + 0.5);
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawMapImage(ctx) {
    const meta = this.mapMeta;
    const image = this.mapImage;
    if (!meta || !image) return;
    const topLeft = imagePixelToWorld(meta, 0, 0);
    const screen = worldToScreen(this.view, topLeft.x, topLeft.y);
    const scale = meta.resolution * this.view.scale;
    const yaw = toFiniteNumber(meta.origin && meta.origin.yaw, 0);
    ctx.save();
    ctx.translate(screen.x, screen.y);
    ctx.rotate(-yaw);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(image, 0, 0, meta.width * scale, meta.height * scale);
    ctx.restore();
  }

  _drawMapBorder(ctx) {
    if (!this.mapMeta) return;
    const corners = [
      imagePixelToWorld(this.mapMeta, 0, 0),
      imagePixelToWorld(this.mapMeta, this.mapMeta.width, 0),
      imagePixelToWorld(this.mapMeta, this.mapMeta.width, this.mapMeta.height),
      imagePixelToWorld(this.mapMeta, 0, this.mapMeta.height),
    ].map((world) => worldToScreen(this.view, world.x, world.y));
    ctx.save();
    ctx.strokeStyle = COLORS.mapBorder;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    for (let index = 1; index < corners.length; index += 1) {
      ctx.lineTo(corners[index].x, corners[index].y);
    }
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  _drawScaleBar(ctx) {
    const bar = chooseScaleBar(this.view.scale, MAPVIEW.scaleBarMaxPx);
    const x = 16;
    const y = this.cssHeight - 18;
    ctx.save();
    ctx.font = '11px ui-monospace, Consolas, monospace';
    const labelWidth = ctx.measureText(bar.label).width;
    const width = Math.max(bar.pixels, labelWidth);
    ctx.fillStyle = COLORS.scaleBarBg;
    ctx.beginPath();
    ctx.roundRect(x - 6, y - 20, width + 14, 28, 6);
    ctx.fill();
    ctx.strokeStyle = COLORS.scaleBar;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + bar.pixels, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 2);
    ctx.moveTo(x + bar.pixels, y - 4);
    ctx.lineTo(x + bar.pixels, y + 2);
    ctx.stroke();
    ctx.fillStyle = COLORS.textDim || COLORS.textMuted;
    ctx.textBaseline = 'bottom';
    ctx.fillText(bar.label, x, y - 6);
    ctx.restore();
  }

  /* ------------------------------ 动态层 ------------------------------ */

  _drawDynamic() {
    const ctx = this.dynCtx;
    if (!ctx) return;
    this._prepareCtx(ctx);

    if (this.layers.scan) this._drawScan(ctx);
    if (this.layers.path) this._drawPath(ctx);
    if (this.layers.trail) this._drawTrail(ctx);
    this._drawInitialPoseMarker(ctx);
    this._drawGoal(ctx);
    if (this.layers.robot) this._drawRobot(ctx);
    this._drawPreview(ctx);
    if (MAPVIEW.hoverCrosshair) this._drawHoverCrosshair(ctx);
  }

  /** 判断某事件帧与当前机器人/地图帧是否一致（不一致则隐藏，避免错误叠加）。 */
  _frameMatches(frame) {
    if (!frame || frame === 'map') return true;
    if (!this.robot || !this.robot.frame) return false;
    return this.robot.frame === frame;
  }

  _drawScan(ctx) {
    const scan = this.scan;
    if (!scan || !this._frameMatches(scan.frame)) return;
    const points = scan.points;
    const count = Math.floor(points.length / 2);
    if (!count) return;
    ctx.save();
    ctx.fillStyle = COLORS.scan;
    for (let index = 0; index < count; index += 1) {
      const wx = points[index * 2];
      const wy = points[index * 2 + 1];
      if (!isFiniteNumber(wx) || !isFiniteNumber(wy)) continue;
      const screen = worldToScreen(this.view, wx, wy);
      if (screen.x < -4 || screen.y < -4 || screen.x > this.cssWidth + 4 || screen.y > this.cssHeight + 4) {
        continue;
      }
      const radius = this.view.scale > 90 ? 1.6 : this.view.scale > 40 ? 2 : 2.4;
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  _drawPath(ctx) {
    const path = this.path;
    if (!path || !this._frameMatches(path.frame)) return;
    const points = path.points;
    if (points.length < 4) return;
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    // 光晕 + 主线
    for (const [width, color] of [
      [7, COLORS.pathGlow],
      [2.5, COLORS.path],
    ]) {
      ctx.beginPath();
      let started = false;
      for (let index = 0; index + 1 < points.length; index += 2) {
        const wx = points[index];
        const wy = points[index + 1];
        if (!isFiniteNumber(wx) || !isFiniteNumber(wy)) continue;
        const screen = worldToScreen(this.view, wx, wy);
        if (!started) {
          ctx.moveTo(screen.x, screen.y);
          started = true;
        } else {
          ctx.lineTo(screen.x, screen.y);
        }
      }
      if (started) {
        ctx.lineWidth = width;
        ctx.strokeStyle = color;
        ctx.stroke();
      }
    }
    // 终点圆点
    const last = worldToScreen(this.view, points[points.length - 2], points[points.length - 1]);
    ctx.beginPath();
    ctx.arc(last.x, last.y, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = COLORS.path;
    ctx.fill();
    ctx.restore();
  }

  _drawTrail(ctx) {
    const trail = this.trail;
    if (!trail || trail.length < 2) return;
    ctx.save();
    ctx.beginPath();
    for (let index = 0; index < trail.length; index += 1) {
      const screen = worldToScreen(this.view, trail[index].x, trail[index].y);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    }
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = COLORS.trail;
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.restore();
  }

  _drawArrow(ctx, screen, yaw, length, width, fill, stroke, dashed = false) {
    ctx.save();
    ctx.translate(screen.x, screen.y);
    ctx.rotate(yawToScreenAngle(yaw));
    ctx.beginPath();
    ctx.moveTo(length, 0);
    ctx.lineTo(-length * 0.55, width);
    ctx.lineTo(-length * 0.25, 0);
    ctx.lineTo(-length * 0.55, -width);
    ctx.closePath();
    if (fill) {
      ctx.fillStyle = fill;
      ctx.fill();
    }
    if (stroke) {
      ctx.lineWidth = 1.6;
      ctx.strokeStyle = stroke;
      if (dashed) ctx.setLineDash([3, 3]);
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawRobot(ctx) {
    const robot = this.robot;
    if (!robot) return;
    const screen = worldToScreen(this.view, robot.x, robot.y);
    const unlocalized = robot.frame && robot.frame !== 'map';
    const size = clamp(this.view.scale * 0.28, 12, 22);
    this._drawArrow(
      ctx,
      screen,
      robot.yaw,
      size,
      size * 0.62,
      unlocalized ? COLORS.robotOdom : COLORS.robot,
      COLORS.robotOutline,
    );
    ctx.save();
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 2, 0, Math.PI * 2);
    ctx.fillStyle = COLORS.robotOutline;
    ctx.fill();
    if (unlocalized) {
      ctx.beginPath();
      ctx.setLineDash([3, 3]);
      ctx.arc(screen.x, screen.y, size + 5, 0, Math.PI * 2);
      ctx.strokeStyle = COLORS.robotOdom;
      ctx.lineWidth = 1.2;
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawGoal(ctx) {
    const goal = this.goal;
    if (!goal) return;
    const screen = worldToScreen(this.view, goal.x, goal.y);
    ctx.save();
    ctx.strokeStyle = COLORS.goal;
    ctx.fillStyle = COLORS.goalFill;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(screen.x - 14, screen.y);
    ctx.lineTo(screen.x - 5, screen.y);
    ctx.moveTo(screen.x + 5, screen.y);
    ctx.lineTo(screen.x + 14, screen.y);
    ctx.moveTo(screen.x, screen.y - 14);
    ctx.lineTo(screen.x, screen.y - 5);
    ctx.moveTo(screen.x, screen.y + 5);
    ctx.lineTo(screen.x, screen.y + 14);
    ctx.stroke();
    if (isFiniteNumber(goal.yaw)) {
      ctx.beginPath();
      ctx.moveTo(screen.x, screen.y);
      ctx.lineTo(screen.x + Math.cos(yawToScreenAngle(goal.yaw)) * 24, screen.y + Math.sin(yawToScreenAngle(goal.yaw)) * 24);
      ctx.lineWidth = 2;
      ctx.stroke();
    }
    if (this.robot) {
      const meters = distance(this.robot.x, this.robot.y, goal.x, goal.y);
      ctx.font = '11px ui-monospace, Consolas, monospace';
      ctx.fillStyle = COLORS.goal;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'bottom';
      ctx.fillText(`${meters.toFixed(2)} m`, screen.x, screen.y - 14);
    }
    ctx.restore();
  }

  _drawInitialPoseMarker(ctx) {
    const pose = this.initialpose;
    if (!pose) return;
    const screen = worldToScreen(this.view, pose.x, pose.y);
    ctx.save();
    ctx.globalAlpha = 0.85;
    this._drawArrow(ctx, screen, pose.yaw, 17, 10, COLORS.initialposeFill, COLORS.initialpose, true);
    ctx.restore();
  }

  _drawPreview(ctx) {
    const preview = this.preview;
    if (!preview) return;
    const screen = worldToScreen(this.view, preview.x, preview.y);
    ctx.save();
    // 起始点
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 5, 0, Math.PI * 2);
    ctx.fillStyle = COLORS.initialpose;
    ctx.fill();
    // 朝向箭头
    ctx.strokeStyle = COLORS.initialpose;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(screen.x, screen.y);
    ctx.lineTo(
      screen.x + Math.cos(yawToScreenAngle(preview.yaw)) * 42,
      screen.y + Math.sin(yawToScreenAngle(preview.yaw)) * 42,
    );
    ctx.stroke();
    // 箭头头部
    const angle = yawToScreenAngle(preview.yaw);
    const tipX = screen.x + Math.cos(angle) * 42;
    const tipY = screen.y + Math.sin(angle) * 42;
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(tipX - Math.cos(angle - 0.5) * 10, tipY - Math.sin(angle - 0.5) * 10);
    ctx.lineTo(tipX - Math.cos(angle + 0.5) * 10, tipY - Math.sin(angle + 0.5) * 10);
    ctx.closePath();
    ctx.fillStyle = COLORS.initialpose;
    ctx.fill();
    ctx.restore();
  }

  _drawHoverCrosshair(ctx) {
    if (!this.hover.inside) return;
    const { x, y } = this.hover.screen;
    ctx.save();
    ctx.strokeStyle = COLORS.hover;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x - 5, y);
    ctx.lineTo(x + 5, y);
    ctx.moveTo(x, y - 5);
    ctx.lineTo(x, y + 5);
    ctx.stroke();
    ctx.restore();
  }
}
