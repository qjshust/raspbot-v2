/**
 * ui.js —— 通用 UI 组件：DOM 小工具、自绘模态框、Toast、事件日志面板、地图列表、快捷键表。
 *
 * 约束（§6.3）：无 alert / confirm / prompt，统一用自绘模态框；无外部图标。
 */

import { LAYER_SETTINGS, SHORTCUTS, LOG_LIMIT, mapPreviewUrl } from './config.js';
import { formatNumber, formatTimestamp } from './math.js';

/** 按 id 取元素（唯一允许的取元素方式，供 DOM 契约测试扫描）。 */
export function $el(id) {
  return document.getElementById(id);
}

export function setHidden(el, hidden) {
  if (!el) return;
  el.classList.toggle('hidden', Boolean(hidden));
}

export function setText(el, text) {
  if (!el) return;
  const value = text === null || text === undefined ? '' : String(text);
  if (el.textContent !== value) el.textContent = value;
}

export function setDisabled(el, disabled) {
  if (!el) return;
  el.disabled = Boolean(disabled);
}

export function setPressed(el, pressed) {
  if (!el) return;
  el.setAttribute('aria-pressed', pressed ? 'true' : 'false');
}

/** 切换元素上的 tone-* / badge-tone-* 语义色类。 */
export function setTone(el, tone) {
  if (!el) return;
  for (const name of ['ok', 'warn', 'error', 'info', 'muted']) {
    el.classList.toggle(`tone-${name}`, tone === name);
    el.classList.toggle(`badge-tone-${name}`, tone === name);
  }
}

const TONE_ICON = {
  info: 'i-help',
  ok: 'i-check',
  warn: 'i-alert',
  error: 'i-alert',
};

/* ------------------------------------------------------------------------- */
/* Toast                                                                     */
/* ------------------------------------------------------------------------- */

export class ToastManager {
  constructor(rootEl) {
    this.root = rootEl;
    this.max = 5;
    this.count = 0;
  }

  show(message, options = {}) {
    if (!this.root || !message) return null;
    const level = ['info', 'ok', 'warn', 'error'].includes(options.level) ? options.level : 'info';
    const timeout = Number.isFinite(options.timeout) ? options.timeout : 4200;
    this.count += 1;

    const toast = document.createElement('div');
    toast.className = `toast tone-${level}`;
    toast.setAttribute('role', level === 'error' ? 'alert' : 'status');
    toast.dataset.id = String(this.count);

    const icon = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    icon.setAttribute('class', 'icon toast-icon');
    icon.setAttribute('aria-hidden', 'true');
    const use = document.createElementNS('http://www.w3.org/2000/svg', 'use');
    use.setAttribute('href', `#${TONE_ICON[level] || 'i-help'}`);
    icon.append(use);

    const text = document.createElement('span');
    text.className = 'toast-text';
    text.textContent = String(message);

    toast.append(icon, text);
    this.root.append(toast);

    const remove = () => {
      if (!toast.isConnected) return;
      toast.classList.add('leaving');
      setTimeout(() => toast.remove(), 220);
    };
    if (timeout > 0) setTimeout(remove, timeout);
    toast.addEventListener('click', remove);

    while (this.root.children.length > this.max) {
      this.root.firstElementChild?.remove();
    }
    return toast;
  }

  info(message, timeout) {
    return this.show(message, { level: 'info', timeout });
  }
  success(message, timeout) {
    return this.show(message, { level: 'ok', timeout });
  }
  warn(message, timeout) {
    return this.show(message, { level: 'warn', timeout });
  }
  error(message, timeout) {
    return this.show(message, { level: 'error', timeout: timeout ?? 6000 });
  }
}

/* ------------------------------------------------------------------------- */
/* 模态框管理                                                                */
/* ------------------------------------------------------------------------- */

const FOCUSABLE = 'button:not(:disabled), input:not(:disabled), select:not(:disabled), textarea:not(:disabled), [href]';

/**
 * 自绘模态框管理器：
 *   - 打开/关闭、Esc 关闭、遮罩点击关闭、焦点恢复与 Tab 焦点环；
 *   - openConfirm() 返回 Promise<boolean>，替代 window.confirm。
 */
export class ModalManager {
  constructor(options = {}) {
    this.onChange = typeof options.onChange === 'function' ? options.onChange : () => {};
    this.stack = [];
    this.closeHandlers = new Map();
    this.lastFocus = null;
    this.attached = false;
  }

  attach() {
    if (this.attached) return;
    this.attached = true;
    document.querySelectorAll('[data-dialog]').forEach((dialog) => {
      dialog.addEventListener('click', (event) => {
        if (event.target === dialog) this.close(dialog, false);
      });
      dialog.addEventListener('click', (event) => {
        const closer = event.target instanceof Element ? event.target.closest('[data-close]') : null;
        if (closer && dialog.contains(closer)) this.close(dialog, false);
      });
    });
    document.addEventListener('keydown', (event) => this._onKeyDown(event), true);
  }

  open(dialog, closeHandler) {
    if (!dialog) return;
    if (this.stack.length === 0) this.lastFocus = document.activeElement;
    if (this.stack.includes(dialog)) return;
    this.stack.push(dialog);
    if (typeof closeHandler === 'function') this.closeHandlers.set(dialog, closeHandler);
    dialog.classList.remove('hidden');
    this.onChange(true);
    const focusable = dialog.querySelector('[data-autofocus]') || dialog.querySelector(FOCUSABLE);
    if (focusable && typeof focusable.focus === 'function') {
      requestAnimationFrame(() => focusable.focus());
    }
  }

  close(dialog, result = false) {
    const index = this.stack.indexOf(dialog);
    if (index === -1) return;
    this.stack.splice(index, 1);
    dialog.classList.add('hidden');
    const handler = this.closeHandlers.get(dialog);
    this.closeHandlers.delete(dialog);
    if (handler) handler(result);
    if (this.stack.length === 0 && this.lastFocus && typeof this.lastFocus.focus === 'function') {
      this.lastFocus.focus();
      this.lastFocus = null;
    }
    this.onChange(this.stack.length > 0);
  }

  closeAll() {
    while (this.stack.length > 0) {
      this.close(this.stack[this.stack.length - 1], false);
    }
  }

  isOpen() {
    return this.stack.length > 0;
  }

  isOpenId(id) {
    return this.stack.some((dialog) => dialog.id === id);
  }

  closeTop() {
    if (this.stack.length === 0) return false;
    this.close(this.stack[this.stack.length - 1], false);
    return true;
  }

  /**
   * 二次确认弹窗（替代 window.confirm）。
   * @param {{title?:string, message?:string, okText?:string, cancelText?:string, danger?:boolean}} options
   * @returns {Promise<boolean>}
   */
  openConfirm(options = {}) {
    const dialog = $el('dialog-confirm');
    if (!dialog) return Promise.resolve(false);
    // 已有一个确认框时先按取消处理，避免 Promise 悬挂
    if (this.stack.includes(dialog)) this.close(dialog, false);

    const titleEl = $el('confirm-title');
    const messageEl = $el('confirm-message');
    const okBtn = $el('btn-confirm-ok');
    const cancelBtn = $el('btn-confirm-cancel');
    setText(titleEl, options.title || '确认操作');
    setText(messageEl, options.message || '');
    setText(okBtn, options.okText || '确定');
    setText(cancelBtn, options.cancelText || '取消');
    okBtn.classList.toggle('btn-primary', !options.danger);
    okBtn.classList.toggle('btn-danger-solid', Boolean(options.danger));

    return new Promise((resolve) => {
      const finish = (result) => {
        okBtn.removeEventListener('click', onOk);
        cancelBtn.removeEventListener('click', onCancel);
        resolve(Boolean(result));
      };
      const onOk = () => this.close(dialog, true);
      const onCancel = () => this.close(dialog, false);
      okBtn.addEventListener('click', onOk);
      cancelBtn.addEventListener('click', onCancel);
      this.open(dialog, finish);
      requestAnimationFrame(() => cancelBtn.focus());
    });
  }

  _onKeyDown(event) {
    if (event.key === 'Escape' && this.stack.length > 0) {
      event.preventDefault();
      event.stopPropagation();
      this.closeTop();
      return;
    }
    if (event.key !== 'Tab' || this.stack.length === 0) return;
    const dialog = this.stack[this.stack.length - 1];
    const focusables = Array.from(dialog.querySelectorAll(FOCUSABLE)).filter(
      (el) => el.offsetParent !== null || el === document.activeElement,
    );
    if (focusables.length === 0) return;
    const first = focusables[0];
    const last = focusables[focusables.length - 1];
    if (event.shiftKey && document.activeElement === first) {
      event.preventDefault();
      last.focus();
    } else if (!event.shiftKey && document.activeElement === last) {
      event.preventDefault();
      first.focus();
    }
  }
}

/* ------------------------------------------------------------------------- */
/* 事件日志面板                                                              */
/* ------------------------------------------------------------------------- */

const LEVEL_TEXT = { info: 'INFO', warn: 'WARN', error: 'ERROR' };

export class LogPanel {
  /**
   * @param {object} refs {panelEl,bodyEl,listEl,countEl,filterEl,pauseBtn,clearBtn,downloadBtn,toggleBtn,emptyEl}
   */
  constructor(refs, options = {}) {
    this.panelEl = refs.panelEl;
    this.bodyEl = refs.bodyEl;
    this.listEl = refs.listEl;
    this.countEl = refs.countEl;
    this.filterEl = refs.filterEl;
    this.pauseBtn = refs.pauseBtn;
    this.clearBtn = refs.clearBtn;
    this.downloadBtn = refs.downloadBtn;
    this.toggleBtn = refs.toggleBtn;
    this.emptyEl = refs.emptyEl;
    this.max = Number.isFinite(options.max) ? options.max : LOG_LIMIT;
    this.entries = [];
    this.nodes = [];
    this.filter = 'all';
    this.paused = false;
    this.collapsed = false;
    this.onDownload = typeof options.onDownload === 'function' ? options.onDownload : null;
    this._bind();
    this._renderEmpty();
  }

  _bind() {
    if (this.filterEl) {
      this.filterEl.addEventListener('change', () => this.setFilter(this.filterEl.value));
    }
    if (this.pauseBtn) {
      this.pauseBtn.addEventListener('click', () => this.togglePause());
    }
    if (this.clearBtn) {
      this.clearBtn.addEventListener('click', () => this.clear());
    }
    if (this.downloadBtn) {
      this.downloadBtn.addEventListener('click', () => this.download());
    }
    if (this.toggleBtn) {
      this.toggleBtn.addEventListener('click', () => this.toggleCollapse());
    }
  }

  /**
   * 追加一条日志。
   * @param {{ts:number, level:string, msg:string, source?:string}} entry
   */
  add(entry) {
    if (!entry) return;
    const level = LEVEL_TEXT[entry.level] ? entry.level : 'info';
    const msg = String(entry.msg ?? '');
    if (!msg) return;
    const ts = Number.isFinite(entry.ts) ? entry.ts : Date.now() / 1000;
    const record = { ts, level, msg, source: entry.source || '' };
    this.entries.push(record);
    const node = this._createLine(record);
    this.nodes.push(node);
    if (this.listEl) this.listEl.append(node);
    while (this.entries.length > this.max) {
      this.entries.shift();
      const old = this.nodes.shift();
      old?.remove();
    }
    if (this.countEl) this.countEl.textContent = String(this.entries.length);
    this._renderEmpty();
    this._applyVisibility(node, level);
    if (!this.paused) this._scrollToBottom();
  }

  addBatch(entries) {
    if (!Array.isArray(entries)) return;
    for (const entry of entries) this.add(entry);
  }

  _createLine(record) {
    const line = document.createElement('div');
    line.className = `log-line level-${record.level}`;
    const time = document.createElement('span');
    time.className = 'log-time';
    time.textContent = formatTimestamp(record.ts);
    const level = document.createElement('span');
    level.className = 'log-level';
    level.textContent = LEVEL_TEXT[record.level];
    const msg = document.createElement('span');
    msg.className = 'log-msg';
    msg.textContent = record.source ? `[${record.source}] ${record.msg}` : record.msg;
    line.append(time, level, msg);
    return line;
  }

  _applyVisibility(node, level) {
    if (!node) return;
    const visible = this.filter === 'all' || this.filter === level;
    node.classList.toggle('hidden', !visible);
  }

  setFilter(filter) {
    this.filter = ['all', 'info', 'warn', 'error'].includes(filter) ? filter : 'all';
    if (this.filterEl && this.filterEl.value !== this.filter) {
      this.filterEl.value = this.filter;
    }
    this.entries.forEach((entry, index) => this._applyVisibility(this.nodes[index], entry.level));
    if (!this.paused) this._scrollToBottom();
  }

  togglePause() {
    this.paused = !this.paused;
    if (this.pauseBtn) {
      this.pauseBtn.setAttribute('aria-pressed', this.paused ? 'true' : 'false');
      setText(this.pauseBtn, this.paused ? '继续滚动' : '暂停滚动');
    }
    if (!this.paused) this._scrollToBottom();
  }

  toggleCollapse() {
    this.collapsed = !this.collapsed;
    this.panelEl?.classList.toggle('collapsed', this.collapsed);
    if (this.toggleBtn) this.toggleBtn.setAttribute('aria-expanded', this.collapsed ? 'false' : 'true');
    if (!this.collapsed) this._scrollToBottom();
  }

  clear() {
    this.entries = [];
    this.nodes = [];
    if (this.listEl) this.listEl.textContent = '';
    if (this.countEl) this.countEl.textContent = '0';
    this._renderEmpty();
  }

  /** @param {{ts?:number,line:string}[]} lines */
  addSessionLogs(lines) {
    if (!Array.isArray(lines)) return;
    for (const item of lines) {
      const text = typeof item === 'string' ? item : item && item.line;
      if (!text) continue;
      this.add({
        ts: Number.isFinite(item && item.ts) ? item.ts : Date.now() / 1000,
        level: 'info',
        msg: String(text),
        source: '会话',
      });
    }
  }

  download() {
    if (this.onDownload) {
      this.onDownload();
      return;
    }
    const text = this.entries
      .map((entry) => `[${formatTimestamp(entry.ts)}] [${LEVEL_TEXT[entry.level]}] ${entry.msg}`)
      .join('\n');
    const blob = new Blob([text || '（无日志）\n'], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const stamp = formatTimestamp(Date.now() / 1000).replace(/:/g, '');
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = `raspbot-console-${stamp}.log`;
    document.body.append(anchor);
    anchor.click();
    anchor.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  _scrollToBottom() {
    if (!this.bodyEl) return;
    this.bodyEl.scrollTop = this.bodyEl.scrollHeight;
  }

  _renderEmpty() {
    if (!this.emptyEl) return;
    setHidden(this.emptyEl, this.entries.length > 0);
  }
}

/* ------------------------------------------------------------------------- */
/* 地图列表                                                                  */
/* ------------------------------------------------------------------------- */

function previewSrcFor(map) {
  let src = typeof map.preview_url === 'string' && map.preview_url.startsWith('/')
    ? map.preview_url
    : mapPreviewUrl(map.name);
  if (!src.includes('?')) {
    const version = Number.isFinite(map.mtime) ? map.mtime : 0;
    src += `?v=${encodeURIComponent(version)}`;
  }
  return src;
}

/**
 * 渲染地图列表。
 * @param {object} options {listEl, emptyEl, maps, selectedName, onSelect}
 */
export function renderMapList(options = {}) {
  const { listEl, emptyEl, maps = [], selectedName = null, onSelect } = options;
  if (!listEl) return;
  listEl.textContent = '';
  if (!Array.isArray(maps) || maps.length === 0) {
    setHidden(emptyEl, false);
    return;
  }
  setHidden(emptyEl, true);

  for (const map of maps) {
    if (!map || typeof map.name !== 'string') continue;
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'map-item';
    button.disabled = map.valid === false;
    button.title = map.valid === false
      ? `地图 ${map.name} 文件不完整，不可用于导航`
      : `选择地图 ${map.name} 用于导航`;
    const selected = selectedName === map.name;
    button.classList.toggle('selected', selected);
    button.setAttribute('aria-pressed', selected ? 'true' : 'false');

    if (map.valid === false) {
      const placeholder = document.createElement('span');
      placeholder.className = 'map-thumb-placeholder';
      placeholder.textContent = '无效';
      button.append(placeholder);
    } else {
      const img = document.createElement('img');
      img.className = 'map-thumb';
      img.alt = `地图 ${map.name} 预览`;
      img.loading = 'lazy';
      img.decoding = 'async';
      img.src = previewSrcFor(map);
      img.addEventListener('error', () => {
        const placeholder = document.createElement('span');
        placeholder.className = 'map-thumb-placeholder';
        placeholder.textContent = '无预览';
        img.replaceWith(placeholder);
      });
      button.append(img);
    }

    const info = document.createElement('span');
    info.className = 'map-info';
    const name = document.createElement('span');
    name.className = 'map-name';
    name.textContent = map.name;
    const meta = document.createElement('span');
    meta.className = 'map-meta';
    const resolution = Number.isFinite(map.resolution) ? `${formatNumber(map.resolution, 3)} m/px` : '—';
    const mtime = formatTimestamp(map.mtime);
    meta.textContent = `${resolution} · ${mtime}`;
    info.append(name, meta);
    button.append(info);

    button.addEventListener('click', () => {
      if (typeof onSelect === 'function') onSelect(map);
    });
    item.append(button);
    listEl.append(item);
  }
  if (listEl.children.length === 0) setHidden(emptyEl, false);
}

/* ------------------------------------------------------------------------- */
/* 设置/帮助静态内容                                                         */
/* ------------------------------------------------------------------------- */

/** 用 config.SHORTCUTS 填充帮助弹窗的快捷键表（单一数据源）。 */
export function fillShortcutList(listEl, shortcuts = SHORTCUTS) {
  if (!listEl) return;
  listEl.textContent = '';
  for (const item of shortcuts) {
    const dt = document.createElement('dt');
    dt.textContent = item.keys;
    const dd = document.createElement('dd');
    dd.textContent = item.desc;
    listEl.append(dt, dd);
  }
}

/** 图层默认 checkbox 与配置/图层的同步（设置弹窗）。 */
export function readLayerSettings() {
  const layers = {};
  for (const item of LAYER_SETTINGS) {
    const checkbox = $el(`set-layer-${item.key}`);
    layers[item.key] = checkbox ? checkbox.checked : true;
  }
  return layers;
}

export function syncLayerSettings(layers) {
  for (const item of LAYER_SETTINGS) {
    const checkbox = $el(`set-layer-${item.key}`);
    if (checkbox) checkbox.checked = Boolean(layers[item.key]);
  }
}
