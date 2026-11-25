/**
 * teleop.js —— 遥控控制器。
 *
 * 功能（§6.2 / §3.3.4）：
 *   - 键盘 WASD 前后/横移、Q/E 自转、Shift 加速（不超过 state.limits）；
 *   - 虚拟摇杆支持鼠标/触摸，松手回中并立即停止；
 *   - 仅在「会话 running 且非急停且已连接」时允许，20 Hz 发送 POST /api/teleop；
 *   - 松开全部输入、窗口失焦、切换标签页、禁用时立即 POST /api/teleop/stop（松手即停）；
 *   - 导航中首次遥控先弹「将取消导航目标」确认（网关也会自动取消，这里做安全提示）。
 *
 * 该模块只负责输入与节流，安全判定由 app 通过 getContext() 注入。
 */

import { TELEOP } from './config.js';
import { clamp, clampVelocity, joystickVelocity, keyVelocity } from './math.js';

const MOVEMENT_KEYS = new Set([
  'w', 'a', 's', 'd', 'q', 'e', 'shift', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright',
]);

function normalizeKey(event) {
  const raw = typeof event.key === 'string' ? event.key.toLowerCase() : '';
  if (raw === 'shift') return 'shift';
  if (raw === ' ') return 'space';
  return raw;
}

export class TeleopController {
  /**
   * @param {object} api createApi() 返回值
   * @param {object} options
   *   joystickEl, knobEl, speedSliderEl, speedValueEl, stopBtnEl, keyEls
   *   getContext(): {enabled, reason, limits, navHasGoal}
   *   confirmNavCancel(): Promise<boolean>
   *   onActiveChange(active, velocity)
   *   onSpeedChange(scale)
   *   onError(message)
   *   onBlocked(reason)
   *   isInputBlocked(): boolean  弹窗打开时返回 true，不抢键盘
   */
  constructor(api, options = {}) {
    this.api = api;
    this.options = options;
    this.keys = new Set();
    this.joystick = { pointerId: null, x: 0, y: 0, magnitude: 0 };
    this.speedScale = clamp(
      Number.isFinite(options.speedScale) ? options.speedScale : TELEOP.speedScaleDefault,
      TELEOP.speedScaleMin,
      TELEOP.speedScaleMax,
    );
    this.limits = { maxLinear: 0.3, maxLateral: 0.3, maxAngular: 1.5 };
    this.enabled = false;
    this.blockedReason = '';
    this.active = false;
    this.seq = 0;
    this.navCancelConfirmed = false;
    this.navConfirmPending = false;
    this.suppressUntilRelease = false;
    this.lastErrorToastAt = 0;
    this.lastBlockedToastAt = 0;
    this.intervalId = null;
    this.sending = false;
    this._sentSomething = false;
    this.destroyed = false;
    this._bound = {
      keydown: (event) => this._onKeyDown(event),
      keyup: (event) => this._onKeyUp(event),
      blur: () => this.stopInputs({ sendStop: true }),
      visibility: () => {
        if (document.visibilityState === 'hidden') this.stopInputs({ sendStop: true });
      },
      joystickDown: (event) => this._onJoystickDown(event),
      joystickMove: (event) => this._onJoystickMove(event),
      joystickUp: (event) => this._onJoystickUp(event),
      speedInput: () => this._onSpeedInput(),
      stopClick: () => this.stopInputs({ sendStop: true }),
    };
    this.started = false;
  }

  start() {
    if (this.started) return;
    this.started = true;
    window.addEventListener('keydown', this._bound.keydown, { passive: false });
    window.addEventListener('keyup', this._bound.keyup);
    window.addEventListener('blur', this._bound.blur);
    document.addEventListener('visibilitychange', this._bound.visibility);

    const { joystickEl, speedSliderEl, stopBtnEl } = this.options;
    if (joystickEl) {
      joystickEl.addEventListener('pointerdown', this._bound.joystickDown);
      joystickEl.addEventListener('pointermove', this._bound.joystickMove);
      joystickEl.addEventListener('pointerup', this._bound.joystickUp);
      joystickEl.addEventListener('pointercancel', this._bound.joystickUp);
    }
    if (speedSliderEl) {
      speedSliderEl.value = String(Math.round(this.speedScale * 100));
      speedSliderEl.addEventListener('input', this._bound.speedInput);
    }
    if (stopBtnEl) stopBtnEl.addEventListener('click', this._bound.stopClick);

    this._updateSpeedLabel();
    this._updateJoystickDisabled();
    this.intervalId = setInterval(() => this._tick(), TELEOP.intervalMs);
  }

  destroy() {
    this.destroyed = true;
    if (this.intervalId) clearInterval(this.intervalId);
    this.intervalId = null;
    if (!this.started) return;
    window.removeEventListener('keydown', this._bound.keydown);
    window.removeEventListener('keyup', this._bound.keyup);
    window.removeEventListener('blur', this._bound.blur);
    document.removeEventListener('visibilitychange', this._bound.visibility);
    this.started = false;
  }

  setEnabled(enabled, reason = '') {
    const next = Boolean(enabled);
    const changed = this.enabled !== next || this.blockedReason !== reason;
    this.enabled = next;
    this.blockedReason = next ? '' : reason || '当前不可遥控';
    this._updateJoystickDisabled();
    if (!next && changed) this.stopInputs({ sendStop: true });
  }

  setLimits(limits) {
    if (!limits) return;
    this.limits = {
      maxLinear: Number.isFinite(limits.maxLinear) ? limits.maxLinear : this.limits.maxLinear,
      maxLateral: Number.isFinite(limits.maxLateral) ? limits.maxLateral : this.limits.maxLateral,
      maxAngular: Number.isFinite(limits.maxAngular) ? limits.maxAngular : this.limits.maxAngular,
    };
    this._updateSpeedLabel();
  }

  setSpeedScale(scale, { notify = true } = {}) {
    this.speedScale = clamp(scale, TELEOP.speedScaleMin, TELEOP.speedScaleMax);
    const slider = this.options.speedSliderEl;
    if (slider) slider.value = String(Math.round(this.speedScale * 100));
    this._updateSpeedLabel();
    if (notify) this.options.onSpeedChange?.(this.speedScale);
  }

  isActive() {
    return this.active;
  }

  /** 清空所有输入；必要时立即发送零速（松手即停）。 */
  stopInputs({ sendStop = true } = {}) {
    const wasActive = this.active;
    this.keys.clear();
    this._clearKeyIndicators();
    this._resetJoystick();
    this.active = false;
    this.navCancelConfirmed = false;
    this.suppressUntilRelease = false;
    this._notifyActive();
    if (sendStop && (wasActive || this._sentSomething)) {
      this._sendStop();
    }
  }

  /** 会话停止/急停/断连时调用：立即零速并停发。 */
  stop() {
    this.stopInputs({ sendStop: true });
  }

  /* --------------------------------------------------------------------- */
  /* 键盘                                                                  */
  /* --------------------------------------------------------------------- */

  _onKeyDown(event) {
    if (this.destroyed) return;
    if (this.options.isInputBlocked?.()) return;
    if (this._isTypingTarget(event.target)) return;
    const key = normalizeKey(event);
    if (!MOVEMENT_KEYS.has(key)) return;
    if (event.isComposing || event.repeat) {
      if (MOVEMENT_KEYS.has(key)) event.preventDefault();
      return;
    }
    const context = this._context();
    if (!context.enabled) {
      this._notifyBlocked(context.reason);
      return;
    }
    event.preventDefault();
    this.keys.add(key);
    this._updateKeyIndicator(key, true);
  }

  _onKeyUp(event) {
    const key = normalizeKey(event);
    if (!MOVEMENT_KEYS.has(key)) return;
    this.keys.delete(key);
    this._updateKeyIndicator(key, false);
    if (!this._hasMovementInput()) {
      // 松手即停：不等待下一个 tick
      if (this.active) {
        this.active = false;
        this._notifyActive();
        this._sendStop();
      }
    }
  }

  /** 输入框/文本域聚焦时不抢键盘（避免在数值输入里打字触发遥控）。 */
  _isTypingTarget(target) {
    if (!target || typeof target.tagName !== 'string') return false;
    const tag = target.tagName.toUpperCase();
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || target.isContentEditable === true;
  }

  _hasMovementInput() {
    if (this.joystick.magnitude > 0.001) return true;
    return Array.from(this.keys).some((key) => key !== 'shift');
  }

  _updateKeyIndicator(key, pressed) {
    const map = this.options.keyEls;
    if (!map) return;
    const el = map[key] || (key.startsWith('arrow') ? map[{ arrowup: 'w', arrowdown: 's', arrowleft: 'a', arrowright: 'd' }[key]] : null);
    if (el) el.classList.toggle('active', pressed);
  }

  _clearKeyIndicators() {
    const map = this.options.keyEls;
    if (!map) return;
    for (const el of Object.values(map)) el?.classList.remove('active');
  }

  /* --------------------------------------------------------------------- */
  /* 虚拟摇杆                                                              */
  /* --------------------------------------------------------------------- */

  _onJoystickDown(event) {
    if (this.destroyed) return;
    const context = this._context();
    if (!context.enabled) {
      this._notifyBlocked(context.reason);
      return;
    }
    event.preventDefault();
    this.joystick.pointerId = event.pointerId;
    try {
      this.options.joystickEl?.setPointerCapture(event.pointerId);
    } catch {
      /* 忽略 */
    }
    this._updateJoystickFromEvent(event);
  }

  _onJoystickMove(event) {
    if (this.joystick.pointerId !== event.pointerId) return;
    this._updateJoystickFromEvent(event);
  }

  _onJoystickUp(event) {
    if (this.joystick.pointerId !== event.pointerId) return;
    try {
      this.options.joystickEl?.releasePointerCapture(event.pointerId);
    } catch {
      /* 忽略 */
    }
    this._resetJoystick();
    if (!this._hasMovementInput() && this.active) {
      this.active = false;
      this._notifyActive();
      this._sendStop();
    }
  }

  _updateJoystickFromEvent(event) {
    const el = this.options.joystickEl;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const maxRadius = Math.max(10, rect.width / 2 - 14);
    let dx = event.clientX - centerX;
    let dy = event.clientY - centerY;
    const length = Math.hypot(dx, dy);
    if (length > maxRadius) {
      dx = (dx / length) * maxRadius;
      dy = (dy / length) * maxRadius;
    }
    this.joystick.x = dx / maxRadius;
    this.joystick.y = dy / maxRadius;
    this.joystick.magnitude = Math.min(1, Math.hypot(this.joystick.x, this.joystick.y));
    this._applyJoystickKnob(dx, dy);
  }

  _applyJoystickKnob(dx, dy) {
    const knob = this.options.knobEl;
    if (!knob) return;
    knob.style.transform = `translate(${dx.toFixed(1)}px, ${dy.toFixed(1)}px)`;
  }

  _resetJoystick() {
    this.joystick = { pointerId: null, x: 0, y: 0, magnitude: 0 };
    this._applyJoystickKnob(0, 0);
  }

  _updateJoystickDisabled() {
    this.options.joystickEl?.classList.toggle('disabled', !this.enabled);
  }

  /* --------------------------------------------------------------------- */
  /* 速度滑杆                                                              */
  /* --------------------------------------------------------------------- */

  _onSpeedInput() {
    const slider = this.options.speedSliderEl;
    if (!slider) return;
    this.setSpeedScale(Number(slider.value) / 100);
    this._sendStopIfActive();
  }

  _sendStopIfActive() {
    if (this.active) {
      this.active = false;
      this._notifyActive();
      this._sendStop();
    }
  }

  _updateSpeedLabel() {
    const el = this.options.speedValueEl;
    if (!el) return;
    el.textContent = `${(this.limits.maxLinear * this.speedScale).toFixed(2)} m/s`;
  }

  /* --------------------------------------------------------------------- */
  /* 20 Hz 发送                                                            */
  /* --------------------------------------------------------------------- */

  _context() {
    const context = this.options.getContext?.() || {};
    return {
      enabled: Boolean(context.enabled),
      reason: context.reason || '当前不可遥控',
      limits: context.limits || this.limits,
      navHasGoal: Boolean(context.navHasGoal),
    };
  }

  _desiredVelocity() {
    const joystickActive = this.joystick.magnitude > 0.02;
    const boost = this.keys.has('shift');
    const options = {
      maxLinear: this.limits.maxLinear,
      maxLateral: this.limits.maxLateral,
      maxAngular: this.limits.maxAngular,
      scale: this.speedScale,
      boost,
      deadzone: TELEOP.deadzone,
    };
    if (joystickActive) {
      const velocity = joystickVelocity(this.joystick.x, this.joystick.y, options);
      return clampVelocity(velocity, this.limits);
    }
    const keys = new Set(this.keys);
    return clampVelocity(keyVelocity(keys, options), this.limits);
  }

  _tick() {
    if (this.destroyed) return;
    const context = this._context();
    if (!context.enabled) {
      if (this.active || this._hasMovementInput()) this.stopInputs({ sendStop: true });
      return;
    }
    if (this.suppressUntilRelease) {
      if (!this._hasMovementInput()) this.suppressUntilRelease = false;
      else return;
    }

    const velocity = this._desiredVelocity();
    const moving = Math.abs(velocity.vx) > 1e-4 || Math.abs(velocity.vy) > 1e-4 || Math.abs(velocity.wz) > 1e-4;

    if (!moving) {
      if (this.active) {
        this.active = false;
        this._notifyActive();
        this._sendStop();
      }
      return;
    }

    // 导航中首次遥控：提示“将取消导航目标”（网关收到 teleop 后也会自动取消）
    if (context.navHasGoal && !this.navCancelConfirmed) {
      if (this.navConfirmPending) return;
      this.navConfirmPending = true;
      this.active = false;
      this._notifyActive();
      Promise.resolve(this.options.confirmNavCancel?.())
        .then((ok) => {
          this.navConfirmPending = false;
          if (ok) {
            this.navCancelConfirmed = true;
          } else {
            this.suppressUntilRelease = true;
          }
        })
        .catch(() => {
          this.navConfirmPending = false;
          this.suppressUntilRelease = true;
        });
      return;
    }

    this._sendTeleop(velocity);
  }

  _sendTeleop(velocity) {
    if (this.sending) return;
    this.sending = true;
    this.seq += 1;
    this._sentSomething = true;
    const wasActive = this.active;
    this.active = true;
    if (!wasActive) this._notifyActive();
    this.api
      .teleop(velocity.vx, velocity.vy, velocity.wz, this.seq)
      .catch((error) => {
        this._reportError(error);
        this.stopInputs({ sendStop: true });
      })
      .finally(() => {
        this.sending = false;
      });
    this.options.onVelocity?.(velocity);
  }

  _sendStop() {
    this._sentSomething = false;
    this.api.teleopStop().catch((error) => {
      // 停止请求失败不阻塞 UI；网关 500 ms 看门狗会兜底零速
      this._reportError(error, { quiet: true });
    });
  }

  _notifyActive() {
    this.options.onActiveChange?.(this.active);
  }

  _notifyBlocked(reason) {
    const now = Date.now();
    if (now - this.lastBlockedToastAt < 1500) return;
    this.lastBlockedToastAt = now;
    this.options.onBlocked?.(reason || '当前不可遥控');
  }

  _reportError(error, { quiet = false } = {}) {
    if (quiet) return;
    const now = Date.now();
    if (now - this.lastErrorToastAt < TELEOP.errorToastIntervalMs) return;
    this.lastErrorToastAt = now;
    const message = error && error.message ? error.message : '遥控指令发送失败';
    this.options.onError?.(message);
  }
}
