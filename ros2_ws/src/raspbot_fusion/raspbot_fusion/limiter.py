"""VelocityLimiter —— /cmd_vel → /cmd_vel_safe 的纯逻辑限速器。

设计目标（离线可测：不 import rclpy / sensor_msgs，只用标准库）：
  1. 一阶斜率限幅：输出变化率 ≤ max_lin_accel / max_ang_accel（FR-E-04：阶跃无突跳）；
  2. 幅值夹紧：|vx|,|vy| ≤ max_linear（IMU 掉线时 ≤ degraded_max_linear），
     |wz| ≤ max_angular（FR-E-04：输出 ≤ max_linear）；
  3. estop 锁存：true 后输出恒零，直到显式 false（CONTRACTS §2 /estop 锁存语义）；
  4. 输入超时看门狗（P0）：指令超过 input_timeout 未更新 → 输出瞬时归零。
     防止 limiter 30 Hz 持续转发最后一条非零指令、使底盘 0.5 s 看门狗永不触发
     （J-07/SW-F01：三级停车从最后有效指令起算 ≤0.5 s，input_timeout=0.4 s
     留出 mux→limiter→base 两级各约 33 ms 传播时间，总链 ≈0.47 s）；
  5. dt 异常保护：dt ≤ 0 或 > 0.5（含 None/NaN/非数值）按 0 处理，状态不跳变、
     不抛异常（SW-U02 精神）。

处理顺序：estop 锁存（恒零，最高优先级）→ 输入超时（**瞬时**归零）→
斜率限幅 → 幅值夹紧。安全归零（estop / 输入超时）为瞬时跳变，不受斜率限幅
约束（SW-F01 判据 ≤0.5 s：0.4 s 超时触发后不允许再花时间平滑减速）；斜率限幅
只作用于正常指令目标的变化（含超时/急停解除后的从零恢复加速）。
"""

import math

__all__ = ['VelocityLimiter']


def _sanitize_dt(dt):
    """dt ≤ 0 或 > 0.5（含 None/NaN/非数值）一律按 0 处理；dt=0.5 恰好合法。"""
    try:
        dt = float(dt)
    except (TypeError, ValueError):
        return 0.0
    if not (0.0 < dt <= 0.5):  # NaN 在此判 False → 归 0
        return 0.0
    return dt


def _finite_or_zero(value):
    """数值防护：非数值/NaN/inf 按 0 处理（安全方向：宁停勿乱）。"""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) else 0.0


def _clamp(value, low, high):
    return max(low, min(high, value))


class VelocityLimiter:
    """一阶斜率限幅 + 幅值夹紧 + estop 锁存 + 输入超时看门狗。

    单位：线速度 m/s、角速度 rad/s、加速度 m/s² 与 rad/s²、时间 s。
    """

    def __init__(self, max_linear=0.30, max_angular=1.5, max_lin_accel=0.6,
                 max_ang_accel=2.0, degraded_max_linear=0.15, imu_timeout=1.0,
                 input_timeout=0.4):
        limits = {
            'max_linear': max_linear,
            'max_angular': max_angular,
            'max_lin_accel': max_lin_accel,
            'max_ang_accel': max_ang_accel,
            'degraded_max_linear': degraded_max_linear,
            'imu_timeout': imu_timeout,
            'input_timeout': input_timeout,
        }
        for name, value in limits.items():
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError('%s 必须是数值，收到 %r' % (name, value))
            if not (value > 0.0) or not math.isfinite(value):
                raise ValueError('%s 必须为正数，收到 %r' % (name, value))
            setattr(self, name, value)

        self._estop_latched = False
        self._prev = (0.0, 0.0, 0.0)  # 上一拍输出（限幅内部状态）

    # ---------- 状态判定 ----------
    @property
    def estop_latched(self):
        return self._estop_latched

    def imu_stale(self, imu_age_sec):
        """IMU 是否视为掉线：age > imu_timeout（None/NaN/非数值视为掉线）。"""
        if imu_age_sec is None:
            return True
        try:
            imu_age_sec = float(imu_age_sec)
        except (TypeError, ValueError):
            return True
        return not (imu_age_sec <= self.imu_timeout)

    def input_stale(self, input_age_sec):
        """指令是否视为超时：age > input_timeout（None=从未收到，视为超时）。"""
        if input_age_sec is None:
            return True
        try:
            input_age_sec = float(input_age_sec)
        except (TypeError, ValueError):
            return True
        return not (input_age_sec <= self.input_timeout)

    # ---------- estop ----------
    def estop(self, active):
        """true：锁存（此后输出恒零，直到显式 false）；false：解除。"""
        self._estop_latched = bool(active)
        if self._estop_latched:
            self._prev = (0.0, 0.0, 0.0)  # 解除后从零平滑起步，无突跳

    # ---------- 主逻辑 ----------
    def step(self, target_vx, target_vy, target_wz, dt, imu_age_sec,
             input_age_sec=None):
        """执行一拍限幅，返回 (vx, vy, wz)。"""
        dt = _sanitize_dt(dt)

        # 0) estop 锁存：恒零，优先级最高
        if self._estop_latched:
            self._prev = (0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)

        # 1) 输入超时：安全归零必须瞬时（J-07/SW-F01：从最后有效指令起算
        #    ≤0.5 s，0.4 s 超时触发后不允许再花时间减速），不受斜率限幅约束；
        #    清零内部状态，恢复指令后从零平滑加速。
        if self.input_stale(input_age_sec):
            self._prev = (0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)

        tvx = _finite_or_zero(target_vx)
        tvy = _finite_or_zero(target_vy)
        twz = _finite_or_zero(target_wz)

        # 2) 一阶斜率限幅（逐轴）
        pvx, pvy, pwz = self._prev
        max_dv = self.max_lin_accel * dt
        max_dw = self.max_ang_accel * dt
        vx = pvx + _clamp(tvx - pvx, -max_dv, max_dv)
        vy = pvy + _clamp(tvy - pvy, -max_dv, max_dv)
        wz = pwz + _clamp(twz - pwz, -max_dw, max_dw)

        # 3) 幅值夹紧（IMU 掉线 → 线性上限降为 degraded_max_linear）
        if self.imu_stale(imu_age_sec):
            vmax = self.degraded_max_linear
        else:
            vmax = self.max_linear
        vx = _clamp(vx, -vmax, vmax)
        vy = _clamp(vy, -vmax, vmax)
        wz = _clamp(wz, -self.max_angular, self.max_angular)

        self._prev = (vx, vy, wz)
        return (vx, vy, wz)
