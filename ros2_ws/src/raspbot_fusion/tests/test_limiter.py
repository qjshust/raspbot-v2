"""VelocityLimiter 离线单元测试（pytest，无 rclpy/sensor_msgs 依赖）。

覆盖（追溯）：
- 阶跃限幅：dt 步进无突跳、每步增量 ≤ 设定加速度、解析解一致（FR-E-04）
- 幅值夹紧：|v| ≤ max_linear（IMU 正常/掉线 degraded）、|wz| ≤ max_angular（FR-E-04）
- estop 锁存/解除：true 恒零、false 从零平滑恢复（CONTRACTS §2）
- IMU 掉线降级上限 + 恢复（FR-E-02 / SYS-S03）
- 输入超时看门狗：/cmd_vel 静默 → 瞬时归零、恢复平滑加速（P0 / J-07/SW-F01）
- dt 异常保护：≤0 / >0.5 / None / NaN 按 0 处理，不抛异常（SW-U02 精神）

运行：python -m pytest tests -q
"""

import pytest

from raspbot_fusion.limiter import VelocityLimiter


def make_limiter(**overrides):
    params = dict(max_linear=0.30, max_angular=1.5, max_lin_accel=0.6,
                  max_ang_accel=2.0, degraded_max_linear=0.15,
                  imu_timeout=1.0, input_timeout=0.4)
    params.update(overrides)
    return VelocityLimiter(**params)


DT = 0.05          # 步长（20 Hz，与节点 30 Hz 同量级）
MAX_DV = 0.6 * DT  # 每步最大线速度增量 0.03 m/s
MAX_DW = 2.0 * DT  # 每步最大角速度增量 0.1 rad/s


# ---------------------------------------------------------------------------
# FR-E-04：阶跃限幅（无突跳、加速度不超过设定）
# ---------------------------------------------------------------------------
class TestSlewLimit:
    def test_step_increments_never_exceed_accel(self):
        lim = make_limiter()
        prev = (0.0, 0.0, 0.0)
        for _ in range(60):
            out = lim.step(0.3, 0.0, 1.5, DT, 0.0, 0.0)
            assert abs(out[0] - prev[0]) <= MAX_DV + 1e-12
            assert abs(out[1] - prev[1]) <= MAX_DV + 1e-12
            assert abs(out[2] - prev[2]) <= MAX_DW + 1e-12
            prev = out
        assert prev == pytest.approx((0.3, 0.0, 1.5), abs=1e-9)

    def test_step_matches_analytic_ramp(self):
        """解析断言：第 n 步 vx = min(0.3, n * max_lin_accel * dt)。"""
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for n in range(1, 12):
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
            assert out[0] == pytest.approx(min(0.3, n * MAX_DV), abs=1e-12)
        for _ in range(5):  # 收敛后保持
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(0.3, abs=1e-12)

    def test_no_jump_on_step_input(self):
        """FR-E-04：目标从 0 阶跃到最大值，第一拍输出不突跳。"""
        lim = make_limiter()
        out = lim.step(0.3, 0.0, 1.5, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(MAX_DV, abs=1e-12)
        assert out[2] == pytest.approx(MAX_DW, abs=1e-12)

    def test_negative_directions_and_vy(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(20):
            out = lim.step(-0.3, -0.2, -1.5, DT, 0.0, 0.0)
        assert out == pytest.approx((-0.3, -0.2, -1.5), abs=1e-9)


# ---------------------------------------------------------------------------
# FR-E-04：幅值夹紧
# ---------------------------------------------------------------------------
class TestAmplitudeClamp:
    def test_linear_and_angular_steady_state(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(200):
            out = lim.step(5.0, 5.0, 9.0, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(0.30, abs=1e-9)
        assert out[1] == pytest.approx(0.30, abs=1e-9)
        assert out[2] == pytest.approx(1.5, abs=1e-9)

    def test_clamp_holds_with_large_legal_dt(self):
        """dt=0.5（最大合法值）单步最多 0.3，仍需夹紧且不越界。"""
        lim = make_limiter()
        out = lim.step(5.0, 0.0, 0.0, 0.5, 0.0, 0.0)
        assert out[0] == pytest.approx(0.30, abs=1e-12)
        assert out[2] == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# CONTRACTS §2：estop 锁存 / 解除
# ---------------------------------------------------------------------------
class TestEstop:
    def test_latch_outputs_zero_until_released(self):
        lim = make_limiter()
        lim.estop(True)
        for _ in range(10):
            out = lim.step(0.3, 0.2, 1.5, DT, 0.0, 0.0)
            assert out == (0.0, 0.0, 0.0)
        assert lim.estop_latched
        lim.estop(True)  # 重复 true 仍锁存
        assert lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0) == (0.0, 0.0, 0.0)

    def test_release_resumes_smoothly_from_zero(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(20):  # 先跑到满速
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(0.3, abs=1e-9)
        lim.estop(True)
        assert lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0) == (0.0, 0.0, 0.0)
        lim.estop(False)
        assert lim.estop_latched is False
        out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        # 解除后第一拍从零平滑起步，无突跳
        assert 0.0 < out[0] <= MAX_DV + 1e-12


# ---------------------------------------------------------------------------
# FR-E-02 / SYS-S03：IMU 掉线降级 + 恢复
# ---------------------------------------------------------------------------
class TestImuDegradation:
    def test_full_limit_when_fresh(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(200):
            out = lim.step(1.0, 0.0, 0.0, DT, 0.1, 0.0)
        assert out[0] == pytest.approx(0.30, abs=1e-9)

    def test_degraded_limit_when_stale(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(200):
            out = lim.step(1.0, 0.0, 0.0, DT, 1.5, 0.0)
        assert out[0] == pytest.approx(0.15, abs=1e-9)

    def test_recovery_restores_full_limit(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(200):
            out = lim.step(1.0, 0.0, 0.0, DT, 1.5, 0.0)
        assert out[0] == pytest.approx(0.15, abs=1e-9)
        for _ in range(200):
            out = lim.step(1.0, 0.0, 0.0, DT, 0.1, 0.0)
        assert out[0] == pytest.approx(0.30, abs=1e-9)

    def test_stale_boundary_and_bad_values(self):
        lim = make_limiter()
        assert not lim.imu_stale(1.0)           # 恰好 1.0 s：不超时
        assert lim.imu_stale(1.0 + 1e-9)
        assert lim.imu_stale(None)              # 从未收到 → 视为掉线
        assert lim.imu_stale(float('nan'))      # NaN 按掉线（安全方向）
        assert lim.imu_stale('x')               # 非数值按掉线


# ---------------------------------------------------------------------------
# SW-U02 精神：dt 异常保护
# ---------------------------------------------------------------------------
class TestDtProtection:
    def test_invalid_dt_keeps_state_unchanged(self):
        lim = make_limiter()
        out = (0.0, 0.0, 0.0)
        for _ in range(10):
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(0.3, abs=1e-9)
        prev = out
        # 目标为 0：若 bad dt 被当作有效时间，输出会立刻减速；按 0 处理则保持
        for bad in (0.0, -0.1, 0.5001, 1.0, 100.0, None, float('nan'), 'x'):
            out = lim.step(0.0, 0.0, 0.0, bad, 0.0, 0.0)
            assert out == pytest.approx(prev, abs=1e-12)
        # dt=0.5 恰好合法：0.3 - 0.6*0.5 = 0
        out = lim.step(0.0, 0.0, 0.0, 0.5, 0.0, 0.0)
        assert out[0] == pytest.approx(0.0, abs=1e-12)

    def test_no_exception_on_bad_dt(self):
        lim = make_limiter()
        for bad in (0.0, -1.0, 1.0, None, float('nan'), 'x'):
            out = lim.step(0.1, 0.0, 0.0, bad, 0.0, 0.0)  # 不抛异常
            assert out[0] == 0.0  # dt 按 0 → 状态保持 0


# ---------------------------------------------------------------------------
# P0 输入超时看门狗（lead 补充）：/cmd_vel 静默 → 瞬时归零（J-07/SW-F01 三级停车）
# ---------------------------------------------------------------------------
class TestInputTimeout:
    def test_silence_after_1s_zeroes_output(self):
        """持续喂目标 1 s 后停止喂 → 超时一步瞬时归零并保持。"""
        lim = make_limiter(input_timeout=0.4)
        out = (0.0, 0.0, 0.0)
        for _ in range(20):  # 持续喂 1 s（input_age=0 表示刚收到指令）
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, input_age_sec=0.0)
        assert out[0] == pytest.approx(0.3, abs=1e-9)
        age = 0.0
        for _ in range(60):  # 停止喂：input_age 每拍递增，模拟时间流逝
            age += DT
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, input_age_sec=age)
        assert out == (0.0, 0.0, 0.0)

    def test_timeout_starts_only_after_0_4s(self):
        lim = make_limiter(input_timeout=0.4)
        out = (0.0, 0.0, 0.0)
        for _ in range(20):
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        # age 恰 0.4 s：未超时，继续跟随目标
        out = lim.step(0.3, 0.0, 0.0, DT, 0.0, input_age_sec=0.4)
        assert out[0] == pytest.approx(0.3, abs=1e-12)
        # age > 0.4 s：瞬时归零（安全路径，不经斜率限幅）
        out2 = lim.step(0.3, 0.0, 0.0, DT, 0.0, input_age_sec=0.4 + 1e-9)
        assert out2 == (0.0, 0.0, 0.0)

    def test_immediate_zero_at_threshold_crossing(self):
        """0.2 m/s 目标：input_age 刚过阈值的一步，输出直接为 0（无中间值）。"""
        lim = make_limiter(input_timeout=0.4)
        out = (0.0, 0.0, 0.0)
        for _ in range(20):
            out = lim.step(0.2, 0.0, 0.0, DT, 0.0, 0.0)
        assert out[0] == pytest.approx(0.2, abs=1e-9)
        out = lim.step(0.2, 0.0, 0.0, DT, 0.0, input_age_sec=0.4 + 1e-9)
        assert out == (0.0, 0.0, 0.0)  # 无 0.2→0.17→… 的中间斜坡
        # 超时期间持续恒零（即使目标保持非零）
        for age in (0.5, 1.0, 2.0):
            assert lim.step(0.2, 0.0, 0.0, DT, 0.0, age) == (0.0, 0.0, 0.0)

    def test_never_received_input_is_stale(self):
        """从未收到 /cmd_vel：视为超时，启动即输出零速。"""
        lim = make_limiter()
        assert lim.input_stale(None)
        out = lim.step(0.3, 0.0, 0.0, DT, 0.0, input_age_sec=None)
        assert out == (0.0, 0.0, 0.0)
        assert lim.input_stale(0.4) is False        # 边界：恰好 0.4 不超时
        assert lim.input_stale(0.4 + 1e-9) is True

    def test_recovery_after_timeout(self):
        """超时瞬时归零后恢复指令：从零平滑加速（斜率限幅仍生效）。"""
        lim = make_limiter(input_timeout=0.4)
        out = (0.0, 0.0, 0.0)
        for _ in range(20):
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
        age = 0.45
        for _ in range(40):  # 超时瞬时归零并保持
            age += DT
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, age)
        assert out == (0.0, 0.0, 0.0)
        prev = 0.0
        for _ in range(20):  # 恢复喂指令：平滑加速回 0.3（每步 ≤ MAX_DV）
            out = lim.step(0.3, 0.0, 0.0, DT, 0.0, 0.0)
            assert out[0] - prev <= MAX_DV + 1e-12
            prev = out[0]
        assert out[0] == pytest.approx(0.3, abs=1e-9)

    def test_estop_overrides_input_timeout(self):
        """estop 锁存优先级高于输入超时。"""
        lim = make_limiter()
        lim.estop(True)
        for age in (0.0, 0.4, 1.0, None):
            assert lim.step(0.3, 0.0, 0.0, DT, 0.0, age) == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# 非法参数拒绝（SW-U15 精神）
# ---------------------------------------------------------------------------
class TestParamValidation:
    @pytest.mark.parametrize('kwargs', [
        dict(max_linear=0.0), dict(max_linear=-1.0), dict(max_linear='x'),
        dict(max_angular=0.0), dict(max_lin_accel=0.0), dict(max_ang_accel=-2.0),
        dict(degraded_max_linear=0.0), dict(imu_timeout=0.0), dict(imu_timeout=-1.0),
        dict(input_timeout=0.0), dict(input_timeout=-0.4),
        dict(input_timeout=None), dict(max_linear=float('inf')),
    ])
    def test_invalid_params_raise_valueerror(self, kwargs):
        with pytest.raises(ValueError):
            make_limiter(**kwargs)
