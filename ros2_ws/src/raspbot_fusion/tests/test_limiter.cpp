// ============================================================================
// VelocityLimiter gtest 单元测试（C++ 重写版）
//
// 与 tests/test_limiter.py（原 Python 版，33 用例）1:1 映射，逐条复刻断言与
// 容差（EXPECT_NEAR 的 abs 容差 = Python pytest.approx 的 abs）。
//
// 类型映射（仅接口差异，语义等价）：
//   - Python None（从未收到数据）→ C++ kNever(NaN)；
//   - Python 非数值字符串 'x' → C++ 无对应（强类型），NaN 覆盖其全部路径。
//
// 覆盖（追溯）：
// - 阶跃限幅：dt 步进无突跳、每步增量 ≤ 设定加速度、解析解一致（FR-E-04）
// - 幅值夹紧：|v| ≤ max_linear（IMU 正常/掉线 degraded）、|wz| ≤ max_angular
// - estop 锁存/解除：true 恒零、false 从零平滑恢复（CONTRACTS §2）
// - IMU 掉线降级上限 + 恢复（FR-E-02 / SYS-S03）
// - 输入超时看门狗：/cmd_vel 静默 → 瞬时归零、恢复平滑加速（P0 / J-07/SW-F01）
// - dt 异常保护：≤0 / >0.5 / NaN 按 0 处理，不抛异常（SW-U02 精神）
//
// 运行：colcon test --packages-select raspbot_fusion
//      或 ./build/raspbot_fusion/test_limiter
// ============================================================================

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "raspbot_fusion/limiter.hpp"

namespace
{

using raspbot_fusion::Vel;
using raspbot_fusion::VelocityLimiter;

constexpr double kDt = 0.05;            // 步长（与节点 30 Hz 同量级）
constexpr double kMaxDv = 0.6 * kDt;    // 每步最大线速度增量 0.03 m/s
constexpr double kMaxDw = 2.0 * kDt;    // 每步最大角速度增量 0.1 rad/s
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void ExpectVel(const Vel & v, double vx, double vy, double wz, double tol)
{
  EXPECT_NEAR(v.vx, vx, tol);
  EXPECT_NEAR(v.vy, vy, tol);
  EXPECT_NEAR(v.wz, wz, tol);
}

void ExpectVelZero(const Vel & v)
{
  EXPECT_DOUBLE_EQ(v.vx, 0.0);
  EXPECT_DOUBLE_EQ(v.vy, 0.0);
  EXPECT_DOUBLE_EQ(v.wz, 0.0);
}

}  // namespace

// ---------------------------------------------------------------------------
// FR-E-04：阶跃限幅（无突跳、加速度不超过设定）
// ---------------------------------------------------------------------------
class SlewLimitTest : public ::testing::Test
{
};

TEST_F(SlewLimitTest, StepIncrementsNeverExceedAccel)
{
  VelocityLimiter lim;
  Vel prev{0.0, 0.0, 0.0};
  for (int i = 0; i < 60; ++i) {
    const Vel out = lim.step(0.3, 0.0, 1.5, kDt, 0.0, 0.0);
    EXPECT_LE(std::abs(out.vx - prev.vx), kMaxDv + 1e-12);
    EXPECT_LE(std::abs(out.vy - prev.vy), kMaxDv + 1e-12);
    EXPECT_LE(std::abs(out.wz - prev.wz), kMaxDw + 1e-12);
    prev = out;
  }
  ExpectVel(prev, 0.3, 0.0, 1.5, 1e-9);
}

TEST_F(SlewLimitTest, StepMatchesAnalyticRamp)
{
  // 解析断言：第 n 步 vx = min(0.3, n * max_lin_accel * dt)。
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int n = 1; n < 12; ++n) {
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
    EXPECT_NEAR(out.vx, std::min(0.3, n * kMaxDv), 1e-12);
  }
  for (int i = 0; i < 5; ++i) {  // 收敛后保持
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.3, 1e-12);
}

TEST_F(SlewLimitTest, NoJumpOnStepInput)
{
  // FR-E-04：目标从 0 阶跃到最大值，第一拍输出不突跳。
  VelocityLimiter lim;
  const Vel out = lim.step(0.3, 0.0, 1.5, kDt, 0.0, 0.0);
  EXPECT_NEAR(out.vx, kMaxDv, 1e-12);
  EXPECT_NEAR(out.wz, kMaxDw, 1e-12);
}

TEST_F(SlewLimitTest, NegativeDirectionsAndVy)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {
    out = lim.step(-0.3, -0.2, -1.5, kDt, 0.0, 0.0);
  }
  ExpectVel(out, -0.3, -0.2, -1.5, 1e-9);
}

// ---------------------------------------------------------------------------
// FR-E-04：幅值夹紧
// ---------------------------------------------------------------------------
class AmplitudeClampTest : public ::testing::Test
{
};

TEST_F(AmplitudeClampTest, LinearAndAngularSteadyState)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 200; ++i) {
    out = lim.step(5.0, 5.0, 9.0, kDt, 0.0, 0.0);
  }
  ExpectVel(out, 0.30, 0.30, 1.5, 1e-9);
}

TEST_F(AmplitudeClampTest, ClampHoldsWithLargeLegalDt)
{
  // dt=0.5（最大合法值）单步最多 0.3，仍需夹紧且不越界。
  VelocityLimiter lim;
  const Vel out = lim.step(5.0, 0.0, 0.0, 0.5, 0.0, 0.0);
  EXPECT_NEAR(out.vx, 0.30, 1e-12);
  EXPECT_NEAR(out.wz, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// CONTRACTS §2：estop 锁存 / 解除
// ---------------------------------------------------------------------------
class EstopTest : public ::testing::Test
{
};

TEST_F(EstopTest, LatchOutputsZeroUntilReleased)
{
  VelocityLimiter lim;
  lim.estop(true);
  for (int i = 0; i < 10; ++i) {
    ExpectVelZero(lim.step(0.3, 0.2, 1.5, kDt, 0.0, 0.0));
  }
  EXPECT_TRUE(lim.estop_latched());
  lim.estop(true);  // 重复 true 仍锁存
  ExpectVelZero(lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0));
}

TEST_F(EstopTest, ReleaseResumesSmoothlyFromZero)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {  // 先跑到满速
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.3, 1e-9);
  lim.estop(true);
  ExpectVelZero(lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0));
  lim.estop(false);
  EXPECT_FALSE(lim.estop_latched());
  out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  // 解除后第一拍从零平滑起步，无突跳
  EXPECT_GT(out.vx, 0.0);
  EXPECT_LE(out.vx, kMaxDv + 1e-12);
}

// ---------------------------------------------------------------------------
// FR-E-02 / SYS-S03：IMU 掉线降级 + 恢复
// ---------------------------------------------------------------------------
class ImuDegradationTest : public ::testing::Test
{
};

TEST_F(ImuDegradationTest, FullLimitWhenFresh)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 200; ++i) {
    out = lim.step(1.0, 0.0, 0.0, kDt, 0.1, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.30, 1e-9);
}

TEST_F(ImuDegradationTest, DegradedLimitWhenStale)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 200; ++i) {
    out = lim.step(1.0, 0.0, 0.0, kDt, 1.5, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.15, 1e-9);
}

TEST_F(ImuDegradationTest, RecoveryRestoresFullLimit)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 200; ++i) {
    out = lim.step(1.0, 0.0, 0.0, kDt, 1.5, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.15, 1e-9);
  for (int i = 0; i < 200; ++i) {
    out = lim.step(1.0, 0.0, 0.0, kDt, 0.1, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.30, 1e-9);
}

TEST_F(ImuDegradationTest, StaleBoundaryAndBadValues)
{
  VelocityLimiter lim;
  EXPECT_FALSE(lim.imu_stale(1.0));           // 恰好 1.0 s：不超时
  EXPECT_TRUE(lim.imu_stale(1.0 + 1e-9));
  EXPECT_TRUE(lim.imu_stale(kNaN));           // 从未收到 → 视为掉线（Python None）
}

// ---------------------------------------------------------------------------
// SW-U02 精神：dt 异常保护
// ---------------------------------------------------------------------------
class DtProtectionTest : public ::testing::Test
{
};

TEST_F(DtProtectionTest, InvalidDtKeepsStateUnchanged)
{
  VelocityLimiter lim;
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 10; ++i) {
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.3, 1e-9);
  const Vel prev = out;
  // 目标为 0：若 bad dt 被当作有效时间，输出会立刻减速；按 0 处理则保持
  // （Python 的 None / 'x' / nan 在 C++ 中统一为 NaN）
  for (double bad : {0.0, -0.1, 0.5001, 1.0, 100.0, kNaN}) {
    out = lim.step(0.0, 0.0, 0.0, bad, 0.0, 0.0);
    ExpectVel(out, prev.vx, prev.vy, prev.wz, 1e-12);
  }
  // dt=0.5 恰好合法：0.3 - 0.6*0.5 = 0
  out = lim.step(0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
  EXPECT_NEAR(out.vx, 0.0, 1e-12);
}

TEST_F(DtProtectionTest, NoExceptionOnBadDt)
{
  VelocityLimiter lim;
  for (double bad : {0.0, -1.0, 1.0, kNaN}) {
    Vel out{1.0, 1.0, 1.0};
    EXPECT_NO_THROW(out = lim.step(0.1, 0.0, 0.0, bad, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(out.vx, 0.0);  // dt 按 0 → 状态保持 0
  }
}

// ---------------------------------------------------------------------------
// P0 输入超时看门狗：/cmd_vel 静默 → 瞬时归零（J-07/SW-F01 三级停车）
// ---------------------------------------------------------------------------
class InputTimeoutTest : public ::testing::Test
{
};

TEST_F(InputTimeoutTest, SilenceAfter1sZeroesOutput)
{
  // 持续喂目标 1 s 后停止喂 → 超时一步瞬时归零并保持。
  VelocityLimiter lim(0.30, 1.5, 0.6, 2.0, 0.15, 1.0, 0.4);
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {  // 持续喂 1 s（input_age=0 表示刚收到指令）
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.3, 1e-9);
  double age = 0.0;
  for (int i = 0; i < 60; ++i) {  // 停止喂：input_age 每拍递增，模拟时间流逝
    age += kDt;
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, age);
  }
  ExpectVelZero(out);
}

TEST_F(InputTimeoutTest, TimeoutStartsOnlyAfter0_4s)
{
  VelocityLimiter lim(0.30, 1.5, 0.6, 2.0, 0.15, 1.0, 0.4);
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  // age 恰 0.4 s：未超时，继续跟随目标
  out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.4);
  EXPECT_NEAR(out.vx, 0.3, 1e-12);
  // age > 0.4 s：瞬时归零（安全路径，不经斜率限幅）
  const Vel out2 = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.4 + 1e-9);
  ExpectVelZero(out2);
}

TEST_F(InputTimeoutTest, ImmediateZeroAtThresholdCrossing)
{
  // 0.2 m/s 目标：input_age 刚过阈值的一步，输出直接为 0（无中间值）。
  VelocityLimiter lim(0.30, 1.5, 0.6, 2.0, 0.15, 1.0, 0.4);
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {
    out = lim.step(0.2, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  EXPECT_NEAR(out.vx, 0.2, 1e-9);
  ExpectVelZero(lim.step(0.2, 0.0, 0.0, kDt, 0.0, 0.4 + 1e-9));  // 无 0.2→0.17→… 斜坡
  // 超时期间持续恒零（即使目标保持非零）
  for (double age : {0.5, 1.0, 2.0}) {
    ExpectVelZero(lim.step(0.2, 0.0, 0.0, kDt, 0.0, age));
  }
}

TEST_F(InputTimeoutTest, NeverReceivedInputIsStale)
{
  // 从未收到 /cmd_vel：视为超时，启动即输出零速（Python None → C++ NaN）。
  VelocityLimiter lim;
  EXPECT_TRUE(lim.input_stale(kNaN));
  ExpectVelZero(lim.step(0.3, 0.0, 0.0, kDt, 0.0, kNaN));
  EXPECT_FALSE(lim.input_stale(0.4));          // 边界：恰好 0.4 不超时
  EXPECT_TRUE(lim.input_stale(0.4 + 1e-9));
}

TEST_F(InputTimeoutTest, RecoveryAfterTimeout)
{
  // 超时瞬时归零后恢复指令：从零平滑加速（斜率限幅仍生效）。
  VelocityLimiter lim(0.30, 1.5, 0.6, 2.0, 0.15, 1.0, 0.4);
  Vel out{0.0, 0.0, 0.0};
  for (int i = 0; i < 20; ++i) {
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
  }
  double age = 0.45;
  for (int i = 0; i < 40; ++i) {  // 超时瞬时归零并保持
    age += kDt;
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, age);
  }
  ExpectVelZero(out);
  double prev = 0.0;
  for (int i = 0; i < 20; ++i) {  // 恢复喂指令：平滑加速回 0.3（每步 ≤ kMaxDv）
    out = lim.step(0.3, 0.0, 0.0, kDt, 0.0, 0.0);
    EXPECT_LE(out.vx - prev, kMaxDv + 1e-12);
    prev = out.vx;
  }
  EXPECT_NEAR(out.vx, 0.3, 1e-9);
}

TEST_F(InputTimeoutTest, EstopOverridesInputTimeout)
{
  // estop 锁存优先级高于输入超时。
  VelocityLimiter lim;
  lim.estop(true);
  for (double age : {0.0, 0.4, 1.0, kNaN}) {
    ExpectVelZero(lim.step(0.3, 0.0, 0.0, kDt, 0.0, age));
  }
}

// ---------------------------------------------------------------------------
// 非法参数拒绝（SW-U15 精神）—— 对应 Python 13 组 parametrize
// slot 编码：0=max_linear 1=max_angular 2=max_lin_accel 3=max_ang_accel
//           4=degraded_max_linear 5=imu_timeout 6=input_timeout
// ---------------------------------------------------------------------------
namespace
{

VelocityLimiter make_with_slot(int slot, double value)
{
  switch (slot) {
    case 0:
      return VelocityLimiter(value, 1.5, 0.6, 2.0, 0.15, 1.0, 0.4);
    case 1:
      return VelocityLimiter(0.30, value, 0.6, 2.0, 0.15, 1.0, 0.4);
    case 2:
      return VelocityLimiter(0.30, 1.5, value, 2.0, 0.15, 1.0, 0.4);
    case 3:
      return VelocityLimiter(0.30, 1.5, 0.6, value, 0.15, 1.0, 0.4);
    case 4:
      return VelocityLimiter(0.30, 1.5, 0.6, 2.0, value, 1.0, 0.4);
    case 5:
      return VelocityLimiter(0.30, 1.5, 0.6, 2.0, 0.15, value, 0.4);
    case 6:
      return VelocityLimiter(0.30, 1.5, 0.6, 2.0, 0.15, 1.0, value);
    default:
      throw std::logic_error("bad slot");
  }
}

const char * slot_name(int slot)
{
  switch (slot) {
    case 0: return "max_linear";
    case 1: return "max_angular";
    case 2: return "max_lin_accel";
    case 3: return "max_ang_accel";
    case 4: return "degraded_max_linear";
    case 5: return "imu_timeout";
    case 6: return "input_timeout";
    default: return "unknown";
  }
}

}  // namespace

class ParamValidationTest : public ::testing::TestWithParam<std::tuple<double, int>>
{
};

TEST_P(ParamValidationTest, InvalidParamsRaiseValueError)
{
  EXPECT_THROW(
    make_with_slot(std::get<1>(GetParam()), std::get<0>(GetParam())),
    std::invalid_argument);
}

struct ParamName
{
  template<typename ParamType>
  std::string operator()(const ::testing::TestParamInfo<ParamType> & info) const
  {
    const double value = std::get<0>(info.param);
    const int slot = std::get<1>(info.param);
    std::string suffix;
    if (std::isnan(value)) {
      suffix = "nan";          // Python None/'x' 的 C++ 等价
    } else if (std::isinf(value)) {
      suffix = "inf";
    } else {
      std::string s = std::to_string(value);
      // 去掉多余的零（-1.000000 → -1）
      if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') {
          s.pop_back();
        }
        if (!s.empty() && s.back() == '.') {
          s.pop_back();
        }
      }
      if (s == "-0") {
        s = "0";
      }
      // gtest 参数名只允许字母数字下划线：-1.5 → neg1_5
      if (!s.empty() && s[0] == '-') {
        s = "neg" + s.substr(1);
      }
      for (auto& c : s) {
        if (c == '.') {
          c = '_';
        }
      }
      suffix = s;
    }
    return slot_name(slot) + std::string("_") + suffix;
  }
};

INSTANTIATE_TEST_SUITE_P(
  InvalidParams,
  ParamValidationTest,
  ::testing::Values(
    // Python parametrize 13 组 1:1 映射：
    // ('x' → NaN)、(None → NaN)
    std::make_tuple(0.0, 0),      // max_linear=0.0
    std::make_tuple(-1.0, 0),     // max_linear=-1.0
    std::make_tuple(kNaN, 0),     // max_linear='x'
    std::make_tuple(0.0, 1),      // max_angular=0.0
    std::make_tuple(0.0, 2),      // max_lin_accel=0.0
    std::make_tuple(-2.0, 3),     // max_ang_accel=-2.0
    std::make_tuple(0.0, 4),      // degraded_max_linear=0.0
    std::make_tuple(0.0, 5),      // imu_timeout=0.0
    std::make_tuple(-1.0, 5),     // imu_timeout=-1.0
    std::make_tuple(0.0, 6),      // input_timeout=0.0
    std::make_tuple(-0.4, 6),     // input_timeout=-0.4
    std::make_tuple(kNaN, 6),     // input_timeout=None
    std::make_tuple(kInf, 0)),    // max_linear=inf
  ParamName());
