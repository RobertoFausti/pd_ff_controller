// Unit tests for PdFf (the PID+feedforward algorithm) in isolation, with no
// controller/ros2_control/rclcpp machinery involved — only Eigen vectors and
// plain method calls on the class under test.

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "pd_ff_controller/pd_ff.hpp"

namespace
{

using pd_ff_controller::PdFf;

constexpr int64_t kDtNs = 1'000'000;  // 1 ms

Eigen::VectorXd Vec1(double v)
{
  Eigen::VectorXd out(1);
  out << v;
  return out;
}

Eigen::VectorXd Vec2(double a, double b)
{
  Eigen::VectorXd out(2);
  out << a, b;
  return out;
}

PdFf::JointGains Gains(
  double p, double d = 0.0, double i = 0.0, double i_max = 0.0, double i_min = 0.0)
{
  return {p, d, i, i_max, i_min};
}

// ── proportional / derivative / feedforward ──────────────────────────────────

TEST(PdFfTest, PositionErrorOnly_ProducesProportionalTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/10.0)});

  const auto out = algo.compute(Vec1(2.0), Vec1(0.0), Vec1(0.0), kDtNs);

  EXPECT_NEAR(out.tau_p[0], 20.0, 1e-9);
  EXPECT_NEAR(out.tau_d[0], 0.0, 1e-9);
  EXPECT_NEAR(out.tau_i[0], 0.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 20.0, 1e-9);
}

TEST(PdFfTest, VelocityErrorOnly_ProducesDerivativeTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/4.0)});

  const auto out = algo.compute(Vec1(0.0), Vec1(3.0), Vec1(0.0), kDtNs);

  EXPECT_NEAR(out.tau_d[0], 12.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 12.0, 1e-9);
}

TEST(PdFfTest, CombinedPD_SumsCorrectly)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0, /*d=*/2.0)});

  const auto out = algo.compute(Vec1(1.0), Vec1(0.5), Vec1(0.0), kDtNs);

  // tau = 5*1.0 + 2*0.5 = 6.0
  EXPECT_NEAR(out.tau[0], 6.0, 1e-9);
}

TEST(PdFfTest, NegativeError_ProducesNegativeTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0)});

  const auto out = algo.compute(Vec1(-2.0), Vec1(0.0), Vec1(0.0), kDtNs);

  EXPECT_NEAR(out.tau[0], -10.0, 1e-9);
}

TEST(PdFfTest, Feedforward_AddsOnTopOfPD)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0)});

  const auto out = algo.compute(Vec1(1.0), Vec1(0.0), Vec1(2.0), kDtNs);

  // tau = p_term (5.0) + ff (2.0); ff is never clamped or attributed to tau_p/tau_i/tau_d.
  EXPECT_NEAR(out.tau_p[0], 5.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 7.0, 1e-9);
}

TEST(PdFfTest, FeedforwardOnly_ZeroGains_PassesThroughUnchanged)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0)});

  const auto out = algo.compute(Vec1(0.0), Vec1(0.0), Vec1(7.5), kDtNs);

  EXPECT_NEAR(out.tau[0], 7.5, 1e-9);
}

// ── integral action ──────────────────────────────────────────────────────────

TEST(PdFfTest, IntegralAccumulates_OverMultipleCycles)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)});

  const auto pos_error = Vec1(1.0);
  const auto zero = Vec1(0.0);

  // control_toolbox's BACK_CALCULATION integral update returns the
  // *pre-update* accumulator each call (one-cycle lag versus a naive
  // "accumulate then report" model): first call reports the still-zero
  // initial state, and each subsequent call reports the PREVIOUS cycle's
  // accumulation of i_gain * (dt_ns/1e9) * pos_error. Verified directly
  // against this Jazzy build of control_toolbox.
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.0, 1e-9);
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.002, 1e-9);
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.004, 1e-9);
}

// All joints unconditionally use control_toolbox's BACK_CALCULATION anti-windup
// strategy (see BackCalculationStrategy() in pd_ff.cpp). Its tracking_time_constant
// is derived internally by PdFf rather than left at 0.0 (control_toolbox's own
// "auto-derive" value): control_toolbox's recommended auto-derivation
// (sqrt(d/i) if d!=0, else p/i) divides by i unconditionally, which is NaN
// when both p and d are 0 — a pure-integral gain set, exercised below and
// used by several other tests in this file. Confirmed via a standalone probe
// directly against this Jazzy build of control_toolbox (bypassing PdFf) that
// tracking_time_constant=0.0 does produce NaN output for such a gain set, and
// that PdFf's internal fallback constant avoids it.

TEST(PdFfTest, BackCalculationPureIntegral_SaturatesWithoutNaN)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/100.0, /*i_max=*/0.01, /*i_min=*/-0.01)});

  const auto zero = Vec1(0.0);

  for (int k = 0; k < 5; ++k) {
    const double tau = algo.compute(Vec1(1.0), zero, zero, kDtNs).tau[0];
    ASSERT_FALSE(std::isnan(tau));
    EXPECT_LE(tau, 0.01 + 1e-9);
  }
}

TEST(PdFfTest, BackCalculationPD_TracksSqrtDOverIFormula)
{
  // p=0 (irrelevant here), d and i both nonzero: PdFf should use
  // control_toolbox's own recommended tracking_time_constant, sqrt(d/i), not
  // the pure-integral fallback — exercised indirectly by checking the output
  // stays finite and saturates at i_max, same safety bar as the pure-I case.
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/3.0, /*i=*/100.0, /*i_max=*/0.01, /*i_min=*/-0.01)});

  const auto zero = Vec1(0.0);
  for (int k = 0; k < 5; ++k) {
    const double tau = algo.compute(Vec1(1.0), zero, zero, kDtNs).tau[0];
    ASSERT_FALSE(std::isnan(tau));
    EXPECT_LE(tau, 0.01 + 1e-9);
  }
}

// ── invalid period ────────────────────────────────────────────────────────────

TEST(PdFfTest, DtZero_TotalCommandIsFeedforwardOnly)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/10.0, /*d=*/5.0, /*i=*/2.0, /*i_max=*/100.0, /*i_min=*/-100.0)});

  // control_toolbox::Pid::compute_command returns exactly 0.0 for the PID
  // contribution when dt_ns == 0, regardless of gains/errors — a safety
  // fallback. Feedforward still passes through untouched.
  const auto out = algo.compute(Vec1(1.0), Vec1(1.0), Vec1(3.0), /*dt_ns=*/0);

  EXPECT_NEAR(out.tau[0], 3.0, 1e-9);
}

// ── multi-joint independence ──────────────────────────────────────────────────

TEST(PdFfTest, MultipleJoints_IndependentGainsAndErrors)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/10.0, /*d=*/0.0), Gains(/*p=*/0.0, /*d=*/4.0)});

  const auto out = algo.compute(Vec2(2.0, 0.0), Vec2(0.0, 3.0), Vec2(0.0, 0.0), kDtNs);

  EXPECT_NEAR(out.tau[0], 20.0, 1e-9);
  EXPECT_NEAR(out.tau[1], 12.0, 1e-9);
}

TEST(PdFfTest, OutputVectorSizes_MatchJointCount)
{
  PdFf algo;
  algo.setGains({Gains(1.0), Gains(1.0), Gains(1.0)});

  const auto out = algo.compute(
    Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), kDtNs);

  EXPECT_EQ(out.tau.size(), 3);
  EXPECT_EQ(out.tau_p.size(), 3);
  EXPECT_EQ(out.tau_i.size(), 3);
  EXPECT_EQ(out.tau_d.size(), 3);
}

// ── reset() / resetJoint() ─────────────────────────────────────────────────────

TEST(PdFfTest, Reset_ClearsIntegralAccumulator)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)});

  const auto pos_error = Vec1(1.0);
  const auto zero = Vec1(0.0);

  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.0, 1e-9);
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.002, 1e-9);

  algo.reset();

  // Back to the first cycle's value — the accumulator restarted from zero.
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.0, 1e-9);
}

TEST(PdFfTest, ResetJoint_ClearsOnlyThatJointsAccumulator)
{
  PdFf algo;
  algo.setGains(
    {Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0),
     Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)});

  const auto pos_error = Vec2(1.0, 1.0);
  const auto zero = Vec2(0.0, 0.0);

  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.0, 1e-9);
  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[1], 0.002, 1e-9);

  algo.resetJoint(0);  // only joint 0's accumulator is cleared

  const auto out = algo.compute(pos_error, zero, zero, kDtNs);
  EXPECT_NEAR(out.tau[0], 0.0, 1e-9);    // restarted
  EXPECT_NEAR(out.tau[1], 0.004, 1e-9);  // kept accumulating undisturbed
}

// ── setGains() semantics ──────────────────────────────────────────────────────

TEST(PdFfTest, SetGains_RepeatedCalls_PreserveIntegralAccumulator)
{
  PdFf algo;
  const std::vector<PdFf::JointGains> gains{
    Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)};
  algo.setGains(gains);

  const auto pos_error = Vec1(1.0);
  const auto zero = Vec1(0.0);

  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.0, 1e-9);

  // Re-applying the same gains (as the controller does every cycle to pick up
  // live ros2 param updates) must NOT reset the accumulator.
  algo.setGains(gains);

  EXPECT_NEAR(algo.compute(pos_error, zero, zero, kDtNs).tau[0], 0.002, 1e-9);
}

TEST(PdFfTest, SetGains_JointCountChange_ReconstructsWithFreshState)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)});

  const auto zero1 = Vec1(0.0);
  algo.compute(Vec1(1.0), zero1, zero1, kDtNs);
  algo.compute(Vec1(1.0), zero1, zero1, kDtNs);  // accumulator now at 0.004

  // Growing to 2 joints reconstructs the internal Pid instances from scratch.
  algo.setGains(
    {Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0),
     Gains(/*p=*/0.0, /*d=*/0.0, /*i=*/2.0, /*i_max=*/1000.0, /*i_min=*/-1000.0)});

  const auto zero2 = Vec2(0.0, 0.0);
  const auto out = algo.compute(Vec2(1.0, 1.0), zero2, zero2, kDtNs);

  // First-cycle value (0.0, see IntegralAccumulates_OverMultipleCycles) for
  // both joints, not a continuation of the discarded state (which would show
  // up as a nonzero value on joint 0, e.g. 0.004 continuing its prior climb).
  EXPECT_NEAR(out.tau[0], 0.0, 1e-9);
  EXPECT_NEAR(out.tau[1], 0.0, 1e-9);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
