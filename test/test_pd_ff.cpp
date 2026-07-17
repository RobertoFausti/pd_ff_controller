// Unit tests for PdFf (the PD+feedforward algorithm) in isolation, with no
// controller/ros2_control/rclcpp machinery involved — only Eigen vectors and
// plain method calls on the class under test.

#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "pd_ff_controller/pd_ff.hpp"

namespace
{

using pd_ff_controller::PdFf;

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

PdFf::JointGains Gains(double p, double d = 0.0)
{
  return {p, d};
}

// ── proportional / derivative / feedforward ──────────────────────────────────

TEST(PdFfTest, PositionErrorOnly_ProducesProportionalTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/10.0)});

  const auto out = algo.compute(Vec1(2.0), Vec1(0.0), Vec1(0.0));

  EXPECT_NEAR(out.tau_p[0], 20.0, 1e-9);
  EXPECT_NEAR(out.tau_d[0], 0.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 20.0, 1e-9);
}

TEST(PdFfTest, VelocityErrorOnly_ProducesDerivativeTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0, /*d=*/4.0)});

  const auto out = algo.compute(Vec1(0.0), Vec1(3.0), Vec1(0.0));

  EXPECT_NEAR(out.tau_d[0], 12.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 12.0, 1e-9);
}

TEST(PdFfTest, CombinedPD_SumsCorrectly)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0, /*d=*/2.0)});

  const auto out = algo.compute(Vec1(1.0), Vec1(0.5), Vec1(0.0));

  // tau = 5*1.0 + 2*0.5 = 6.0
  EXPECT_NEAR(out.tau[0], 6.0, 1e-9);
}

TEST(PdFfTest, NegativeError_ProducesNegativeTorque)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0)});

  const auto out = algo.compute(Vec1(-2.0), Vec1(0.0), Vec1(0.0));

  EXPECT_NEAR(out.tau[0], -10.0, 1e-9);
}

TEST(PdFfTest, Feedforward_AddsOnTopOfPD)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/5.0)});

  const auto out = algo.compute(Vec1(1.0), Vec1(0.0), Vec1(2.0));

  // tau = p_term (5.0) + ff (2.0); ff is never clamped or attributed to tau_p/tau_d.
  EXPECT_NEAR(out.tau_p[0], 5.0, 1e-9);
  EXPECT_NEAR(out.tau[0], 7.0, 1e-9);
}

TEST(PdFfTest, FeedforwardOnly_ZeroGains_PassesThroughUnchanged)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/0.0)});

  const auto out = algo.compute(Vec1(0.0), Vec1(0.0), Vec1(7.5));

  EXPECT_NEAR(out.tau[0], 7.5, 1e-9);
}

// ── multi-joint independence ──────────────────────────────────────────────────

TEST(PdFfTest, MultipleJoints_IndependentGainsAndErrors)
{
  PdFf algo;
  algo.setGains({Gains(/*p=*/10.0, /*d=*/0.0), Gains(/*p=*/0.0, /*d=*/4.0)});

  const auto out = algo.compute(Vec2(2.0, 0.0), Vec2(0.0, 3.0), Vec2(0.0, 0.0));

  EXPECT_NEAR(out.tau[0], 20.0, 1e-9);
  EXPECT_NEAR(out.tau[1], 12.0, 1e-9);
}

TEST(PdFfTest, OutputVectorSizes_MatchJointCount)
{
  PdFf algo;
  algo.setGains({Gains(1.0), Gains(1.0), Gains(1.0)});

  const auto out = algo.compute(
    Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));

  EXPECT_EQ(out.tau.size(), 3);
  EXPECT_EQ(out.tau_p.size(), 3);
  EXPECT_EQ(out.tau_d.size(), 3);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
