#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "control_toolbox/pid.hpp"

namespace pd_ff_controller
{

class PdFf
{
public:
  struct JointGains {
    double p{0.0};      // proportional gain [N·m/rad]
    double d{0.0};      // derivative gain   [N·m·s/rad]
    double i{0.0};      // integral gain     [N·m/(rad·s)]
    double i_max{0.0};  // upper clamp for the integral contribution
    double i_min{0.0};  // lower clamp for the integral contribution
  };

  struct Output {
    Eigen::VectorXd tau;    // total torque command per joint
    Eigen::VectorXd tau_p;  // P-term contribution per joint (diagnostic)
    Eigen::VectorXd tau_i;  // I-term contribution per joint (diagnostic)
    Eigen::VectorXd tau_d;  // D-term contribution per joint (diagnostic)
  };

  // Set per-joint gains. Vector size must equal the number of controlled joints.
  // Safe to call every control cycle: existing control_toolbox::Pid instances are
  // updated in place (their integral accumulators are preserved), and are only
  // (re)constructed when the joint count changes.
  void setGains(const std::vector<JointGains> & gains);

  // Reset all PID internal state (integral accumulator, last error, etc.).
  void reset();

  // Reset a single joint's PID internal state (e.g. on a stance/swing phase
  // transition, to avoid the old phase's accumulated error being multiplied
  // by the new phase's gains). i must be a valid joint index.
  void resetJoint(std::size_t i);

  // Compute PID + FF torques. All input vectors must be size n_joints.
  // dt_ns is the control period in nanoseconds (control_toolbox::Pid's own
  // duration type); it must be nonzero or control_toolbox::Pid will return a
  // zero command for that cycle.
  Output compute(
    const Eigen::VectorXd & pos_error,
    const Eigen::VectorXd & vel_error,
    const Eigen::VectorXd & ff,
    int64_t dt_ns);

private:
  std::vector<control_toolbox::Pid> pids_;
};

}  // namespace pd_ff_controller
