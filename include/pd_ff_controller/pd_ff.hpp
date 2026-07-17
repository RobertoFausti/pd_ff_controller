#pragma once

#include <vector>

#include <Eigen/Dense>

namespace pd_ff_controller
{

class PdFf
{
public:
  struct JointGains {
    double p{0.0};  // proportional gain [N·m/rad]
    double d{0.0};  // derivative gain   [N·m·s/rad]
  };

  struct Output {
    Eigen::VectorXd tau;    // total torque command per joint
    Eigen::VectorXd tau_p;  // P-term contribution per joint (diagnostic)
    Eigen::VectorXd tau_d;  // D-term contribution per joint (diagnostic)
  };

  // Store per-joint gains for use by compute(). Vector size must equal the
  // number of controlled joints. Safe to call every control cycle.
  void setGains(const std::vector<JointGains> & gains);

  // Compute PD + FF torques. All input vectors must be size n_joints.
  Output compute(
    const Eigen::VectorXd & pos_error,
    const Eigen::VectorXd & vel_error,
    const Eigen::VectorXd & ff);

private:
  std::vector<JointGains> gains_;
};

}  // namespace pd_ff_controller
