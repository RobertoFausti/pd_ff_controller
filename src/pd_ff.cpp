#include "pd_ff_controller/pd_ff.hpp"

namespace pd_ff_controller
{

void PdFf::setGains(const std::vector<JointGains> & gains)
{
  gains_ = gains;
}

PdFf::Output PdFf::compute(
  const Eigen::VectorXd & pos_error,
  const Eigen::VectorXd & vel_error,
  const Eigen::VectorXd & ff)
{
  const auto n = static_cast<Eigen::Index>(gains_.size());
  Output out;
  out.tau   = Eigen::VectorXd::Zero(n);
  out.tau_p = Eigen::VectorXd::Zero(n);
  out.tau_d = Eigen::VectorXd::Zero(n);

  for (Eigen::Index i = 0; i < n; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    out.tau_p[i] = gains_[idx].p * pos_error[i];
    out.tau_d[i] = gains_[idx].d * vel_error[i];
    out.tau[i]   = out.tau_p[i] + out.tau_d[i] + ff[i];
  }
  return out;
}

}  // namespace pd_ff_controller
