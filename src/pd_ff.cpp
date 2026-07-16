#include "pd_ff_controller/pd_ff.hpp"

#include <cmath>
#include <limits>

namespace pd_ff_controller
{

namespace
{

// control_toolbox recommends tracking_time_constant = 0.0, which it then
// auto-derives as sqrt(d/i) if d != 0, else p/i. That auto-derivation divides
// by i unconditionally, so it produces NaN (0/0) whenever both p and d are 0
// — a pure-integral gain set, which this controller supports and tests. To
// avoid ever depending on that internal auto-derivation (and its NaN case),
// PdFf computes its own tracking_time_constant explicitly here, using the
// same formula control_toolbox recommends when it's well-defined (p or d
// nonzero), and a small fixed fallback otherwise (pure-I, or i == 0 in which
// case the value is unused since the integral term is always zero).
constexpr double kFallbackTrackingTimeConstant = 0.001;  // 1 ms

double TrackingTimeConstant(const PdFf::JointGains & g)
{
  if (g.i != 0.0 && g.d != 0.0) return std::sqrt(std::abs(g.d / g.i));
  if (g.i != 0.0 && g.p != 0.0) return std::abs(g.p / g.i);
  return kFallbackTrackingTimeConstant;
}

// All joints use the back_calculation anti-windup strategy. Output (u_max/u_min)
// is intentionally left unclamped — PdFf has never clamped the total command,
// only the integral contribution (i_max/i_min), and that behavior is preserved
// here.
control_toolbox::AntiWindupStrategy BackCalculationStrategy(const PdFf::JointGains & g)
{
  control_toolbox::AntiWindupStrategy strat;
  strat.type = control_toolbox::AntiWindupStrategy::BACK_CALCULATION;
  strat.i_max = g.i_max;
  strat.i_min = g.i_min;
  strat.tracking_time_constant = TrackingTimeConstant(g);
  return strat;
}

constexpr double kUMax = std::numeric_limits<double>::infinity();
constexpr double kUMin = -std::numeric_limits<double>::infinity();

}  // namespace

void PdFf::setGains(const std::vector<JointGains> & gains)
{
  // Reconstruct only on a joint-count change; otherwise update gains on the
  // existing Pid instances so their integral accumulators survive being
  // called every control cycle (Pid::set_gains does not reset() internally).
  if (pids_.size() != gains.size()) {
    pids_.assign(
      gains.size(), control_toolbox::Pid(0.0, 0.0, 0.0, kUMax, kUMin,
        control_toolbox::AntiWindupStrategy{}));
  }
  for (std::size_t i = 0; i < gains.size(); ++i) {
    pids_[i].set_gains(
      gains[i].p, gains[i].i, gains[i].d, kUMax, kUMin, BackCalculationStrategy(gains[i]));
  }
}

void PdFf::reset()
{
  for (auto & pid : pids_) {
    pid.reset();
  }
}

void PdFf::resetJoint(std::size_t i)
{
  pids_[i].reset();
}

PdFf::Output PdFf::compute(
  const Eigen::VectorXd & pos_error,
  const Eigen::VectorXd & vel_error,
  const Eigen::VectorXd & ff,
  int64_t dt_ns)
{
  const auto n = static_cast<Eigen::Index>(pids_.size());
  Output out;
  out.tau   = Eigen::VectorXd::Zero(n);
  out.tau_p = Eigen::VectorXd::Zero(n);
  out.tau_i = Eigen::VectorXd::Zero(n);
  out.tau_d = Eigen::VectorXd::Zero(n);

  for (Eigen::Index i = 0; i < n; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const double total = pids_[idx].compute_command(pos_error[i], vel_error[i], dt_ns);
    const auto gains = pids_[idx].get_gains();

    // P and D are never clamped by control_toolbox::Pid, so they can be
    // reconstructed exactly from the gains and inputs; whatever remains of
    // `total` is exactly the I-term, regardless of the back_calculation
    // correction control_toolbox applied internally.
    out.tau_p[i] = gains.p_gain_ * pos_error[i];
    out.tau_d[i] = gains.d_gain_ * vel_error[i];
    out.tau_i[i] = total - out.tau_p[i] - out.tau_d[i];
    out.tau[i]   = total + ff[i];
  }
  return out;
}

}  // namespace pd_ff_controller
