#include "pd_ff_controller/pd_ff_controller.hpp"

#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pd_ff_controller
{

controller_interface::CallbackReturn PdFfController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PdFfController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : params_.joint_names) {
    cfg.names.push_back(name + "/" + params_.command_interface);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
PdFfController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // interface-major order: all joints for iface[0], then all joints for iface[1], ...
  // must match the access pattern in update_and_write_commands
  for (const auto & iface : params_.state_interfaces) {
    for (const auto & name : params_.joint_names) {
      cfg.names.push_back(name + "/" + iface);
    }
  }
  return cfg;
}

controller_interface::CallbackReturn PdFfController::on_configure(
  const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  n_joints_ = params_.joint_names.size();

  // Resolve which reference blocks (if any) carry "effort" / "gait_state" —
  // these have no paired state block, so unlike position/velocity (always
  // blocks 0/1) they're located by name, not fixed position.
  effort_block_idx_ = -1;
  gait_block_idx_ = -1;
  for (size_t k = 0; k < params_.reference_interfaces.size(); ++k) {
    if (params_.reference_interfaces[k] == "effort") effort_block_idx_ = static_cast<int>(k);
    if (params_.reference_interfaces[k] == "gait_state") gait_block_idx_ = static_cast<int>(k);
  }

  // Resolve leg groupings: every controlled joint must belong to exactly one
  // leg, and every leg's leg_number must be a unique, valid index into the
  // gait_state array of the incoming reference message.
  n_legs_ = params_.leg_names.size();
  joint_to_leg_number_.assign(n_joints_, -1);
  std::vector<bool> leg_number_used(n_legs_, false);
  for (const auto & leg_name : params_.leg_names) {
    const auto & leg = params_.legs.leg_names_map.at(leg_name);
    if (leg.leg_number < 0 || static_cast<size_t>(leg.leg_number) >= n_legs_) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "on_configure: leg '%s' has leg_number %ld out of range [0, %zu)",
        leg_name.c_str(), leg.leg_number, n_legs_);
      return CallbackReturn::ERROR;
    }
    if (leg_number_used[static_cast<size_t>(leg.leg_number)]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "on_configure: leg_number %ld is claimed by more than one leg", leg.leg_number);
      return CallbackReturn::ERROR;
    }
    leg_number_used[static_cast<size_t>(leg.leg_number)] = true;

    for (const auto & joint_name : leg.joint_names) {
      const auto it = std::find(params_.joint_names.begin(), params_.joint_names.end(), joint_name);
      if (it == params_.joint_names.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "on_configure: leg '%s' references unknown joint '%s'",
          leg_name.c_str(), joint_name.c_str());
        return CallbackReturn::ERROR;
      }
      const auto idx = static_cast<size_t>(std::distance(params_.joint_names.begin(), it));
      if (joint_to_leg_number_[idx] != -1) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "on_configure: joint '%s' is claimed by more than one leg", joint_name.c_str());
        return CallbackReturn::ERROR;
      }
      joint_to_leg_number_[idx] = leg.leg_number;
    }
  }
  for (size_t i = 0; i < n_joints_; ++i) {
    if (joint_to_leg_number_[i] == -1) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "on_configure: joint '%s' is not covered by any leg", params_.joint_names[i].c_str());
      return CallbackReturn::ERROR;
    }
  }
  auto sub_qos = rclcpp::SystemDefaultsQoS();
  sub_qos.keep_last(1).best_effort();

  ref_sub_ = get_node()->create_subscription<ReferenceMsg>(
    "~/reference", sub_qos,
    [this](const std::shared_ptr<ReferenceMsg> msg) { input_ref_.set(*msg); });

  ReferenceMsg nan_ref;
  nan_ref.name.assign(n_joints_, std::string());
  nan_ref.position.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  nan_ref.velocity.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  nan_ref.torque.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  nan_ref.gait_state.assign(n_legs_, ReferenceMsg::STANCE);
  input_ref_.set(nan_ref);

  // pre-allocate message storage so the RT loop never allocates
  tau_p_msg_.data.assign(n_joints_, 0.0);
  tau_d_msg_.data.assign(n_joints_, 0.0);

  auto pub_qos = rclcpp::SystemDefaultsQoS();
  tau_p_pub_ = get_node()->create_publisher<PdActionsMsg>("~/tau_p", pub_qos);
  tau_d_pub_ = get_node()->create_publisher<PdActionsMsg>("~/tau_d", pub_qos);
  rt_tau_p_pub_ = std::make_unique<PdActionsPublisher>(tau_p_pub_);
  rt_tau_d_pub_ = std::make_unique<PdActionsPublisher>(tau_d_pub_);

  // Seed algorithm with initial (stance) gains — no gait signal exists yet.
  std::vector<PdFf::JointGains> gains(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    const auto & g = params_.gains.joint_names_map.at(params_.joint_names[i]).stance;
    gains[i] = {g.p, g.d};
  }
  algo_.setGains(gains);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PdFfController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const size_t n_ref   = params_.reference_interfaces.size();
  const size_t n_state = params_.state_interfaces.size();

  reference_interfaces_.assign(n_ref * n_joints_, 0.0);

  // Seed each reference block that has a paired state block with the current HW value.
  // Blocks beyond n_state (i.e. the FF/gait_state blocks) stay at 0. Guard against
  // state_interfaces_ not being assigned yet (e.g. hardware not connected) — indexing
  // an empty/undersized vector here would otherwise be undefined behavior.
  for (size_t k = 0; k < n_state && k < n_ref; ++k) {
    for (size_t i = 0; i < n_joints_; ++i) {
      const size_t idx = k * n_joints_ + i;
      if (idx >= state_interfaces_.size()) continue;
      const auto val = state_interfaces_[idx].get_optional();
      if (val.has_value()) {
        reference_interfaces_[idx] = val.value();
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
PdFfController::on_export_reference_interfaces()
{
  const size_t n_ref = params_.reference_interfaces.size();
  reference_interfaces_.resize(n_ref * n_joints_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.reserve(n_ref * n_joints_);

  const std::string prefix = std::string(get_node()->get_name()) + "/";

  for (size_t k = 0; k < n_ref; ++k) {
    for (size_t i = 0; i < n_joints_; ++i) {
      ifaces.emplace_back(
        prefix + params_.joint_names[i], params_.reference_interfaces[k],
        &reference_interfaces_[k * n_joints_ + i]);
    }
  }

  return ifaces;
}

bool PdFfController::on_set_chained_mode(bool) { return true; }

controller_interface::return_type PdFfController::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Only called when not in chained mode; in chained mode the upstream controller
  // writes directly into reference_interfaces_ via the exported CommandInterfaces.
  auto ref_op = input_ref_.try_get();
  if (!ref_op.has_value()) return controller_interface::return_type::OK;

  const auto & ref = ref_op.value();
  if (ref.name.size() != n_joints_ || ref.position.size() != n_joints_ ||
      ref.velocity.size() != n_joints_ || ref.torque.size() != n_joints_ ||
      ref.gait_state.size() != n_legs_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "JointCmd reference size mismatch (name=%zu pos=%zu vel=%zu torque=%zu gait_state=%zu; "
      "expected name/pos/vel/torque=%zu, gait_state=%zu); skipping",
      ref.name.size(), ref.position.size(), ref.velocity.size(), ref.torque.size(),
      ref.gait_state.size(), n_joints_, n_legs_);
    return controller_interface::return_type::ERROR;
  }
  const size_t n_ref = params_.reference_interfaces.size();
  for (size_t i = 0; i < n_joints_; ++i) {
    if (n_ref >= 1 && i < ref.position.size() && std::isfinite(ref.position[i]))
      reference_interfaces_[0 * n_joints_ + i] = ref.position[i];
    if (n_ref >= 2 && i < ref.velocity.size() && std::isfinite(ref.velocity[i]))
      reference_interfaces_[1 * n_joints_ + i] = ref.velocity[i];
    if (effort_block_idx_ >= 0 && i < ref.torque.size() && std::isfinite(ref.torque[i]))
      reference_interfaces_[static_cast<size_t>(effort_block_idx_) * n_joints_ + i] = ref.torque[i];
    // gait_state has no NaN sentinel (uint8), so unlike position/velocity/torque it is
    // applied unconditionally from every received message, not skipped on invalid data.
    if (gait_block_idx_ >= 0) {
      const int64_t leg = joint_to_leg_number_[i];
      if (leg >= 0 && static_cast<size_t>(leg) < ref.gait_state.size()) {
        reference_interfaces_[static_cast<size_t>(gait_block_idx_) * n_joints_ + i] =
          static_cast<double>(ref.gait_state[static_cast<size_t>(leg)]);
      }
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type PdFfController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Refresh gains each cycle — they are declared dynamic in the YAML so they
  // can be updated at runtime via ros2 param set without restarting the controller.
  params_ = param_listener_->get_params();
  std::vector<PdFf::JointGains> gains(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    bool is_swing = false;
    if (gait_block_idx_ >= 0) {
      const double v = reference_interfaces_[static_cast<size_t>(gait_block_idx_) * n_joints_ + i];
      is_swing = std::isfinite(v) && v >= 0.5;
    }
    const auto & g = params_.gains.joint_names_map.at(params_.joint_names[i]);
    if (is_swing) {
      const auto & phase = g.swing;
      gains[i] = {phase.p, phase.d};
    } else {
      const auto & phase = g.stance;
      gains[i] = {phase.p, phase.d};
    }
  }
  algo_.setGains(gains);

  // Build error and feedforward vectors from hardware/reference interfaces.
  const size_t n_ref   = params_.reference_interfaces.size();
  const size_t n_state = params_.state_interfaces.size();

  Eigen::VectorXd pos_error = Eigen::VectorXd::Zero(n_joints_);
  Eigen::VectorXd vel_error = Eigen::VectorXd::Zero(n_joints_);
  Eigen::VectorXd ff        = Eigen::VectorXd::Zero(n_joints_);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double ref0   = reference_interfaces_[0 * n_joints_ + i];
    const auto   state0 = state_interfaces_[0 * n_joints_ + i].get_optional();
    // On skip, pos_error/vel_error stay 0.0, producing zero P/D contribution
    // for this joint this cycle, rather than skipping the write entirely.
    if (!state0.has_value() || !std::isfinite(ref0)) continue;
    pos_error[i] = ref0 - state0.value();

    if (n_ref >= 2 && n_state >= 2) {
      const double ref1   = reference_interfaces_[1 * n_joints_ + i];
      const auto   state1 = state_interfaces_[1 * n_joints_ + i].get_optional();
      if (state1.has_value() && std::isfinite(ref1))
        vel_error[i] = ref1 - state1.value();
    }

    if (effort_block_idx_ >= 0) {
      const double ff_val =
        reference_interfaces_[static_cast<size_t>(effort_block_idx_) * n_joints_ + i];
      if (std::isfinite(ff_val)) ff[i] = ff_val;
    }
  }

  // Delegate to algorithm.
  const auto out = algo_.compute(pos_error, vel_error, ff);

  // Write commands and fill diagnostic messages.
  for (size_t i = 0; i < n_joints_; ++i) {
    tau_p_msg_.data[i] = out.tau_p[i];
    tau_d_msg_.data[i] = out.tau_d[i];
    if (!command_interfaces_[i].set_value(out.tau[i]))
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot set command on dof [%zu]!", i);
  }

  rt_tau_p_pub_->try_publish(tau_p_msg_);
  rt_tau_d_pub_->try_publish(tau_d_msg_);
  return controller_interface::return_type::OK;
}

}  // namespace pd_ff_controller

PLUGINLIB_EXPORT_CLASS(
  pd_ff_controller::PdFfController, controller_interface::ChainableControllerInterface)
