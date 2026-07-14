#include "pd_ff_controller/pd_ff_controller.hpp"

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

  auto sub_qos = rclcpp::SystemDefaultsQoS();
  sub_qos.keep_last(1).best_effort();

  ref_sub_ = get_node()->create_subscription<ReferenceMsg>(
    "~/reference", sub_qos,
    [this](const std::shared_ptr<ReferenceMsg> msg) { input_ref_.set(*msg); });

  ReferenceMsg nan_ref;
  nan_ref.position.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  nan_ref.velocity.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  nan_ref.effort.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  input_ref_.set(nan_ref);

  // pre-allocate message storage so the RT loop never allocates
  tau_p_msg_.data.assign(n_joints_, 0.0);
  tau_d_msg_.data.assign(n_joints_, 0.0);

  auto pub_qos = rclcpp::SystemDefaultsQoS();
  tau_p_pub_ = get_node()->create_publisher<PdActionsMsg>("~/tau_p", pub_qos);
  tau_d_pub_ = get_node()->create_publisher<PdActionsMsg>("~/tau_d", pub_qos);
  rt_tau_p_pub_ = std::make_unique<PdActionsPublisher>(tau_p_pub_);
  rt_tau_d_pub_ = std::make_unique<PdActionsPublisher>(tau_d_pub_);

  // Seed algorithm with initial gains.
  std::vector<PdFf::JointGains> gains(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    const auto & g = params_.gains.joint_names_map.at(params_.joint_names[i]);
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
  // Blocks beyond n_state (i.e. the FF block) stay at 0.
  for (size_t k = 0; k < n_state && k < n_ref; ++k) {
    for (size_t i = 0; i < n_joints_; ++i) {
      const auto val = state_interfaces_[k * n_joints_ + i].get_optional();
      if (val.has_value()) {
        reference_interfaces_[k * n_joints_ + i] = val.value();
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
  if (ref.position.size() != n_joints_ || ref.velocity.size() != n_joints_ || ref.effort.size() != n_joints_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "JointState reference size mismatch (pos=%zu vel=%zu eff=%zu, expected %zu); skipping",
      ref.position.size(), ref.velocity.size(), ref.effort.size(), n_joints_);
    return controller_interface::return_type::ERROR;
  }
  const size_t n_ref = params_.reference_interfaces.size();
  for (size_t i = 0; i < n_joints_; ++i) {
    if (n_ref >= 1 && i < ref.position.size() && std::isfinite(ref.position[i]))
      reference_interfaces_[0 * n_joints_ + i] = ref.position[i];
    if (n_ref >= 2 && i < ref.velocity.size() && std::isfinite(ref.velocity[i]))
      reference_interfaces_[1 * n_joints_ + i] = ref.velocity[i];
    if (n_ref >= 3 && i < ref.effort.size() && std::isfinite(ref.effort[i]))
      reference_interfaces_[2 * n_joints_ + i] = ref.effort[i];
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
    const auto & g = params_.gains.joint_names_map.at(params_.joint_names[i]);
    gains[i] = {g.p, g.d};
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
    if (!state0.has_value() || !std::isfinite(ref0)) continue;
    pos_error[i] = ref0 - state0.value();

    if (n_ref >= 2 && n_state >= 2) {
      const double ref1   = reference_interfaces_[1 * n_joints_ + i];
      const auto   state1 = state_interfaces_[1 * n_joints_ + i].get_optional();
      if (state1.has_value() && std::isfinite(ref1))
        vel_error[i] = ref1 - state1.value();
    }

    if (n_ref > n_state) {
      const double ff_val = reference_interfaces_[(n_ref - 1) * n_joints_ + i];
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
