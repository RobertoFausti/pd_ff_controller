#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "pd_ff_controller/pd_ff_controller_parameters.hpp"

namespace pd_ff_controller
{

class PdFfController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)  override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  size_t n_joints_{0};

  // reference_interfaces_ (inherited backing storage) layout — interface-major:
  //   block k = [k*N .. (k+1)*N - 1]  →  params_.reference_interfaces[k], all joints
  //   default: block 0 = position, block 1 = velocity, block 2 = effort (FF)
  //
  // state_interfaces_ (from HW) layout — same interface-major scheme:
  //   block k = [k*N .. (k+1)*N - 1]  →  params_.state_interfaces[k], all joints
  //   default: block 0 = position, block 1 = velocity
  //
  // Feedforward is active when n_ref > n_state; it occupies the last reference block.

  using ReferenceMsg = trajectory_msgs::msg::JointTrajectoryPoint;
  rclcpp::Subscription<ReferenceMsg>::SharedPtr ref_sub_;
  realtime_tools::RealtimeThreadSafeBox<ReferenceMsg> input_ref_;

  // Per-joint P and D action publishers. Messages are pre-allocated in on_configure;
  // data[i] is overwritten each cycle and published via try_publish (RT-safe).
  using PdActionsMsg = std_msgs::msg::Float64MultiArray;
  using PdActionsPublisher = realtime_tools::RealtimePublisher<PdActionsMsg>;
  rclcpp::Publisher<PdActionsMsg>::SharedPtr tau_p_pub_;
  rclcpp::Publisher<PdActionsMsg>::SharedPtr tau_d_pub_;
  std::unique_ptr<PdActionsPublisher> rt_tau_p_pub_;
  std::unique_ptr<PdActionsPublisher> rt_tau_d_pub_;
  PdActionsMsg tau_p_msg_;
  PdActionsMsg tau_d_msg_;
};

}  // namespace pd_ff_controller
