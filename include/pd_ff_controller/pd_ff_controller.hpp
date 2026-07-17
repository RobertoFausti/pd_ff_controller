#pragma once

#include <memory>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plan4ari_msgs/msg/joint_cmd.hpp"

#include "pd_ff_controller/pd_ff.hpp"
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

  PdFf algo_;

  // reference_interfaces_ (inherited backing storage) layout — interface-major:
  //   block k = [k*N .. (k+1)*N - 1]  →  params_.reference_interfaces[k], all joints
  //   default: block 0 = position, block 1 = velocity, block 2 = effort, block 3 = gait_state
  //
  // state_interfaces_ (from HW) layout — same interface-major scheme:
  //   block k = [k*N .. (k+1)*N - 1]  →  params_.state_interfaces[k], all joints
  //   default: block 0 = position, block 1 = velocity
  //
  // "position"/"velocity" stay hardcoded to blocks 0/1. "effort" and
  // "gait_state" have no paired state block and are resolved by NAME (scanned
  // once in on_configure() into effort_block_idx_/gait_block_idx_) rather than
  // by position, since either or both may be absent or reordered.

  using ReferenceMsg = plan4ari_msgs::msg::JointCmd;
  rclcpp::Subscription<ReferenceMsg>::SharedPtr ref_sub_;
  realtime_tools::RealtimeThreadSafeBox<ReferenceMsg> input_ref_;

  int effort_block_idx_{-1};   // index of "effort" in params_.reference_interfaces, or -1
  int gait_block_idx_{-1};     // index of "gait_state" in params_.reference_interfaces, or -1
  size_t n_legs_{0};
  std::vector<int64_t> joint_to_leg_number_;   // size n_joints_; joint i -> its leg's leg_number

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
