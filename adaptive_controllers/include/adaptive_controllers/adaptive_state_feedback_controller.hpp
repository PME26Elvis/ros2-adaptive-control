// Copyright 2025
#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <Eigen/Dense>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "adaptive_controllers/observer_base.hpp"
#include "adaptive_controllers/adaptive_law_base.hpp"

namespace adaptive_controllers {

class AdaptiveStateFeedbackController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  // Interface claims
  std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_ifaces_;

  // Joint names and DOF
  std::vector<std::string> joint_names_;
  std::size_t dof_{1};

  // State-space matrices and signals
  Eigen::MatrixXd A_, B_, C_, K_;
  Eigen::VectorXd x_, u_, y_, r_;          // plant states, control, output, reference
  Eigen::VectorXd u_min_, u_max_;          // command limits (per joint)

  // Reference input subscription (non-real-time → realtime buffer bridge)
  using RefMsg = std_msgs::msg::Float64MultiArray;
  rclcpp::Subscription<RefMsg>::SharedPtr ref_sub_;
  realtime_tools::RealtimeBuffer<Eigen::VectorXd> ref_rt_;

  // Plugins (only if enabled through parameter)
  bool enable_plugins_{false};
  std::string observer_type_;
  std::string adaptive_type_;
  pluginlib::UniquePtr<ObserverBase> observer_;
  pluginlib::UniquePtr<AdaptiveLawBase> adapt_;

  // Helpers
  void declare_common_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  void setup_reference_subscription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  Eigen::VectorXd clamp(const Eigen::VectorXd& v,
                        const Eigen::VectorXd& vmin,
                        const Eigen::VectorXd& vmax) const;
};

}  // namespace adaptive_controllers
