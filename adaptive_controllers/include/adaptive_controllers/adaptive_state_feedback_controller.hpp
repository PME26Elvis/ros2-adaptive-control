// Copyright 2025
#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
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
  // Interfaces
  std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces_;
  std::vector<hardware_interface::LoanedStateInterface>   state_ifaces_;

  // Joints & sizes
  std::vector<std::string> joint_names_;
  std::size_t dof_{1};              // number of joints
  std::size_t nx_{2};               // state dimension = 2*dof

  // State-space
  Eigen::MatrixXd A_, B_, C_, K_;   // A(2n,2n), B(2n,n), C(n,2n), K(n,2n)
  Eigen::VectorXd x_, xhat_, u_, y_, r_;
  Eigen::VectorXd u_min_, u_max_;

  // Reference (non-RT -> RT)
  using RefMsg = std_msgs::msg::Float64MultiArray;
  rclcpp::Subscription<RefMsg>::SharedPtr ref_sub_;
  realtime_tools::RealtimeBuffer<Eigen::VectorXd> ref_rt_;

  // Debug (RT publisher)
  using DebugMsg = std_msgs::msg::Float64MultiArray;
  std::shared_ptr< realtime_tools::RealtimePublisher<DebugMsg> > dbg_pub_;

  // Plugins (only if enabled through parameter)
  bool        enable_plugins_{false};
  std::string observer_type_;
  std::string adaptive_type_;
  pluginlib::UniquePtr<ObserverBase>   observer_;
  pluginlib::UniquePtr<AdaptiveLawBase> adapt_;

  // Helpers
  void declare_common_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  bool make_default_mats();  // build default A,B,C for 2nd-order model
  bool read_and_validate_mats(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  void setup_reference_subscription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  void setup_debug_publisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  Eigen::VectorXd clamp(const Eigen::VectorXd& v,
                        const Eigen::VectorXd& vmin,
                        const Eigen::VectorXd& vmax) const;

  // Safe getters for names / sizes（留給現有程式碼相容）
  inline bool sized_ok() const { return (dof_ > 0) && (nx_ == 2*dof_) && (A_.size() > 0); }
  inline std::string joint_name_safe(std::size_t i) const {
    if (joint_names_.empty()) return "joint" + std::to_string(i);
    if (i < joint_names_.size()) return joint_names_[i];
    return "joint" + std::to_string(i);
  }
};

}  // namespace adaptive_controllers
