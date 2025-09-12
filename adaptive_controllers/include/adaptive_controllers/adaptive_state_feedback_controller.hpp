#pragma once
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <Eigen/Dense>
#include "adaptive_controllers/observer_base.hpp"
#include "adaptive_controllers/adaptive_law_base.hpp"

namespace adaptive_controllers {
class AdaptiveStateFeedbackController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

 private:
  std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_ifaces_;

  Eigen::MatrixXd A_, B_, C_, K_;
  Eigen::VectorXd x_, u_, y_, r_;

  std::unique_ptr<pluginlib::ClassLoader<ObserverBase>> observer_loader_;
  std::unique_ptr<pluginlib::ClassLoader<AdaptiveLawBase>> adapt_loader_;
  std::unique_ptr<ObserverBase> observer_;
  std::unique_ptr<AdaptiveLawBase> adapt_;

  std::string observer_type_;
  std::string adaptive_type_;
  size_t dof_{1};
};
}  // namespace adaptive_controllers
