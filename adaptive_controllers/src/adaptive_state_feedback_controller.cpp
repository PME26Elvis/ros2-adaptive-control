#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

using controller_interface::InterfaceConfiguration;
using controller_interface::ControllerInterface;
using controller_interface::return_type;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace adaptive_controllers {

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_init() {
  observer_loader_ = std::make_unique<pluginlib::ClassLoader<ObserverBase>>(
      "adaptive_controllers", "adaptive_controllers::ObserverBase");
  adapt_loader_ = std::make_unique<pluginlib::ClassLoader<AdaptiveLawBase>>(
      "adaptive_controllers", "adaptive_controllers::AdaptiveLawBase");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();
  node->declare_parameter<std::size_t>("dof", 1);
  node->declare_parameter<std::string>("observer_type", "adaptive_controllers/LuenbergerObserver");
  node->declare_parameter<std::string>("adaptive_type", "adaptive_controllers/MRACLaw");

  dof_ = node->get_parameter("dof").as_int();
  observer_type_ = node->get_parameter("observer_type").as_string();
  adaptive_type_ = node->get_parameter("adaptive_type").as_string();

  A_.setZero(dof_, dof_);
  B_.setIdentity(dof_, dof_);
  C_.setIdentity(dof_, dof_);
  K_.setZero(dof_, dof_);
  x_.setZero(dof_);
  u_.setZero(dof_);
  y_.setZero(dof_);
  r_.setZero(dof_);

  observer_.reset();
  adapt_.reset();
  try { observer_ = observer_loader_->createUniqueInstance(observer_type_); } catch(...) {}
  try { adapt_ = adapt_loader_->createUniqueInstance(adaptive_type_); } catch(...) {}

  if(observer_) { ObserverParams p; observer_->configure(p); observer_->reset(); }
  if(adapt_) { AdaptiveLawParams p; adapt_->configure(p); adapt_->reset(); }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_activate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  for (auto & iface : command_interfaces_) cmd_ifaces_.push_back(std::move(iface));
  for (auto & iface : state_interfaces_) state_ifaces_.push_back(std::move(iface));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_deactivate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration AdaptiveStateFeedbackController::command_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = InterfaceConfiguration::INDIVIDUAL;
  cfg.names.reserve(dof_);
  for (size_t i = 0; i < dof_; ++i) cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_EFFORT);
  return cfg;
}

InterfaceConfiguration AdaptiveStateFeedbackController::state_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = InterfaceConfiguration::INDIVIDUAL;
  for (size_t i = 0; i < dof_; ++i) {
    cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_POSITION);
    cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_VELOCITY);
  }
  return cfg;
}

return_type AdaptiveStateFeedbackController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  if (state_ifaces_.empty() || cmd_ifaces_.empty()) return return_type::OK;
  for (size_t i = 0; i < dof_; ++i) {
    double pos = state_ifaces_[2*i].get_value();
    double vel = state_ifaces_[2*i + 1].get_value();
    x_[0] = pos; y_[0] = pos;
  }
  Eigen::VectorXd xhat = x_;
  if (observer_) observer_->estimate(x_, u_, y_, xhat);
  Eigen::VectorXd u_adapt = Eigen::VectorXd::Zero(dof_);
  if (adapt_) adapt_->update(xhat, r_, u_adapt);
  u_ = -K_ * xhat + u_adapt;
  for (size_t i = 0; i < dof_; ++i) cmd_ifaces_[i].set_value(u_[0]);
  return return_type::OK;
}

}  // namespace adaptive_controllers

PLUGINLIB_EXPORT_CLASS(adaptive_controllers::AdaptiveStateFeedbackController, controller_interface::ControllerInterface)
