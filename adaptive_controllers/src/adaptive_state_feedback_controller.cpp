// Copyright 2025 
#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

using controller_interface::InterfaceConfiguration;
using controller_interface::ControllerInterface;
using controller_interface::return_type;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace adaptive_controllers {

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_init() {
  // 不在 on_init 建立 ClassLoader，避免 build 空間無 ament 索引時丟例外
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  // 參數：為了避免 ParameterValue(unsigned long) 歧義，用 int
  node->declare_parameter<int>("dof", 1);
  node->declare_parameter<std::string>("observer_type", "adaptive_controllers/LuenbergerObserver");
  node->declare_parameter<std::string>("adaptive_type", "adaptive_controllers/MRACLaw");
  // 關鍵：預設不載入 plugin，測試環境不觸發 pluginlib
  node->declare_parameter<bool>("enable_plugins", false);

  const int dof_param = node->get_parameter("dof").as_int();
  dof_ = dof_param > 0 ? static_cast<std::size_t>(dof_param) : 1;

  observer_type_ = node->get_parameter("observer_type").as_string();
  adaptive_type_ = node->get_parameter("adaptive_type").as_string();
  enable_plugins_ = node->get_parameter("enable_plugins").as_bool();

  // Minimal placeholders
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

  if (enable_plugins_) {
    // 只有在 enable_plugins_=true 才嘗試載入，避免在 build 空間找不到套件而丟例外
    try {
      pluginlib::ClassLoader<ObserverBase> observer_loader(
          "adaptive_controllers", "adaptive_controllers::ObserverBase");
      observer_ = observer_loader.createUniqueInstance(observer_type_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node->get_logger(), "Observer plugin load failed: %s", e.what());
    } catch (...) {
      RCLCPP_WARN(node->get_logger(), "Observer plugin load failed (unknown).");
    }

    try {
      pluginlib::ClassLoader<AdaptiveLawBase> adapt_loader(
          "adaptive_controllers", "adaptive_controllers::AdaptiveLawBase");
      adapt_ = adapt_loader.createUniqueInstance(adaptive_type_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node->get_logger(), "Adaptive law plugin load failed: %s", e.what());
    } catch (...) {
      RCLCPP_WARN(node->get_logger(), "Adaptive law plugin load failed (unknown).");
    }

    if (observer_) {
      ObserverParams p;
      observer_->configure(p);
      observer_->reset();
    }
    if (adapt_) {
      AdaptiveLawParams p;
      adapt_->configure(p);
      adapt_->reset();
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_activate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  for (auto &iface : command_interfaces_) cmd_ifaces_.push_back(std::move(iface));
  for (auto &iface : state_interfaces_) state_ifaces_.push_back(std::move(iface));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_deactivate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration AdaptiveStateFeedbackController::command_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_EFFORT);
  }
  return cfg;
}

InterfaceConfiguration AdaptiveStateFeedbackController::state_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(dof_ * 2);
  for (std::size_t i = 0; i < dof_; ++i) {
    cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_POSITION);
    cfg.names.push_back("joint" + std::to_string(i) + "/" + HW_IF_VELOCITY);
  }
  return cfg;
}

return_type AdaptiveStateFeedbackController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  if (state_ifaces_.empty() || cmd_ifaces_.empty()) {
    return return_type::OK;
  }

  // Minimal 1-DOF placeholder wiring (extend for multi-DOF later)
  for (std::size_t i = 0; i < dof_; ++i) {
    const double pos = state_ifaces_[2 * i].get_value();
    const double vel = state_ifaces_[2 * i + 1].get_value();
    (void)vel; // not used yet
    x_[0] = pos;
    y_[0] = pos;
  }

  Eigen::VectorXd xhat = x_;
  if (enable_plugins_ && observer_) observer_->estimate(x_, u_, y_, xhat);

  Eigen::VectorXd u_adapt = Eigen::VectorXd::Zero(dof_);
  if (enable_plugins_ && adapt_) adapt_->update(xhat, r_, u_adapt);

  u_ = -K_ * xhat + u_adapt;

  for (std::size_t i = 0; i < dof_; ++i) {
    cmd_ifaces_[i].set_value(u_[0]);
  }
  return return_type::OK;
}

}  // namespace adaptive_controllers

PLUGINLIB_EXPORT_CLASS(adaptive_controllers::AdaptiveStateFeedbackController,
                       controller_interface::ControllerInterface)
