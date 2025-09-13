// Copyright 2025
#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "adaptive_controllers/param_utils.hpp"

using controller_interface::InterfaceConfiguration;
using controller_interface::ControllerInterface;
using controller_interface::return_type;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using adaptive_controllers::param_utils::read_matrix;

namespace adaptive_controllers {

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_init() {
  // Seed minimal 1-DOF buffers，避免未配置時越界
  dof_ = 1;
  A_.setZero(1,1); B_.setIdentity(1,1); C_.setIdentity(1,1); K_.setZero(1,1);
  x_.setZero(1); u_.setZero(1); y_.setZero(1); r_.setZero(1);
  u_min_ = Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity());
  u_max_ = Eigen::VectorXd::Constant(1,  std::numeric_limits<double>::infinity());
  joint_names_.clear();
  enable_plugins_ = false;
  observer_.reset(); adapt_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdaptiveStateFeedbackController::declare_common_parameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  node->declare_parameter<int>("dof", 1);
  node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  node->declare_parameter<std::string>("observer_type", "adaptive_controllers/LuenbergerObserver");
  node->declare_parameter<std::string>("adaptive_type", "adaptive_controllers/MRACLaw");
  node->declare_parameter<bool>("enable_plugins", false);
  node->declare_parameter<std::vector<double>>("u_min", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("u_max", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("r0", std::vector<double>{});
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  declare_common_parameters(node);

  const int dof_param = node->get_parameter("dof").as_int();
  dof_ = dof_param > 0 ? static_cast<std::size_t>(dof_param) : 1;

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (!joint_names_.empty() && joint_names_.size() != dof_) {
    RCLCPP_ERROR(node->get_logger(), "joints size (%zu) != dof (%zu)", joint_names_.size(), dof_);
    return controller_interface::CallbackReturn::ERROR;
  }

  observer_type_  = node->get_parameter("observer_type").as_string();
  adaptive_type_  = node->get_parameter("adaptive_type").as_string();
  enable_plugins_ = node->get_parameter("enable_plugins").as_bool();

  // Resize buffers with new dof
  A_.setZero(dof_, dof_); B_.setIdentity(dof_, dof_);
  C_.setIdentity(dof_, dof_); K_.setZero(dof_, dof_);
  x_.setZero(dof_); u_.setZero(dof_); y_.setZero(dof_); r_.setZero(dof_);
  u_min_ = Eigen::VectorXd::Constant(dof_, -std::numeric_limits<double>::infinity());
  u_max_ = Eigen::VectorXd::Constant(dof_,  std::numeric_limits<double>::infinity());

  (void)read_matrix(node, "A", dof_, dof_, A_);
  (void)read_matrix(node, "B", dof_, dof_, B_);
  (void)read_matrix(node, "C", dof_, dof_, C_);
  (void)read_matrix(node, "K", dof_, dof_, K_);

  // Limits
  auto vmin = node->get_parameter("u_min").as_double_array();
  auto vmax = node->get_parameter("u_max").as_double_array();
  if (vmin.size() == dof_) for (std::size_t i=0;i<dof_;++i) u_min_[static_cast<Eigen::Index>(i)] = vmin[i];
  if (vmax.size() == dof_) for (std::size_t i=0;i<dof_;++i) u_max_[static_cast<Eigen::Index>(i)] = vmax[i];

  // Initial reference
  auto r0 = node->get_parameter("r0").as_double_array();
  if (!r0.empty() && r0.size() != dof_) {
    RCLCPP_ERROR(node->get_logger(), "r0 size (%zu) != dof (%zu)", r0.size(), dof_);
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i=0;i<r0.size();++i) r_[static_cast<Eigen::Index>(i)] = r0[i];

  // Keep ref subscription disabled in unit tests (no lifecycle node is fine)
  // We only create it when controller is run inside controller_manager.
  // (If you later need it here, ensure a rclcpp context/node exists.)

  observer_.reset(); adapt_.reset();
  if (enable_plugins_) {
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
  ref_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration AdaptiveStateFeedbackController::command_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    cfg.names.push_back(joint_name_safe(i) + "/" + HW_IF_EFFORT);
  }
  return cfg;
}

InterfaceConfiguration AdaptiveStateFeedbackController::state_interface_configuration() const {
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(dof_ * 2);
  for (std::size_t i = 0; i < dof_; ++i) {
    cfg.names.push_back(joint_name_safe(i) + "/" + HW_IF_POSITION);
    cfg.names.push_back(joint_name_safe(i) + "/" + HW_IF_VELOCITY);
  }
  return cfg;
}

Eigen::VectorXd AdaptiveStateFeedbackController::clamp(
    const Eigen::VectorXd& v, const Eigen::VectorXd& vmin, const Eigen::VectorXd& vmax) const {
  Eigen::VectorXd out = v;
  for (Eigen::Index i = 0; i < out.size(); ++i) {
    if (out[i] < vmin[i]) out[i] = vmin[i];
    if (out[i] > vmax[i]) out[i] = vmax[i];
  }
  return out;
}

return_type AdaptiveStateFeedbackController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  // 若尚未配置好大小（或 dof_==0），直接 no-op
  if (!sized_ok()) {
    return return_type::OK;
  }
  if (state_ifaces_.empty() || cmd_ifaces_.empty()) {
    return return_type::OK;
  }

  // Minimal 1-state placeholder（之後擴充為全狀態）
  for (std::size_t i = 0; i < dof_; ++i) {
    const double pos = state_ifaces_[2 * i].get_value();
    (void)pos;
    x_(0) = pos;
    y_(0) = pos;
  }

  Eigen::VectorXd xhat = x_;
  if (enable_plugins_ && observer_) observer_->estimate(x_, u_, y_, xhat);

  Eigen::VectorXd u_adapt = Eigen::VectorXd::Zero(dof_);
  if (enable_plugins_ && adapt_) adapt_->update(xhat, r_, u_adapt);

  Eigen::VectorXd u_cmd = -K_ * xhat + u_adapt;
  u_cmd = clamp(u_cmd, u_min_, u_max_);

  for (std::size_t i = 0; i < dof_; ++i) {
    cmd_ifaces_[i].set_value(u_cmd(0));
  }
  return return_type::OK;
}

}  // namespace adaptive_controllers

PLUGINLIB_EXPORT_CLASS(adaptive_controllers::AdaptiveStateFeedbackController,
                       controller_interface::ControllerInterface)
