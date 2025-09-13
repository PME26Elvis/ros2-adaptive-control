// Copyright 2025
#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "adaptive_controllers/param_utils.hpp"

using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using adaptive_controllers::param_utils::read_matrix;

namespace adaptive_controllers {

controller_interface::CallbackReturn AdaptiveStateFeedbackController::on_init() {
  // seed 1-DOF default
  dof_ = 1; nx_ = 2*dof_;
  A_.setZero(nx_, nx_); B_.setZero(nx_, dof_); C_.setZero(dof_, nx_); K_.setZero(dof_, nx_);
  // default 2nd-order canonical
  make_default_mats();
  x_.setZero(nx_); xhat_.setZero(nx_);
  u_.setZero(dof_); y_.setZero(dof_); r_.setZero(dof_);
  u_min_ = Eigen::VectorXd::Constant(dof_, -std::numeric_limits<double>::infinity());
  u_max_ = Eigen::VectorXd::Constant(dof_,  std::numeric_limits<double>::infinity());
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
  node->declare_parameter<std::vector<double>>("r0",   std::vector<double>{});
}

bool AdaptiveStateFeedbackController::make_default_mats() {
  // default 2nd-order per joint: x=[pos,vel], u acts on acceleration
  // A = [0 I; 0 0], B = [0; I], C = [I 0]
  A_.setZero(nx_, nx_);
  for (std::size_t i=0;i<dof_;++i) {
    // pos'(i) = vel(i)
    A_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(dof_+i)) = 1.0;
  }
  B_.setZero(nx_, dof_);
  for (std::size_t i=0;i<dof_;++i) {
    B_(static_cast<Eigen::Index>(dof_+i), static_cast<Eigen::Index>(i)) = 1.0;
  }
  C_.setZero(dof_, nx_);
  for (std::size_t i=0;i<dof_;++i) {
    C_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) = 1.0;
  }
  // K default = 0
  K_.setZero(dof_, nx_);
  return true;
}

bool AdaptiveStateFeedbackController::read_and_validate_mats(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // try read; if unset keep defaults
  (void)read_matrix(node, "A", nx_,   nx_,   A_);
  (void)read_matrix(node, "B", nx_,   dof_,  B_);
  (void)read_matrix(node, "C", dof_,  nx_,   C_);
  (void)read_matrix(node, "K", dof_,  nx_,   K_);
  // no further numeric validation here (留給未來加穩定性檢查)
  return true;
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();
  declare_common_parameters(node);

  const int dof_param = node->get_parameter("dof").as_int();
  dof_ = dof_param > 0 ? static_cast<std::size_t>(dof_param) : 1;
  nx_  = 2 * dof_;

  // joints
  joint_names_ = node->get_parameter("joints").as_string_array();
  if (!joint_names_.empty() && joint_names_.size() != dof_) {
    RCLCPP_ERROR(node->get_logger(), "joints size (%zu) != dof (%zu)", joint_names_.size(), dof_);
    return controller_interface::CallbackReturn::ERROR;
  }

  observer_type_  = node->get_parameter("observer_type").as_string();
  adaptive_type_  = node->get_parameter("adaptive_type").as_string();
  enable_plugins_ = node->get_parameter("enable_plugins").as_bool();

  // resize & defaults
  A_.setZero(nx_, nx_); B_.setZero(nx_, dof_); C_.setZero(dof_, nx_); K_.setZero(dof_, nx_);
  make_default_mats();
  x_.setZero(nx_); xhat_.setZero(nx_);
  u_.setZero(dof_); y_.setZero(dof_); r_.setZero(dof_);
  u_min_ = Eigen::VectorXd::Constant(dof_, -std::numeric_limits<double>::infinity());
  u_max_ = Eigen::VectorXd::Constant(dof_,  std::numeric_limits<double>::infinity());

  // optional overrides
  read_and_validate_mats(node);

  // limits
  auto vmin = node->get_parameter("u_min").as_double_array();
  auto vmax = node->get_parameter("u_max").as_double_array();
  if (vmin.size() == dof_) for (std::size_t i=0;i<dof_;++i) u_min_[static_cast<Eigen::Index>(i)] = vmin[i];
  if (vmax.size() == dof_) for (std::size_t i=0;i<dof_;++i) u_max_[static_cast<Eigen::Index>(i)] = vmax[i];

  // r0
  auto r0 = node->get_parameter("r0").as_double_array();
  if (!r0.empty() && r0.size() != dof_) {
    RCLCPP_ERROR(node->get_logger(), "r0 size (%zu) != dof (%zu)", r0.size(), dof_);
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i=0;i<r0.size();++i) r_[static_cast<Eigen::Index>(i)] = r0[i];

  // subscriptions / debug pub（只在 manager 環境有效）
  setup_reference_subscription(node);
  setup_debug_publisher(node);

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

    if (observer_) { ObserverParams p; observer_->configure(p); observer_->reset(); }
    if (adapt_)    { AdaptiveLawParams p; adapt_->configure(p);    adapt_->reset(); }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void AdaptiveStateFeedbackController::setup_reference_subscription(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  ref_rt_.writeFromNonRT(r_);  // seed
  auto cb = [this](const RefMsg::SharedPtr msg) {
    if (!msg) return;
    Eigen::VectorXd r_new = r_;
    const auto n = std::min<std::size_t>(dof_, msg->data.size());
    for (std::size_t i=0;i<n;++i) r_new(static_cast<Eigen::Index>(i)) = msg->data[i];
    ref_rt_.writeFromNonRT(r_new);
  };
  ref_sub_ = node->create_subscription<RefMsg>("~/reference", rclcpp::SystemDefaultsQoS(), cb);
}

void AdaptiveStateFeedbackController::setup_debug_publisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  dbg_pub_.reset(new realtime_tools::RealtimePublisher<DebugMsg>(node, "~/debug", 10));
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_activate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  for (auto &iface : command_interfaces_) cmd_ifaces_.push_back(std::move(iface));
  for (auto &iface : state_interfaces_)   state_ifaces_.push_back(std::move(iface));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveStateFeedbackController::on_deactivate(const rclcpp_lifecycle::State&) {
  cmd_ifaces_.clear();
  state_ifaces_.clear();
  ref_sub_.reset();
  dbg_pub_.reset();
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
  if (!sized_ok() || state_ifaces_.empty() || cmd_ifaces_.empty()) {
    return return_type::OK;
  }

  // Read states: x = [pos(0..n-1), vel(0..n-1)]
  for (std::size_t i = 0; i < dof_; ++i) {
    const double pos = state_ifaces_[2 * i].get_value();
    const double vel = state_ifaces_[2 * i + 1].get_value();
    x_(static_cast<Eigen::Index>(i))        = pos;
    x_(static_cast<Eigen::Index>(dof_ + i)) = vel;
  }
  y_ = C_ * x_;
  if (auto r_ptr = ref_rt_.readFromRT()) r_ = *r_ptr;

  xhat_ = x_;
  if (enable_plugins_ && observer_) observer_->estimate(x_, u_, y_, xhat_);

  Eigen::VectorXd u_adapt = Eigen::VectorXd::Zero(dof_);
  if (enable_plugins_ && adapt_)   adapt_->update(xhat_, r_, u_adapt);

  Eigen::VectorXd u_cmd = (-K_ * xhat_) + u_adapt;
  u_cmd = clamp(u_cmd, u_min_, u_max_);

  for (std::size_t i = 0; i < dof_; ++i) {
    cmd_ifaces_[i].set_value(u_cmd(static_cast<Eigen::Index>(i)));
  }

  // Debug publish: [x(2n) | xhat(2n) | u(n) | r(n)]
  if (dbg_pub_ && dbg_pub_->trylock()) {
    auto &msg = dbg_pub_->msg_;
    msg.data.resize(nx_ + nx_ + dof_ + dof_);
    std::size_t k = 0;
    for (Eigen::Index i=0;i<x_.size();++i)   msg.data[k++] = x_(i);
    for (Eigen::Index i=0;i<xhat_.size();++i)msg.data[k++] = xhat_(i);
    for (Eigen::Index i=0;i<u_cmd.size();++i)msg.data[k++] = u_cmd(i);
    for (Eigen::Index i=0;i<r_.size();++i)   msg.data[k++] = r_(i);
    dbg_pub_->unlockAndPublish();
  }

  return return_type::OK;
}

}  // namespace adaptive_controllers

PLUGINLIB_EXPORT_CLASS(adaptive_controllers::AdaptiveStateFeedbackController,
                       controller_interface::ControllerInterface)
