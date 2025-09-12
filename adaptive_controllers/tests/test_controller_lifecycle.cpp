// Copyright 2025 
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"

using adaptive_controllers::AdaptiveStateFeedbackController;
using controller_interface::interface_configuration_type;

TEST(AdaptiveController, LifecycleSmoke) {
  auto ctrl = std::make_shared<AdaptiveStateFeedbackController>();
  EXPECT_EQ(ctrl->on_init(), controller_interface::CallbackReturn::SUCCESS);

  // configure
  EXPECT_EQ(ctrl->on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);

  // interface declarations
  auto cmd_cfg = ctrl->command_interface_configuration();
  auto st_cfg  = ctrl->state_interface_configuration();
  EXPECT_EQ(cmd_cfg.type, interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(st_cfg.type,  interface_configuration_type::INDIVIDUAL);
  // 預設 dof=1：命令 1 個、狀態 2 個（pos/vel）
  EXPECT_EQ(cmd_cfg.names.size(), 1u);
  EXPECT_EQ(st_cfg.names.size(), 2u);

  // activate/deactivate
  EXPECT_EQ(ctrl->on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl->on_deactivate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
}
