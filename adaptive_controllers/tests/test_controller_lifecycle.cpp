// Copyright 2025
#include <gtest/gtest.h>
#include "adaptive_controllers/adaptive_state_feedback_controller.hpp"

using adaptive_controllers::AdaptiveStateFeedbackController;
using controller_interface::interface_configuration_type;

TEST(AdaptiveController, LifecycleSmokeWithoutNode) {
  // 只測基礎介面宣告與 on_init，不觸發 on_configure（需 lifecycle node）
  auto ctrl = std::make_shared<AdaptiveStateFeedbackController>();

  // on_init 應該成功（不依賴 controller_manager）
  EXPECT_EQ(ctrl->on_init(), controller_interface::CallbackReturn::SUCCESS);

  // 介面宣告：預設 dof=1，命令介面 1 個、狀態介面 2 個（pos/vel）
  auto cmd_cfg = ctrl->command_interface_configuration();
  auto st_cfg  = ctrl->state_interface_configuration();

  EXPECT_EQ(cmd_cfg.type, interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(st_cfg.type,  interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(cmd_cfg.names.size(), 1u);
  EXPECT_EQ(st_cfg.names.size(), 2u);

  // 不呼叫 on_configure / on_activate，因為少了 lifecycle node 會丟例外。
  // update() 在未 claim 介面時應該安全返回 OK（no-op）。
  EXPECT_EQ(
    ctrl->update(rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
                 rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}
