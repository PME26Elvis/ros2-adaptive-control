// Copyright 2025 
#pragma once
#include <Eigen/Dense>
#include <cstddef>

namespace adaptive_controllers {

// 傳給各種自適應法則的通用參數包
struct AdaptiveLawParams {
  std::size_t dof{1};       // 幾個關節（等於 r.size()）
  // MRAC/L1 共同
  double gamma{5.0};        // 適應增益
  double sigma{0.01};       // sigma-mod（抗漂移）
  double theta_max{50.0};   // 參數投影界限 |theta| <= theta_max
  // L1 專屬
  double l1_alpha{20.0};    // 一階濾波器極點（rad/s）
};

class AdaptiveLawBase {
public:
  virtual ~AdaptiveLawBase() = default;
  virtual void configure(const AdaptiveLawParams& p) = 0;
  virtual void reset() = 0;
  virtual void update(const Eigen::VectorXd& xhat,
                      const Eigen::VectorXd& r,
                      Eigen::VectorXd& u_adapt) = 0;
};

}  // namespace adaptive_controllers
