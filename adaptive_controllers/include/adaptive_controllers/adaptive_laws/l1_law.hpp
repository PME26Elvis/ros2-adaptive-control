// Copyright 2025
#pragma once
#include "adaptive_controllers/adaptive_law_base.hpp"
#include <Eigen/Dense>

namespace adaptive_controllers {

// 最小可用 L1：phi = [pos, vel, r]，快速適應 + 一階濾波器
class L1Law : public AdaptiveLawBase {
public:
  void configure(const AdaptiveLawParams& p) override;
  void reset() override;
  void update(const Eigen::VectorXd& xhat,
              const Eigen::VectorXd& r,
              Eigen::VectorXd& u_adapt) override;

private:
  std::size_t dof_{0};
  double gamma_{10.0}, sigma_{0.02}, theta_max_{50.0}, alpha_{20.0}; // α 是濾波極點
  Eigen::MatrixXd theta_;     // (dof x 3)
  Eigen::VectorXd ui_last_;   // 濾波前上一個輸出（每關節）
  inline double proj(double v) const {
    if (v >  theta_max_) return  theta_max_;
    if (v < -theta_max_) return -theta_max_;
    return v;
  }
};

} // namespace adaptive_controllers
