// Copyright 2025
#include "adaptive_controllers/adaptive_laws/l1_law.hpp"
#include <algorithm>

namespace adaptive_controllers {

void L1Law::configure(const AdaptiveLawParams& p) {
  dof_ = p.dof;
  gamma_ = p.gamma;
  sigma_ = p.sigma;
  theta_max_ = p.theta_max;
  alpha_ = p.l1_alpha;     // 濾波強度
  theta_.setZero(static_cast<Eigen::Index>(dof_), 3);
  ui_last_.setZero(static_cast<Eigen::Index>(dof_));
}

void L1Law::reset() {
  if (dof_ == 0) return;
  theta_.setZero();
  ui_last_.setZero();
}

void L1Law::update(const Eigen::VectorXd& xhat,
                   const Eigen::VectorXd& r,
                   Eigen::VectorXd& u_adapt) {
  const std::size_t n = static_cast<std::size_t>(r.size());
  if (n == 0) { u_adapt.resize(0); return; }
  if (dof_ != n) {
    dof_ = n;
    theta_.setZero(static_cast<Eigen::Index>(dof_), 3);
    ui_last_.setZero(static_cast<Eigen::Index>(dof_));
  }

  // 假設控制週期 Δt 由上層（controller update rate）主導；此處採 α 的離散等效：y_k = (αΔt)/(1+αΔt) u_k + 1/(1+αΔt) y_{k-1}
  const double dt_alpha = alpha_;  // 以 alpha 代表連續極點，Δt 藏在比例（簡化）
  const double a = dt_alpha / (1.0 + dt_alpha);
  const double b = 1.0 / (1.0 + dt_alpha);

  u_adapt.setZero(static_cast<Eigen::Index>(n));
  for (std::size_t i=0;i<n;++i) {
    const double pos = xhat(static_cast<Eigen::Index>(i));
    const double vel = xhat(static_cast<Eigen::Index>(n+i));
    const double ref = r(static_cast<Eigen::Index>(i));
    const double e   = pos - ref;

    const double phi[3] = {pos, vel, ref};

    for (int k=0;k<3;++k) {
      double td = -gamma_ * e * phi[k] - sigma_ * theta_(static_cast<Eigen::Index>(i), k);
      theta_(static_cast<Eigen::Index>(i), k) = proj(theta_(static_cast<Eigen::Index>(i), k) + td);
    }

    double ui = 0.0;
    for (int k=0;k<3;++k) ui += theta_(static_cast<Eigen::Index>(i), k) * phi[k];

    // L1 濾波
    double yi = a * ui + b * ui_last_(static_cast<Eigen::Index>(i));
    ui_last_(static_cast<Eigen::Index>(i)) = yi;
    u_adapt(static_cast<Eigen::Index>(i)) = yi;
  }
}

} // namespace adaptive_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(adaptive_controllers::L1Law, adaptive_controllers::AdaptiveLawBase)
