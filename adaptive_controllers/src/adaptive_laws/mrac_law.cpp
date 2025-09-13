// Copyright 2025
#include "adaptive_controllers/adaptive_laws/mrac_law.hpp"
#include <algorithm>
namespace adaptive_controllers {

void MRACLaw::configure(const AdaptiveLawParams& p) {
  dof_ = p.dof;
  gamma_ = p.gamma;
  sigma_ = p.sigma;
  theta_max_ = p.theta_max;
  theta_.setZero(static_cast<Eigen::Index>(dof_), 3);
}

void MRACLaw::reset() {
  if (dof_ == 0) return;
  theta_.setZero();
}

void MRACLaw::update(const Eigen::VectorXd& xhat,
                     const Eigen::VectorXd& r,
                     Eigen::VectorXd& u_adapt) {
  const std::size_t n = static_cast<std::size_t>(r.size());
  if (n == 0) { u_adapt.resize(0); return; }
  if (dof_ != n) { // 尺寸發生變化時，重新配置
    dof_ = n;
    theta_.setZero(static_cast<Eigen::Index>(dof_), 3);
  }

  u_adapt.setZero(static_cast<Eigen::Index>(n));
  for (std::size_t i=0;i<n;++i) {
    const double pos = xhat(static_cast<Eigen::Index>(i));
    const double vel = xhat(static_cast<Eigen::Index>(n+i));
    const double ref = r(static_cast<Eigen::Index>(i));
    const double e   = pos - ref;

    const double phi[3] = {pos, vel, ref};

    // θ_dot = -γ * e * φ - σ θ
    for (int k=0;k<3;++k) {
      double td = -gamma_ * e * phi[k] - sigma_ * theta_(static_cast<Eigen::Index>(i), k);
      theta_(static_cast<Eigen::Index>(i), k) = proj(theta_(static_cast<Eigen::Index>(i), k) + td);
    }

    double ui = 0.0;
    for (int k=0;k<3;++k) ui += theta_(static_cast<Eigen::Index>(i), k) * phi[k];
    u_adapt(static_cast<Eigen::Index>(i)) = ui;
  }
}

} // namespace adaptive_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(adaptive_controllers::MRACLaw, adaptive_controllers::AdaptiveLawBase)
