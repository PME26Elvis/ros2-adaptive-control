// Copyright 2025
#include "adaptive_controllers/adaptive_laws/mrac_law.hpp"
#include <algorithm>

namespace adaptive_controllers {

void MRACLaw::configure(const AdaptiveLawParams&) {
  // size will be set on first update based on xhat/r lengths
  theta_.resize(0,0);
  dof_ = 0;
  epos_.resize(0); evel_.resize(0); emix_.resize(0);
}

void MRACLaw::reset() {
  if (dof_ == 0) return;
  theta_.setZero(dof_, 3);
  epos_.setZero(dof_); evel_.setZero(dof_); emix_.setZero(dof_);
}

void MRACLaw::update(const Eigen::VectorXd& xhat,
                     const Eigen::VectorXd& r,
                     Eigen::VectorXd& u_adapt) {
  // xhat.size() = 2n, r.size() = n
  const std::size_t n = static_cast<std::size_t>(r.size());
  if (n == 0) { u_adapt.resize(0); return; }

  if (dof_ != n) {
    dof_ = n;
    theta_.setZero(dof_, 3);
    epos_.setZero(dof_); evel_.setZero(dof_); emix_.setZero(dof_);
  }

  u_adapt.resize(n);
  for (std::size_t i=0;i<n;++i) {
    const double pos = xhat(static_cast<Eigen::Index>(i));
    const double vel = xhat(static_cast<Eigen::Index>(n+i));
    const double ref = r(static_cast<Eigen::Index>(i));

    // simple tracking error terms
    const double e  = (pos - ref);
    const double ed = vel; // desired vel ~ 0 baseline
    epos_(static_cast<Eigen::Index>(i)) = e;
    evel_(static_cast<Eigen::Index>(i)) = ed;
    emix_(static_cast<Eigen::Index>(i)) = (ref); // regressor includes r

    // regressor phi = [pos, vel, r]
    const double phi[3] = {pos, vel, ref};

    // adaptation law: theta_dot = -gamma * e * phi - sigma * theta
    for (int k=0;k<3;++k) {
      double td = -prm_.gamma * e * phi[k] - prm_.sigma * theta_(static_cast<Eigen::Index>(i), k);
      theta_(static_cast<Eigen::Index>(i), k) += td;
      // projection
      theta_(static_cast<Eigen::Index>(i), k) =
        proj(theta_(static_cast<Eigen::Index>(i), k), prm_.theta_max);
    }

    // adaptive term u_i = theta_i^T * phi
    double ui = 0.0;
    for (int k=0;k<3;++k) ui += theta_(static_cast<Eigen::Index>(i), k) * phi[k];
    u_adapt(static_cast<Eigen::Index>(i)) = ui;
  }
}

} // namespace adaptive_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(adaptive_controllers::MRACLaw, adaptive_controllers::AdaptiveLawBase)
