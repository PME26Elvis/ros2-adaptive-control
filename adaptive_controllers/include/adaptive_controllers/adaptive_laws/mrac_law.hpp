// Copyright 2025
#pragma once
#include "adaptive_controllers/adaptive_law_base.hpp"
#include <Eigen/Dense>
#include <vector>

namespace adaptive_controllers {

struct MRACParams {
  double gamma{5.0};      // adaptation gain
  double sigma{0.01};     // sigma-mod to prevent drift
  double theta_max{50.0}; // projection bound (abs)
  // reference model (per joint, 2nd-order canonical)
  double wn{2.0};         // natural freq
  double zeta{0.9};       // damping
};

class MRACLaw : public AdaptiveLawBase {
public:
  MRACLaw() = default;
  ~MRACLaw() override = default;

  void configure(const AdaptiveLawParams& p) override;
  void reset() override;
  void update(const Eigen::VectorXd& xhat,
              const Eigen::VectorXd& r,
              Eigen::VectorXd& u_adapt) override;

private:
  MRACParams prm_;
  std::size_t dof_{1};

  // parameter per joint: theta ∈ R^3 for [pos, vel, r]
  Eigen::MatrixXd theta_;   // (dof x 3)
  Eigen::VectorXd epos_, evel_, emix_; // simple error terms for adaptation

  inline double proj(double v, double vabs_max) const {
    if (v >  vabs_max) return  vabs_max;
    if (v < -vabs_max) return -vabs_max;
    return v;
  }
};

} // namespace adaptive_controllers
