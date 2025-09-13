// Copyright 2025
#pragma once
#include "adaptive_controllers/adaptive_law_base.hpp"
#include <Eigen/Dense>

namespace adaptive_controllers {

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
  std::size_t dof_{0};
  double gamma_{5.0}, sigma_{0.01}, theta_max_{50.0};
  // 每關節 3 個參數：phi = [pos, vel, r]
  Eigen::MatrixXd theta_; // (dof x 3)

  inline double proj(double v) const {
    if (v >  theta_max_) return  theta_max_;
    if (v < -theta_max_) return -theta_max_;
    return v;
  }
};

} // namespace adaptive_controllers
