#pragma once
#include "adaptive_controllers/adaptive_law_base.hpp"
#include <Eigen/Dense>

namespace adaptive_controllers {
class MRACLaw : public AdaptiveLawBase {
 public:
  void configure(const AdaptiveLawParams&) override;
  void reset() override;
  void update(const Eigen::VectorXd& xhat, const Eigen::VectorXd& r,
              Eigen::VectorXd& u_adapt) override;
 private:
  Eigen::VectorXd theta_;
  double gamma_{1.0};
  bool configured_{false};
};
}  // namespace adaptive_controllers
