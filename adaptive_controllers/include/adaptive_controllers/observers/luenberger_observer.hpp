#pragma once
#include "adaptive_controllers/observer_base.hpp"
#include <Eigen/Dense>

namespace adaptive_controllers {
class LuenbergerObserver : public ObserverBase {
 public:
  void configure(const ObserverParams&) override;
  void reset() override;
  void estimate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                Eigen::VectorXd& xhat) override;
 private:
  Eigen::MatrixXd A_, B_, C_, L_;
  Eigen::VectorXd xhat_;
  bool configured_{false};
};
}  // namespace adaptive_controllers
