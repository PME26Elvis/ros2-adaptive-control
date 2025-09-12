#pragma once
#include <Eigen/Dense>

namespace adaptive_controllers {
struct ObserverParams {};
class ObserverBase {
 public:
  virtual ~ObserverBase() = default;
  virtual void configure(const ObserverParams&) = 0;
  virtual void reset() = 0;
  virtual void estimate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                        Eigen::VectorXd& xhat) = 0;
};
}  // namespace adaptive_controllers
