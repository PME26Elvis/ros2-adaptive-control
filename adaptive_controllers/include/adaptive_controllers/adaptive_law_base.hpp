#pragma once
#include <Eigen/Dense>

namespace adaptive_controllers {
struct AdaptiveLawParams {};
class AdaptiveLawBase {
 public:
  virtual ~AdaptiveLawBase() = default;
  virtual void configure(const AdaptiveLawParams&) = 0;
  virtual void reset() = 0;
  virtual void update(const Eigen::VectorXd& xhat, const Eigen::VectorXd& r,
                      Eigen::VectorXd& u_adapt) = 0;
};
}  // namespace adaptive_controllers
