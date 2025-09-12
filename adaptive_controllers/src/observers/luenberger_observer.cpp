#include "adaptive_controllers/observers/luenberger_observer.hpp"
using Eigen::MatrixXd; using Eigen::VectorXd;
namespace adaptive_controllers {
void LuenbergerObserver::configure(const ObserverParams&) {
  A_.resize(1,1); B_.resize(1,1); C_.resize(1,1); L_.resize(1,1);
  A_(0,0)=0.0; B_(0,0)=1.0; C_(0,0)=1.0; L_(0,0)=1.0;
  xhat_.setZero(1);
  configured_=true;
}
void LuenbergerObserver::reset(){ xhat_.setZero(); }
void LuenbergerObserver::estimate(const VectorXd& x, const VectorXd& u, const VectorXd& y, VectorXd& xhat){
  if(!configured_) return;
  VectorXd yhat = C_*xhat_;
  VectorXd dx = A_*xhat_ + B_*u + L_*(y - yhat);
  xhat_ = xhat_ + dx;
  xhat = xhat_;
}
}  // namespace adaptive_controllers
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(adaptive_controllers::LuenbergerObserver, adaptive_controllers::ObserverBase)
