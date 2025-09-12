#include "adaptive_controllers/adaptive_laws/mrac_law.hpp"
using Eigen::VectorXd;
namespace adaptive_controllers {
void MRACLaw::configure(const AdaptiveLawParams&) {
  theta_.setZero(1);
  gamma_=1.0;
  configured_=true;
}
void MRACLaw::reset(){ theta_.setZero(); }
void MRACLaw::update(const VectorXd& xhat, const VectorXd& r, VectorXd& u_adapt){
  if(!configured_) return;
  double e = (r.size()>0? r[0]:0.0) - (xhat.size()>0? xhat[0]:0.0);
  theta_[0] += gamma_*e;
  u_adapt.resize(1);
  u_adapt[0] = theta_[0]*e;
}
}  // namespace adaptive_controllers
