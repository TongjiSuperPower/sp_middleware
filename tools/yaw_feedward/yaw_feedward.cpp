#include "yaw_feedward.hpp"

namespace sp
{
YawFeedward::YawFeedward(float dt, float inertia, float k_ff, float damping, float damping_out_max)
: dt_(dt), inertia_(inertia), k_ff_(k_ff), damping_(damping), damping_out_max_(damping_out_max)
{
}

void YawFeedward::calc(float speed_set)
{
  this->alpha_set_ = (speed_set - speed_filter.out) / dt_;

  this->intertia_out = k_ff_ * inertia_ * this->alpha_set_;
  
  this->damping_out = limit_max(speed_set * damping_, damping_out_max_);

  this->out = intertia_out + damping_out;

  speed_filter.update(speed_set);
}

}  // namespace sp