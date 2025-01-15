#include "yaw_feedward.hpp"

namespace sp
{
YawFeedward::YawFeedward(float dt, float inertia, float k_ff)
: dt_(dt), inertia_(inertia), k_ff_(k_ff)
{
}

void YawFeedward::calc(float speed_set)
{
  this->alpha_set_ = (speed_set - speed_filter.out) / dt_;

  this->out = k_ff_ * inertia_ * this->alpha_set_;

  speed_filter.update(speed_set);
}

}  // namespace sp