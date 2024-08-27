#include "diff_gear.hpp"

namespace tools
{
DiffGear::DiffGear(bool reverse_l, bool reverse_r)
: sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

void DiffGear::calc(float v_pitch, float v_roll)
{
  this->v_left = sign_l_ * (v_pitch + v_roll);
  this->v_right = sign_r_ * (v_pitch - v_roll);
}

}  // namespace tools
