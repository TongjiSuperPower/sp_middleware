#include "diff_gear.hpp"

namespace sp
{
DiffGear::DiffGear(float ratio, bool reverse_l, bool reverse_r)
: ratio_(ratio), sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

void DiffGear::init_gear_angle(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

void DiffGear::calc_end_angle(float angle_l, float angle_r)
{
  this->pitch = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
  this->roll = (sign_l_ * (angle_l - angle_l0_) - sign_r_ * (angle_r - angle_r0_)) / 2 / ratio_;
}

void DiffGear::calc_end_speed(float speed_l, float speed_r)
{
  this->v_pitch = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
  this->v_roll = (sign_l_ * speed_l - sign_r_ * speed_r) / 2 / ratio_;
}

void DiffGear::calc_end_torque(float torque_l, float torque_r)
{
  this->t_pitch = (sign_l_ * torque_l + sign_r_ * torque_r);
  this->t_roll = (sign_l_ * torque_l - sign_r_ * torque_r) * ratio_;
}

void DiffGear::calc_gear_speed(float v_pitch, float v_roll)
{
  this->v_left = sign_l_ * (v_pitch + v_roll * ratio_);
  this->v_right = sign_r_ * (v_pitch - v_roll * ratio_);
}

}  // namespace sp
