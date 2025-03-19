#include "diff_drive.hpp"

namespace sp
{
DiffDrive::DiffDrive(float half_width, float wheel_radius, bool reverse_l, bool reverse_r)
: w_(half_width), r_(wheel_radius), sign_l_(reverse_l ? -1 : 1), sign_r_(reverse_r ? -1 : 1)
{
}

void DiffDrive::calc(float v_set, float w_set)
{
  this->speed_l = sign_l_ * (v_set + w_set * w_) / r_;
  this->speed_r = sign_r_ * (v_set - w_set * w_) / r_;
}

void DiffDrive::update(float motor_speed_l, float motor_speed_r)
{
  this->v = (sign_l_ * motor_speed_l + sign_r_ * motor_speed_r) * r_ / 2;
  this->w = (sign_l_ * motor_speed_l - sign_r_ * motor_speed_r) * r_ / (2 * w_);
}

}  // namespace sp
