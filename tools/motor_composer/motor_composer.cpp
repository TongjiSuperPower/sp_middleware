#include "motor_composer.hpp"

namespace sp
{
MotorComposer::MotorComposer(bool reverse_l, bool reverse_r)
: sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

void MotorComposer::init_angles(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

void MotorComposer::calc_angle(float angle_l, float angle_r)
{
  this->angle = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
}

void MotorComposer::calc_speed(float speed_l, float speed_r)
{
  this->speed = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
}

void MotorComposer::calc_torque(float torque_l, float torque_r)
{
  this->torque = sign_l_ * torque_l + sign_r_ * torque_r;
}

void MotorComposer::calc_speeds(float speed)
{
  this->speed_l = sign_l_ * speed;
  this->speed_r = sign_r_ * speed;
}

}  // namespace sp
