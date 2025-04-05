#include "mecanum.hpp"

namespace sp
{
Mecanum::Mecanum(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr)
: r_(wheel_radius),
  l_(half_length),
  w_(half_width),
  sign_lf_((reverse_lf) ? -1.0f : 1.0f),
  sign_lr_((reverse_lr) ? -1.0f : 1.0f),
  sign_rf_((reverse_rf) ? -1.0f : 1.0f),
  sign_rr_((reverse_rr) ? -1.0f : 1.0f)
{
  this->speed_lf = 0.0f;
  this->speed_lr = 0.0f;
  this->speed_rf = 0.0f;
  this->speed_rr = 0.0f;
}

void Mecanum::calc(float vx, float vy, float wz)
{
  // ref: https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2
  this->speed_lf = sign_lf_ * (vx - vy - (l_ + w_) * wz) / r_;
  this->speed_lr = sign_lr_ * (vx + vy - (l_ + w_) * wz) / r_;
  this->speed_rf = sign_rf_ * (vx + vy + (l_ + w_) * wz) / r_;
  this->speed_rr = sign_rr_ * (vx - vy + (l_ + w_) * wz) / r_;
}

void Mecanum::update(float speed_lf, float speed_lr, float speed_rf, float speed_rr)
{
  float sign_speed_lf = speed_lf / sign_lf_;
  float sign_speed_lr = speed_lr / sign_lr_;
  float sign_speed_rf = speed_rf / sign_rf_;
  float sign_speed_rr = speed_rr / sign_rr_;

  this->vx = (sign_speed_lf + sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_;
  this->vy = (-sign_speed_lf + sign_speed_lr + sign_speed_rf - sign_speed_rr) / 4 * r_;
  this->wz = (-sign_speed_lf - sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_ / (l_ + w_);
}

}  // namespace sp
