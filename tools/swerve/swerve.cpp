#include "swerve.hpp"

#include <cmath>

namespace tools
{
Swerve::Swerve(float wheel_radius, float half_length, float half_width)
: r_(wheel_radius), l_(half_length), w_(half_width)
{
  this->speed_lf = 0.0f;
  this->speed_lr = 0.0f;
  this->speed_rf = 0.0f;
  this->speed_rr = 0.0f;

  this->angle_lf = 0.0f;  // left-front转向角度初始化为0弧度
  this->angle_lr = 0.0f;  // left-rear转向角度初始化为0弧度
  this->angle_rf = 0.0f;  // right-front转向角度初始化为0弧度
  this->angle_rr = 0.0f;  // right-rear转向角度初始化为0弧度
}

void Swerve::calc(float vx, float vy, float wz)
{
  float para_xy = vx * vx + vy * vy + (l_ * wz) * (l_ * wz) + (w_ * wz) * (w_ * wz);
  float para_x = 2 * w_ * wz * vx;
  float para_y = 2 * l_ * wz * vy;

  this->speed_rf = -std::sqrt(para_xy - para_x + para_y) / r_;
  this->speed_lf = -std::sqrt(para_xy - para_x - para_y) / r_;
  this->speed_lr = -std::sqrt(para_xy + para_x - para_y) / r_;
  this->speed_rr = -std::sqrt(para_xy + para_x + para_y) / r_;

  this->angle_rf = std::atan2(vy + l_ * wz, vx - w_ * wz);
  this->angle_lf = std::atan2(vy + l_ * wz, vx + w_ * wz);
  this->angle_lr = std::atan2(vy - l_ * wz, vx + w_ * wz);
  this->angle_rr = std::atan2(vy - l_ * wz, vx - w_ * wz);
}
}  // namespace tools