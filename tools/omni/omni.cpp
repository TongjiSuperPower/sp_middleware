#include "omni.hpp"

#include <cmath>

namespace sp
{
Omni::Omni(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr)
: r_(wheel_radius),
  l_(half_length),
  w_(half_width),
  d_(std::sqrt(half_length * half_length + half_width * half_width)),
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

void Omni::calc(float vx, float vy, float wz)
{
  // 各轮滚动方向上的线速度: u · (v + wz × p) = (-y * vx + x * vy) / D + D * wz, 单位: m/s
  // 注意 x = l_ 处为车头方向, y = w_ 处为车身左侧
  const float v_lf = (-w_ * vx + l_ * vy) / d_ + d_ * wz;
  const float v_lr = (-w_ * vx - l_ * vy) / d_ + d_ * wz;
  const float v_rf = (+w_ * vx + l_ * vy) / d_ + d_ * wz;
  const float v_rr = (+w_ * vx - l_ * vy) / d_ + d_ * wz;

  // 线速度 / 半径 = 转速, 再按电机装配方向取反
  this->speed_lf = sign_lf_ * v_lf / r_;
  this->speed_lr = sign_lr_ * v_lr / r_;
  this->speed_rf = sign_rf_ * v_rf / r_;
  this->speed_rr = sign_rr_ * v_rr / r_;
}

void Omni::update(float speed_lf, float speed_lr, float speed_rf, float speed_rr)
{
  // 电机转速 -> 各轮滚动方向上的线速度, 单位: m/s
  const float v_lf = speed_lf / sign_lf_ * r_;
  const float v_lr = speed_lr / sign_lr_ * r_;
  const float v_rf = speed_rf / sign_rf_ * r_;
  const float v_rr = speed_rr / sign_rr_ * r_;

  // 最小二乘反解 (H^T H 为对角阵, 故为精确解)
  this->vx = (-v_lf - v_lr + v_rf + v_rr) / 4 * d_ / w_;
  this->vy = (+v_lf - v_lr + v_rf - v_rr) / 4 * d_ / l_;
  this->wz = (+v_lf + v_lr + v_rf + v_rr) / 4 / d_;
}

}  // namespace sp
