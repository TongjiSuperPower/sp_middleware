#include "swerve.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
Swerve::Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot)
: r_(wheel_radius), l_(half_length), w_(half_width), sign_(invert_pivot ? -1.0f : 1.0f)
{
}

void Swerve::init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  yaw_offset_lf_ = yaw_lf;
  yaw_offset_lr_ = yaw_lr;
  yaw_offset_rf_ = yaw_rf;
  yaw_offset_rr_ = yaw_rr;
}

void Swerve::calc(
  float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  // 计算中间变量: 各轮速度向量
  float v_lf[2] = {vx - wz * w_, vy + wz * l_};
  float v_lr[2] = {vx - wz * w_, vy - wz * l_};
  float v_rf[2] = {vx + wz * w_, vy + wz * l_};
  float v_rr[2] = {vx + wz * w_, vy - wz * l_};

  // 计算各舵转角和各舵角度
  convert(v_lf, yaw_lf, yaw_offset_lf_, this->angle_lf, this->speed_lf);
  convert(v_lr, yaw_lr, yaw_offset_lr_, this->angle_lr, this->speed_lr);
  convert(v_rf, yaw_rf, yaw_offset_rf_, this->angle_rf, this->speed_rf);
  convert(v_rr, yaw_rr, yaw_offset_rr_, this->angle_rr, this->speed_rr);
}

void Swerve::convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed)
{
  // ref: https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/src/swerve.cpp

  if (v[0] == 0.0f && v[1] == 0.0f) {
    angle = yaw;
    speed = 0.0f;
    return;
  }

  float v_angle = std::atan2(v[1], v[0]);
  float v_angle_flipped = limit_angle(v_angle + PI);
  float pivot_angle = sign_ * limit_angle(yaw - yaw_offset);

  float a = limit_angle(v_angle - pivot_angle);
  float b = limit_angle(v_angle_flipped - pivot_angle);
  float pivot_angle_set = (std::abs(a) < std::abs(b)) ? v_angle : v_angle_flipped;

  angle = limit_angle(sign_ * pivot_angle_set + yaw_offset);
  speed = std::sqrt(v[0] * v[0] + v[1] * v[1]) / r_ * std::cos(a) ;//* std::cos(a) * std::cos(a);
}


void Swerve::update(
  float speed_lf, float speed_lr, float speed_rf, float speed_rr, float angle_lf, float angle_lr,
  float angle_rf, float angle_rr)
{
  // 角度修正
  float theta_lf = limit_angle(angle_lf - yaw_offset_lf_);
  float theta_lr = limit_angle(angle_lr - yaw_offset_lr_);
  float theta_rf = limit_angle(angle_rf - yaw_offset_rf_);
  float theta_rr = limit_angle(angle_rr - yaw_offset_rr_);

  // 各轮线速度分解（车体坐标系）
  this->v_lf_x = speed_lf * r_ * std::cos(theta_lf);
  this->v_lf_y = - speed_lf * r_ * std::sin(theta_lf);

  this->v_lr_x = speed_lr * r_ * std::cos(theta_lr);
  this->v_lr_y = - speed_lr * r_ * std::sin(theta_lr);

  this->v_rf_x = speed_rf * r_ * std::cos(theta_rf);
  this->v_rf_y = - speed_rf * r_ * std::sin(theta_rf);

  this->v_rr_x = speed_rr * r_ * std::cos(theta_rr);
  this->v_rr_y = - speed_rr * r_ * std::sin(theta_rr);

  // 位置向量
  const float pos[4][2] = {
    {+l_, +w_},  // LF
    {-l_, +w_},  // LR
    {+l_, -w_},  // RF
    {-l_, -w_},  // RR
  };

  // 平均线速度
  float vx_sum = (v_lf_x + v_lr_x + v_rf_x + v_rr_x) / 4.0f;
  float vy_sum = (v_lf_y + v_lr_y + v_rf_y + v_rr_y) / 4.0f;

  // 平均角速度
  this->wz_lf = (v_lf_x * pos[0][1] - v_lf_y * pos[0][0]) / (l_ * l_ + w_ * w_);
  this->wz_lr = (v_lr_x * pos[1][1] - v_lr_y * pos[1][0]) / (l_ * l_ + w_ * w_);
  this->wz_rf = (v_rf_x * pos[2][1] - v_rf_y * pos[2][0]) / (l_ * l_ + w_ * w_);
  this->wz_rr = (v_rr_x * pos[3][1] - v_rr_y * pos[3][0]) / (l_ * l_ + w_ * w_);
  float wz_sum = (wz_lf + wz_lr + wz_rf + wz_rr) / 4.0f;

  this->vx = vx_sum;
  this->vy = vy_sum;
  this->wz = wz_sum;
}

}  // namespace sp