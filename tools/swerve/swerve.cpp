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
  speed = std::sqrt(v[0] * v[0] + v[1] * v[1]) / r_ * std::cos(a);
}

}  // namespace sp