#ifndef TOOLS__SWERVE_HPP
#define TOOLS__SWERVE_HPP

#include "tools/math_tools/math_tools.hpp"

namespace tools
{
class Swerve
{
public:
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  Swerve(float wheel_radius, float half_length, float half_width);

  float speed_lf;  // 只读! calc()计算结果, left-front转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, left-rear转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, right-front转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, right-rear转速, 单位: rad/s

  float angle_lf;  // 只读! calc()计算结果, left-front转向角度, 单位: rad
  float angle_lr;  // 只读! calc()计算结果, left-rear转向角度, 单位: rad
  float angle_rf;  // 只读! calc()计算结果, right-front转向角度, 单位: rad
  float angle_rr;  // 只读! calc()计算结果, right-rear转向角度, 单位: rad

  // 底盘速度 -> 各轮转速和各舵转角
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s
  void calc(float vx, float vy, float wz);

private:
  const float r_;
  const float l_;
  const float w_;
};

}  // namespace tools

#endif  // TOOLS__SWERVE_HPP