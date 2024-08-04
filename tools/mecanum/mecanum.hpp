#ifndef MECANUM_HPP
#define MECANUM_HPP

#include "tools/math_tools/math_tools.hpp"

namespace tools
{
class Mecanum
{
public:
  Mecanum(
    float wheel_radius, float half_length, float half_width, bool reverse_lf = false,
    bool reverse_lr = false, bool reverse_rf = true, bool reverse_rr = true);

  float speed_lf;  // rad/s, left front
  float speed_lr;  // rad/s, left rear
  float speed_rf;  // rad/s, right front
  float speed_rr;  // rad/s, right rear

  // +x: forward, +y: left, +z: up
  void calc(float vx, float vy, float wz);

private:
  const float r_;
  const float l_;
  const float w_;
  const float sign_lf_;
  const float sign_lr_;
  const float sign_rf_;
  const float sign_rr_;
};

}  // namespace tools

#endif  // MECANUM_HPP