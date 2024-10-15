#include "math_tools.hpp"

namespace tools
{
float limit_angle(float angle)
{
  while (angle > PI) angle -= 2 * PI;
  while (angle <= -PI) angle += 2 * PI;
  return angle;
}

float limit_min_max(float input, float min, float max)
{
  if (input > max)
    input = max;
  else if (input < min)
    input = min;
  return input;
}

float limit_max(float input, float max) { return limit_min_max(input, -max, max); }

float uint_to_float(uint32_t input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = static_cast<float>(input) / ((1 << bits) - 1);
  return norm * span + min;
}

uint32_t float_to_uint(float input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = (input - min) / span;
  return norm * ((1 << bits) - 1);
}

float inv_sqrt(float x)
{
  float x_half = 0.5f * x;
  int32_t i = *(int32_t *)&x;       // 用整数表示法取得x的表示
  i = 0x5f3759df - (i >> 1);        // 估算
  x = *(float *)&i;                 // 转回浮点数
  x = x * (1.5f - x_half * x * x);  // 牛顿迭代法
  return x;
}

float sqrt(float x) { return x * inv_sqrt(x); }

}  // namespace tools
