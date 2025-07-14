#include "math_tools.hpp"

namespace sp
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

bool scope_min_max(float input, float min, float max) { return (input >= min && input <= max); }

bool scope_max(float input, float max) { return scope_min_max(input, -max, max); }

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

int8_t sgn(float input) { return (input > 0) - (input < 0); }

}  // namespace sp
