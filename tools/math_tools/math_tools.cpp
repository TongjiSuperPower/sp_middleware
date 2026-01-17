#include "math_tools.hpp"

namespace sp
{
float limit_angle(float angle)
{
  while (angle > SP_PI) angle -= 2 * SP_PI;
  while (angle <= -SP_PI) angle += 2 * SP_PI;
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

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (fabsf(in_max - in_min) < 1e-6f) {
    return out_min;
  }
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

int8_t sgn(float input) { return (input > 0) - (input < 0); }

}  // namespace sp
