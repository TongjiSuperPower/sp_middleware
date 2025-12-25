#include "math_tools.hpp"

namespace sp
{
float limit_angle(float angle)
{
  while (angle > SP_PI) angle -= 2 * SP_PI;
  while (angle <= -SP_PI) angle += 2 * SP_PI;
  return angle;
}

float unwrap_angle(float raw)
{
  static float last_raw = 0.0f;
  static float unwrapped = 0.0f;
  static bool inited = false;

  if (!inited) {
    last_raw = raw;
    unwrapped = raw;
    inited = true;
    return unwrapped;
  }

  float delta = raw - last_raw;

  if (delta > SP_PI)
    delta -= 2.0f * SP_PI;
  else if (delta < -SP_PI)
    delta += 2.0f * SP_PI;

  unwrapped += delta;
  last_raw = raw;
  return unwrapped;
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

void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt)
{
  // 数值微分：(v_next - v_prev) / dt
  float inv_dt = 1.0f / dt;
  result[0] = (v_next[0] - v_prev[0]) * inv_dt;
  result[1] = (v_next[1] - v_prev[1]) * inv_dt;
  result[2] = (v_next[2] - v_prev[2]) * inv_dt;
}

}  // namespace sp
