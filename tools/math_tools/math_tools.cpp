#include "math_tools.hpp"

namespace tools
{
float limit_angle(float angle)
{
  while (angle > PI) angle -= 2 * PI;
  while (angle <= -PI) angle += 2 * PI;
  return angle;
}

float limit_max(float input, float max)
{
  if (input > max)
    input = max;
  else if (input < -max)
    input = -max;
  return input;
}

int deadband_limit(int input, int deadline) { return std::fabs(input) < deadline ? 0 : input; }

}  // namespace tools
