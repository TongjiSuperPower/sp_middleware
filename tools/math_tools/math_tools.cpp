#include "math_tools.hpp"

namespace tools
{
float limit_angle(float angle)
{
  while (angle > PI) angle -= 2 * PI;
  while (angle <= -PI) angle += 2 * PI;
  return angle;
}

int deadband_limit(int input, int deadline) { return fabs(input) < deadline ? 0 : input; }

}  // namespace tools
