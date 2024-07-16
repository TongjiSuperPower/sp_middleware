#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <cmath>

namespace tools
{
constexpr float PI = M_PI;

// (-PI, PI]
float limit_angle(float angle);

}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP