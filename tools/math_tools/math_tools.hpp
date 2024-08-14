#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace tools
{
constexpr float PI = M_PI;

// (-PI, PI]
float limit_angle(float angle);

// [-max, max]
float limit_max(float input, float max);

int deadband_limit(int input, int deadline);

// [min, max]
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
uint32_t float_to_uint(float input, float min, float max, size_t bits);

}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP