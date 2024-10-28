#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace sp
{
constexpr float PI = M_PI;

// (-PI, PI]
float limit_angle(float angle);

// [min, max]
float limit_min_max(float input, float min, float max);

// [-max, max]
float limit_max(float input, float max);

// [min, max]
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
uint32_t float_to_uint(float input, float min, float max, size_t bits);

}  // namespace sp

#endif  // TOOLS__MATH_TOOLS_HPP