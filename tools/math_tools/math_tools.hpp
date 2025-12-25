#ifndef SP__MATH_TOOLS_HPP
#define SP__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace sp
{
constexpr float SP_PI = M_PI;

// (-PI, PI]
float limit_angle(float angle);
//inverse of limit_angle
float unwrap_angle(float raw);
// [min, max]
float limit_min_max(float input, float min, float max);

// [-max, max]
float limit_max(float input, float max);

// [min, max]
bool scope_min_max(float input, float min, float max);

// [-max, max]
bool scope_max(float input, float max);

// [min, max]
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
uint32_t float_to_uint(float input, float min, float max, size_t bits);

// sgn
int8_t sgn(float input);

// 三维向量数值微分：result = (v_next - v_prev) / dt
// 输入：v_next[3] 下一时刻向量，v_prev[3] 上一时刻向量
// 输出：result[3] 微分结果（速度/加速度等）
// dt: 时间间隔，默认 1ms
void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt = 1e-3f);

}  // namespace sp

#endif  // SP__MATH_TOOLS_HPP