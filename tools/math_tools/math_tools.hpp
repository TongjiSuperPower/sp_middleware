#ifndef SP__MATH_TOOLS_HPP
#define SP__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace sp
{
constexpr float SP_PI = M_PI;

// (-PI, PI]
float limit_angle(float angle);

/**
 * @brief 角度展开类（消除 ±π 跳变）
 * @note  每个需要展开的角度应使用独立的实例
 *
 * 使用示例：
 * @code
 *   sp::AngleUnwrapper yaw_unwrapper;
 *   sp::AngleUnwrapper pitch_unwrapper;
 *   float yaw_unwrapped = yaw_unwrapper.update(yaw_raw);
 *   float pitch_unwrapped = pitch_unwrapper.update(pitch_raw);
 * @endcode
 */
class AngleUnwrapper
{
public:
  AngleUnwrapper() = default;

  /**
   * @brief 更新角度并返回展开后的值
   * @param raw 原始角度（应在 -π 到 π 范围内）
   * @return 展开后的角度（累积值，无跳变）
   */
  float update(float raw);

  /**
   * @brief 重置状态
   */
  void reset();

  float out = 0.0f;  ///< 展开后的角度输出

private:
  float last_raw_ = 0.0f;
  bool inited_ = false;
};

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