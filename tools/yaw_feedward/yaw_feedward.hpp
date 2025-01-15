#ifndef SP__YAW_FEEDWARD_HPP
#define SP__YAW_FEEDWARD_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"

namespace sp
{
class YawFeedward
{
private:
  const float dt_;       // 控制周期，单位s
  const float inertia_;  // 转动惯量，单位kg·m^2
  const float k_ff_;     // 前馈增益，单位N·m/(rad/s)
  float alpha_set_;      // 目标角加速度，单位rad/s

  sp::LowPassFilter speed_filter{0.1f};  // 速度低通滤波器

public:
  float out = 0;  // 输出力矩，单位N·m（只读）
  YawFeedward(float dt, float inertia, float k_ff = 1.0f);
  void calc(float speed_set);
};

}  // namespace sp

#endif  // YAW_FEEDWARD_HPP