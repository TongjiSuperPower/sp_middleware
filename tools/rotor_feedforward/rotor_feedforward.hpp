#ifndef SP__ROTOR_FEEDFORWARD_HPP
#define SP__ROTOR_FEEDFORWARD_HPP

#include "tools/math_tools/math_tools.hpp"

namespace sp {
class RotorFeedforward {
public:
  // ks: 静摩擦(库仑)力矩, N·m, 大小恒定、方向由速度符号给出
  // kv: 粘滞摩擦系数, N·m/(rad/s)
  // ka: 惯量项, N·m/(rad/s²), 也就是 J/Kt
  // max_torque: 输出限幅, N·m; <= 0 表示不限幅
  // deadband: 目标速度绝对值低于它就不输出前馈, rad/s; <= 0 表示不设死区
  RotorFeedforward(float kv, float ks, float ka = 0.0f, float max_torque = 0.0f,
                   float deadband = 0.0f);

  float out = 0; // 只读! 前馈力矩, N·m

  // 给了加速度就带惯量项; 不给就当没有加速度(ka 项为 0)。
  float calc(float target_speed, float target_acceleration);
  float calc(float target_speed);

private:
  const float kv_; // 旋转单位速度对应的扭矩
  const float ks_; // 克服静摩擦力对应的扭矩
  const float ka_; // 获得对应加速度对应的扭矩, 可以理解成转动惯量
  const float max_torque_; // 最大扭矩
  const float deadband_;   // 速度死区
};

} // namespace sp

#endif // SP__ROTOR_FEEDFORWARD_HPP
