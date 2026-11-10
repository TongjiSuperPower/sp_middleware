#include "rotor_feedforward.hpp"

namespace sp {
RotorFeedforward::RotorFeedforward(float kv, float ks, float ka,
                                   float max_torque, float deadband)
    : kv_(kv), ks_(ks), ka_(ka), max_torque_(max_torque), deadband_(deadband) {}

float RotorFeedforward::calc(float target_speed, float target_acceleration) {
  // 死区里不输出前馈: 目标速度几乎为 0 时, 摩擦项的符号会在 0 附近来回翻,
  // 力矩跟着一跳一跳, 反而把静止的车抖起来。
  if (deadband_ > 0.0f && std::fabs(target_speed) < deadband_)
    return this->out = 0.0f;

  // ks 是库仑摩擦, 大小恒定、方向跟着速度走, 所以乘符号而不是乘速度:
  // 写成 ks * speed 的话低速时这一项趋近 0, 恰好丢掉了它最该起作用的区间。
  const float torque =
      kv_ * target_speed + ks_ * sgn(target_speed) + ka_ * target_acceleration;

  return this->out =
             max_torque_ > 0.0f ? limit_max(torque, max_torque_) : torque;
}

float RotorFeedforward::calc(float target_speed) {
  // 没有加速度信息, 就当加速度为 0: 只出摩擦项, 不猜惯量项。
  return calc(target_speed, 0.0f);
}

} // namespace sp
