#ifndef MOTOR__M3508_HPP
#define MOTOR__M3508_HPP

#include "rm_motor.hpp"

namespace motor
{
constexpr float M3508_P19 = 3591 / 187;
constexpr int16_t M3508_MAX_CURRENT_RAW = 16384;
constexpr float M3508_RAW_TO_TORQUE = 0.3 / M3508_P19 * 20 / M3508_MAX_CURRENT_RAW;

class M3508 : public RM_Motor
{
public:
  M3508(uint8_t motor_id, float ratio = M3508_P19) : RM_Motor(motor_id), ratio_(ratio) {}

  uint16_t rx_id() const { return 0x200 + motor_id_; }
  uint16_t tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

  float angle() const { return RM_Motor::angle() / ratio_; }
  float speed() const { return RM_Motor::speed() / ratio_; }
  float torque() const { return current_raw() * M3508_RAW_TO_TORQUE * ratio_; }

  void cmd(float torque)
  {
    auto raw = static_cast<int16_t>(torque / ratio_ / M3508_RAW_TO_TORQUE);
    if (raw > M3508_MAX_CURRENT_RAW) raw = M3508_MAX_CURRENT_RAW;
    if (raw < -M3508_MAX_CURRENT_RAW) raw = -M3508_MAX_CURRENT_RAW;
    cmd_raw(raw);
  }

private:
  float ratio_;
};

}  // namespace motor

#endif  // MOTOR__M3508_HPP