#ifndef MOTOR__M2006_HPP
#define MOTOR__M2006_HPP

#include "rm_motor.hpp"

namespace motor
{
constexpr float M2006_P36 = 36;
constexpr int16_t M2006_MAX_CURRENT_RAW = 10000;
constexpr float M2006_RAW_TO_TORQUE = 0.18 / M2006_P36 * 10 / M2006_MAX_CURRENT_RAW;

class M2006 : public RM_Motor
{
public:
  M2006(uint8_t motor_id) : RM_Motor(motor_id) {}

  uint16_t rx_id() const { return 0x200 + motor_id_; }
  uint16_t tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

  float angle() const { return RM_Motor::angle() / M2006_P36; }
  float speed() const { return RM_Motor::speed() / M2006_P36; }
  float torque() const { return current_raw() * M2006_RAW_TO_TORQUE * M2006_P36; }

  void cmd(float torque)
  {
    auto raw = static_cast<int16_t>(torque / M2006_P36 / M2006_RAW_TO_TORQUE);
    if (raw > M2006_MAX_CURRENT_RAW) raw = M2006_MAX_CURRENT_RAW;
    if (raw < -M2006_MAX_CURRENT_RAW) raw = -M2006_MAX_CURRENT_RAW;
    cmd_raw(raw);
  }
};

}  // namespace motor

#endif  // MOTOR__M2006_HPP