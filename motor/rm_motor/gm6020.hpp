#ifndef MOTOR__GM6020_HPP
#define MOTOR__GM6020_HPP

#include "rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

namespace motor
{
constexpr int16_t GM6020_MAX_VOTAGE_RAW = 25000;
constexpr int16_t GM6020_MAX_CURRENT_RAW = 16384;
constexpr float GM6020_RAW_TO_SPEED = (13.33 / 60 * 2 * tools::PI) * 24 / GM6020_MAX_VOTAGE_RAW;
constexpr float GM6020_RAW_TO_TORQUE = 0.741 * 3 / GM6020_MAX_CURRENT_RAW;

class GM6020 : public RM_Motor
{
public:
  GM6020(uint8_t motor_id, bool voltage_ctrl = true)
  : RM_Motor(motor_id), voltage_ctrl_(voltage_ctrl)
  {
  }

  uint16_t rx_id() const { return 0x204 + motor_id_; }
  uint16_t tx_id() const
  {
    if (motor_id_ < 5)
      return (voltage_ctrl_) ? 0x1FF : 0x1FE;
    else
      return (voltage_ctrl_) ? 0x2FF : 0x2FE;
  }

  float torque() const { return current_raw() * GM6020_RAW_TO_TORQUE; }

  void cmd(float speed_or_torque)
  {
    int16_t raw;
    if (voltage_ctrl_) {
      raw = static_cast<int16_t>(speed_or_torque / GM6020_RAW_TO_SPEED);
      if (raw > GM6020_MAX_VOTAGE_RAW) raw = GM6020_MAX_VOTAGE_RAW;
      if (raw < -GM6020_MAX_VOTAGE_RAW) raw = -GM6020_MAX_VOTAGE_RAW;
    } else {
      raw = static_cast<int16_t>(speed_or_torque / GM6020_RAW_TO_TORQUE);
      if (raw > GM6020_MAX_CURRENT_RAW) raw = GM6020_MAX_CURRENT_RAW;
      if (raw < -GM6020_MAX_CURRENT_RAW) raw = -GM6020_MAX_CURRENT_RAW;
    }
    cmd_raw(raw);
  }

private:
  bool voltage_ctrl_;
};

}  // namespace motor

#endif  // MOTOR__GM6020_HPP