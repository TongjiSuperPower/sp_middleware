#include "gm6020.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
constexpr int16_t GM6020_MAX_VOTAGE_RAW = 25000;
constexpr int16_t GM6020_MAX_CURRENT_RAW = 16384;
constexpr float GM6020_RAW_TO_SPEED = (13.33 / 60 * 2 * tools::PI) * 24 / GM6020_MAX_VOTAGE_RAW;
constexpr float GM6020_RAW_TO_TORQUE = 0.741 * 3 / GM6020_MAX_CURRENT_RAW;

GM6020::GM6020(uint8_t motor_id, bool voltage_ctrl)
: RM_Motor(motor_id), voltage_ctrl_(voltage_ctrl)
{
}

uint16_t GM6020::rx_id() const { return 0x204 + motor_id_; }

uint16_t GM6020::tx_id() const
{
  if (motor_id_ < 5)
    return (voltage_ctrl_) ? 0x1FF : 0x1FE;
  else
    return (voltage_ctrl_) ? 0x2FF : 0x2FE;
}

float GM6020::angle() const
{
  // TODO multicicrle
  return static_cast<float>(angle_ecd_ - 4095) / 8192 * 2 * tools::PI;
}

float GM6020::speed() const { return static_cast<float>(speed_rpm_) / 60 * 2 * tools::PI; }

float GM6020::torque() const { return static_cast<float>(current_raw_ * GM6020_RAW_TO_TORQUE); }

void GM6020::cmd(float speed_or_torque)
{
  int16_t raw;

  if (voltage_ctrl_) {
    raw = static_cast<int16_t>(speed_or_torque / GM6020_RAW_TO_SPEED);
    if (raw > GM6020_MAX_VOTAGE_RAW) raw = GM6020_MAX_VOTAGE_RAW;
    if (raw < -GM6020_MAX_VOTAGE_RAW) raw = -GM6020_MAX_VOTAGE_RAW;
  }

  else {
    raw = static_cast<int16_t>(speed_or_torque / GM6020_RAW_TO_TORQUE);
    if (raw > GM6020_MAX_CURRENT_RAW) raw = GM6020_MAX_CURRENT_RAW;
    if (raw < -GM6020_MAX_CURRENT_RAW) raw = -GM6020_MAX_CURRENT_RAW;
  }

  cmd_raw(raw);
}

}  // namespace motor
