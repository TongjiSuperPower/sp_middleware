#include "gm6020.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
constexpr float RAW_TO_SPEED = (13.33 / 60 * 2 * tools::PI) * 24 / 25000;
constexpr float RAW_TO_TORQUE = 0.741 * 3 / 16384;

GM6020::GM6020(uint8_t motor_id, bool voltage_ctrl)
: motor_id_(motor_id), voltage_ctrl_(voltage_ctrl), cmd_speed_or_torque_(0)
{
}

uint16_t GM6020::rx_id() const { return 0x204 + motor_id_; }

void GM6020::read(uint8_t * data)
{
  // TODO timestamp
  angle_ecd_ = static_cast<uint16_t>(data[0] << 8 | static_cast<uint16_t>(data[1]));
  speed_rpm_ = static_cast<int16_t>(data[2] << 8 | static_cast<uint16_t>(data[3]));
  current_raw_ = static_cast<int16_t>(data[4] << 8 | static_cast<uint16_t>(data[5]));
  temperature_ = data[6];
}

uint16_t GM6020::tx_id() const
{
  if (motor_id_ < 5) return (voltage_ctrl_) ? 0x1FF : 0x1FE;
  return (voltage_ctrl_) ? 0x2FF : 0x2FE;
}

void GM6020::write(uint8_t * data) const
{
  auto raw =
    static_cast<int16_t>(cmd_speed_or_torque_ / ((voltage_ctrl_) ? RAW_TO_SPEED : RAW_TO_TORQUE));
  data[(motor_id_ - 1) % 4 * 2 + 0] = static_cast<uint8_t>(raw >> 8);
  data[(motor_id_ - 1) % 4 * 2 + 1] = static_cast<uint8_t>(raw);
}

float GM6020::angle() const
{
  // TODO multicicrle
  return (static_cast<float>(angle_ecd_) / 8192 - 0.5) * 2 * tools::PI;
}

float GM6020::speed() const { return static_cast<float>(speed_rpm_) / 60 * 2 * tools::PI; }

float GM6020::torque() const { return static_cast<float>(current_raw_ * RAW_TO_TORQUE); }

void GM6020::cmd(float speed_or_torque) { cmd_speed_or_torque_ = speed_or_torque; }

}  // namespace motor
