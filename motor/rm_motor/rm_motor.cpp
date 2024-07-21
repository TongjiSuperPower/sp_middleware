#include "rm_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
RM_Motor::RM_Motor(uint8_t motor_id) : motor_id_(motor_id), cmd_raw_(0) {}

void RM_Motor::read(uint8_t * data)
{
  // TODO timestamp
  angle_ecd_ = static_cast<uint16_t>((data[0] << 8) | data[1]);
  speed_rpm_ = static_cast<int16_t>((data[2] << 8) | data[3]);
  current_raw_ = static_cast<int16_t>((data[4] << 8) | data[5]);
  temperature_ = data[6];
}

void RM_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = static_cast<uint8_t>(cmd_raw_ >> 8);
  data[(motor_id_ - 1) % 4 * 2 + 1] = static_cast<uint8_t>(cmd_raw_);
}

void RM_Motor::cmd_raw(int16_t raw) { cmd_raw_ = raw; }

}  // namespace motor
