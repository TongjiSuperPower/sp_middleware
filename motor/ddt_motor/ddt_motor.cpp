#include "ddt_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
constexpr float DDT_MOTOR_MAX_RAW = 16383.0f;
constexpr float DDT_MOTOR_RAW_TO_CURRENT = 55.0f / 32767.0f;

DDT_Motor::DDT_Motor(uint8_t motor_id, float torque_const)
: rx_id(0x96 + motor_id),
  tx_id((motor_id < 5) ? 0x32 : 0x33),
  motor_id_(motor_id),
  torque_const_(torque_const)
{
}

void DDT_Motor::read(const uint8_t * data)
{
  int16_t speed_int = (data[0] << 8) | data[1];
  int16_t current_int = (data[2] << 8) | data[3];
  uint16_t angle_int = (data[4] << 8) | data[5];

  this->speed = speed_int * 0.1f * 2 * sp::PI / 60.0f;
  this->torque = current_int * DDT_MOTOR_RAW_TO_CURRENT * torque_const_;
  this->angle = sp::limit_angle(angle_int / 32767.0f * 2 * sp::PI);
  this->error = data[6];
  this->mode = data[7];
}

void DDT_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_ >> 8;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_;
}

void DDT_Motor::cmd(float value)
{
  float raw = value / torque_const_ / DDT_MOTOR_RAW_TO_CURRENT;
  cmd_raw_ = sp::limit_max(raw, DDT_MOTOR_MAX_RAW);
}

}  // namespace sp
