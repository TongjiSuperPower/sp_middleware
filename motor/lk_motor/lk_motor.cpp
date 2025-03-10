#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
constexpr float LK_MOTOR_MAX_RAW = 2048;
constexpr float MF_RAW_TO_CURRENT = 16.5f / LK_MOTOR_MAX_RAW;

LK_Motor::LK_Motor(uint8_t motor_id, float torque_const)
: rx_id(0x140 + motor_id), tx_id(0x280), motor_id_(motor_id), torque_const_(torque_const)
{
}

void LK_Motor::read(const uint8_t * data)
{
  int16_t torque_int = (data[3] << 8) | data[2];
  int16_t speed_int = (data[5] << 8) | data[4];
  uint16_t angle_int = (data[7] << 8) | data[6];

  this->temp = data[1];
  this->torque = torque_int * MF_RAW_TO_CURRENT * torque_const_;  // TODO MG等系列
  this->speed = speed_int / 180.0f * sp::PI;
  this->angle = sp::limit_angle(angle_int / 65535.0f * 2 * sp::PI);  // TODO 编码器不同, 系数不同
}

void LK_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_ >> 8;
}

void LK_Motor::cmd(float value)
{
  float raw = value / torque_const_ / MF_RAW_TO_CURRENT;  // TODO MG等系列
  cmd_raw_ = sp::limit_max(raw, LK_MOTOR_MAX_RAW);
}

}  // namespace sp
