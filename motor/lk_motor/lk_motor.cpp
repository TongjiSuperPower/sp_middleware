#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
constexpr float LK_MOTOR_MAX_RAW_TX = 2000;
constexpr float LK_MOTOR_MAX_RAW_RX = 2048;
constexpr float MG_RAW_TO_CURRENT_TX = 32.0f / LK_MOTOR_MAX_RAW_TX;
constexpr float MG_RAW_TO_CURRENT_RX = 33.0f / LK_MOTOR_MAX_RAW_RX;

LK_Motor::LK_Motor(uint8_t motor_id, float ratio, float torque_const)
: rx_id(0x140 + motor_id), 
  tx_id(0x280), 
  motor_id_(motor_id), 
  ratio_(ratio),
  torque_const_(torque_const),
  has_read_(false)
{
}

void LK_Motor::read(const uint8_t * data)
{
  int16_t torque_int = (data[3] << 8) | data[2];
  int16_t speed_int = (data[5] << 8) | data[4];
  uint16_t angle_int = (data[7] << 8) | data[6];

  // 首次读取时, 初始化last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = angle_int;
  }

  // 多圈编码
  if (angle_int - last_ecd_ > 65535/2)
    step_--;
  else if (angle_int - last_ecd_ < -65535/2)
    step_++;
  last_ecd_ = angle_int;

  float angle_rad = float(angle_int - 65535/2) / 65535 * 2 * PI / ratio_;


  this->temp = data[1];
  this->torque = torque_int * MG_RAW_TO_CURRENT_RX * torque_const_;
  this->speed = -speed_int / 180.0f * sp::PI / ratio_;
  // this->angle = float(angle_int / 65535.0f * 2 * sp::PI / ratio_);
  this->angle = -sp::limit_angle(angle_rad + step_ * 2 * PI / ratio_);

}

void LK_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_ >> 8;
}

void LK_Motor::cmd(float value)
{
  float raw = value / torque_const_ / MG_RAW_TO_CURRENT_TX;
  cmd_raw_ = sp::limit_max(raw, LK_MOTOR_MAX_RAW_TX);
}

}  // namespace sp
