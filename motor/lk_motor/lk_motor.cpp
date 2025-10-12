#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{


LK_Motor::LK_Motor(uint8_t motor_id, float ratio, float torque_const)
: rx_id(0x140 + motor_id), 
  tx_id(0x140 + motor_id), 
  motor_id_(motor_id),
  ratio_(ratio),
  torque_const_(torque_const)
{
}

void LK_Motor::write_state1(uint8_t * data) const
{
  data[0] = 0x9A;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::read_state1(const uint8_t * data)
{
  if (data[0] != 0x9A && data[0] != 0x9B) return;

  this->temp = data[1];
  this->motorState = data[6];
  this->errorState = data[7];
}

void LK_Motor::write_clear_error(uint8_t * data) const
{
  data[0] = 0x9B;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_state2(uint8_t * data) const
{
  data[0] = 0x9C;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::read_state2(const uint8_t * data)
{
  if (data[0] != 0xA7 && data[0] != 0x9C && data[0] != 0xA1) return;

  int16_t iq = (data[3] << 8) | data[2];
  int16_t speed = (data[5] << 8) | data[4];
  uint16_t encoder = (data[7] << 8) | data[6];

  // 首次读取时, 初始化last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = encoder;
  }

  // 多圈编码
  if (encoder - last_ecd_ > 65535/2)
    step_--;
  else if (encoder - last_ecd_ < -65535/2)
    step_++;
  last_ecd_ = encoder;

  float angle_rad = float(encoder - 65535/2) / 65535 * 2 * PI / ratio_;

  this->temp = data[1];
  this->torque = iq * ( 66.0f / 4096.0f ) * torque_const_ * ratio_;
  this->speed = speed / 180.0f * sp::PI / ratio_;
  this->angle = sp::limit_angle(angle_rad + step_ * 2 * PI / ratio_);
}

void LK_Motor::write_turn_off(uint8_t * data) const
{
  data[0] = 0x80;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_turn_on(uint8_t * data) const
{
  data[0] = 0x88;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_stop(uint8_t * data) const
{
  data[0] = 0x81;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_position(uint8_t * data) const
{
  data[0] = 0xA5;
  data[1] = spin_direction_;
  data[4] = cmd_position_;
  data[5] = cmd_position_ >> 8;
  data[6] = cmd_position_ >> 16;
  data[7] = cmd_position_ >> 24;
}

void LK_Motor::write_angle_increment(uint8_t * data) const
{
  data[0] = 0xA7;
  data[4] = cmd_angle_;
  data[5] = cmd_angle_ >> 8;
  data[6] = cmd_angle_ >> 16;
  data[7] = cmd_angle_ >> 24;
}

void LK_Motor::write_torque(uint8_t * data) const
{
  data[0] = 0xA1;
  data[4] = cmd_raw_;
  data[5] = cmd_raw_ >> 8;
}

void LK_Motor::cmd_position(int32_t position, uint8_t direction) 
{
  cmd_position_ = position * 100;
  spin_direction_ = direction;
}

void LK_Motor::cmd_angle(int32_t angle) { cmd_angle_ = angle * 1000; }


void LK_Motor::cmd_torque(float value)
{
  float raw = value / torque_const_ / (33.0f / 2048.f) / ratio_;  // TODO MG等系列
  cmd_raw_ = sp::limit_max(raw, 2048.0f);
}


}  // namespace sp
