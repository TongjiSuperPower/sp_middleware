#include "lk_motor.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
namespace
{

constexpr float RAD_TO_DEG = 180.0f / SP_PI;
constexpr float MAX_RAW_I32 = 2.0e9f;
constexpr float MAX_CURRENT_RAW = 2048.0f;

struct MotorParameters
{
  float ratio;
  float torque_const;
  float current_scale;
  float max_current;
  float encoder_range;
};

// 获取型号参数，电流单位A，转矩常数单位N·m/A
MotorParameters get_motor_parameters(LK_Motors motor_type)
{
  // CAN编码器满量程暂按65536，待实测确认
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return {1.0f, 0.81f, 33.0f / 4096.0f, 15.0f, 65536.0f};
    case LK_Motors::MF7025_28T:
      return {1.0f, 0.48f, 33.0f / 4096.0f, 15.0f, 65536.0f};
    case LK_Motors::MG4005:
    default:
      return {10.0f, 0.06f, 66.0f / 4096.0f, 8.0f, 65536.0f};
  }
}

}  // namespace

LK_Motor::LK_Motor(uint8_t motor_id, LK_Motors motor_type, bool multi_circle)
: rx_id(0x140 + motor_id),
  tx_id(0x140 + motor_id),
  motor_id_(motor_id),
  motor_type_(motor_type),
  multi_circle_(multi_circle)
{
  const auto parameters = get_motor_parameters(motor_type);

  ratio_ = parameters.ratio;
  torque_const_ = parameters.torque_const;
  current_scale_ = parameters.current_scale;
  max_current_ = parameters.max_current;
  encoder_range_ = parameters.encoder_range;

  max_raw_ =
    static_cast<int16_t>(limit_min_max(max_current_ / current_scale_, 0.0f, MAX_CURRENT_RAW));
}

float LK_Motor::ratio() const { return ratio_; }

uint8_t LK_Motor::motor_id() const { return motor_id_; }

LK_Motors LK_Motor::motor_type() const { return motor_type_; }

float LK_Motor::max_current() const { return max_current_; }

int16_t LK_Motor::max_raw() const { return max_raw_; }

bool LK_Motor::is_open() const { return has_read_; }

bool LK_Motor::is_alive(uint32_t now_ms) const
{
  return has_read_ && (now_ms - last_read_ms_ < 100);
}

void LK_Motor::read(const uint8_t * data, uint32_t stamp_ms)
{
  switch (data[0]) {
    case 0x9A:
    case 0x9B:
      read_state1(data);
      break;

    case 0x9C:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
      read_state2(data);
      break;

    case 0x80:
    case 0x81:
    case 0x88:
      break;

    default:
      return;
  }

  has_read_ = true;
  last_read_ms_ = stamp_ms;
}

void LK_Motor::read_state1(const uint8_t * data)
{
  const auto voltage_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
  const auto current_raw = static_cast<int16_t>((data[5] << 8) | data[4]);

  this->temperature = static_cast<int8_t>(data[1]);
  this->bus_voltage = voltage_raw * 0.01f;
  this->bus_current = current_raw * 0.01f;
  this->motor_state = data[6];
  this->error_state = data[7];
}

void LK_Motor::read_state2(const uint8_t * data)
{
  const auto current_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
  const auto speed_raw = static_cast<int16_t>((data[5] << 8) | data[4]);
  const auto encoder = static_cast<uint16_t>((data[7] << 8) | data[6]);

  if (!encoder_inited_) {
    encoder_inited_ = true;
    last_encoder_ = encoder;
  }

  // 编码器跨过零点时更新圈数
  const int32_t delta = static_cast<int32_t>(encoder) - last_encoder_;

  if (delta > static_cast<int32_t>(encoder_range_ / 2.0f))
    circle_--;
  else if (delta < -static_cast<int32_t>(encoder_range_ / 2.0f))
    circle_++;

  last_encoder_ = encoder;

  const float motor_angle = (encoder / encoder_range_ + static_cast<float>(circle_)) * 2.0f * SP_PI;

  this->temperature = static_cast<int8_t>(data[1]);
  this->encoder_raw = encoder;

  if (encoder > this->encoder_raw_max) this->encoder_raw_max = encoder;

  this->current = current_raw * current_scale_;
  this->torque = this->current * torque_const_ * ratio_;
  this->speed = speed_raw / RAD_TO_DEG / ratio_;
  this->multicycle_angle = motor_angle / ratio_;
  this->angle = multi_circle_ ? this->multicycle_angle : limit_angle(this->multicycle_angle);
}

void LK_Motor::cmd_torque(float torque)
{
  // 无效目标按零力矩处理
  if (!std::isfinite(torque)) torque = 0.0f;

  const float raw = torque / ratio_ / torque_const_ / current_scale_;
  cmd_torque_raw_ = static_cast<int16_t>(limit_max(raw, max_raw_));
}

void LK_Motor::write_torque(uint8_t * data) const
{
  const auto raw = static_cast<uint16_t>(cmd_torque_raw_);

  data[0] = 0xA1;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = raw & 0xFF;
  data[5] = (raw >> 8) & 0xFF;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::cmd_velocity(float speed)
{
  // 无效目标按零速度处理
  if (!std::isfinite(speed)) speed = 0.0f;

  const float raw = speed * ratio_ * RAD_TO_DEG * 100.0f;
  cmd_velocity_raw_ = static_cast<int32_t>(limit_max(raw, MAX_RAW_I32));
}

void LK_Motor::write_velocity(uint8_t * data) const
{
  const auto raw = static_cast<uint32_t>(cmd_velocity_raw_);
  const auto current_limit = static_cast<uint16_t>(max_raw_);

  data[0] = 0xA2;
  data[1] = 0x00;
  data[2] = current_limit & 0xFF;
  data[3] = (current_limit >> 8) & 0xFF;
  data[4] = raw & 0xFF;
  data[5] = (raw >> 8) & 0xFF;
  data[6] = (raw >> 16) & 0xFF;
  data[7] = (raw >> 24) & 0xFF;
}

void LK_Motor::cmd_position(float position, uint8_t direction)
{
  // 无效位置或方向不更新目标
  if (!std::isfinite(position) || direction > 1) return;

  float angle = std::fmod(position, 2.0f * SP_PI);
  if (angle < 0.0f) angle += 2.0f * SP_PI;

  cmd_position_raw_ = static_cast<uint32_t>(angle * RAD_TO_DEG * 100.0f);

  // 避免浮点舍入得到36000
  if (cmd_position_raw_ >= 36000) cmd_position_raw_ = 0;

  cmd_direction_ = direction;
}

void LK_Motor::write_position(uint8_t * data) const
{
  const auto raw = cmd_position_raw_;

  data[0] = 0xA5;
  data[1] = cmd_direction_;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = raw & 0xFF;
  data[5] = (raw >> 8) & 0xFF;
  data[6] = (raw >> 16) & 0xFF;
  data[7] = (raw >> 24) & 0xFF;
}

void LK_Motor::cmd_angle_increment(float angle)
{
  // 无效目标按零增量处理
  if (!std::isfinite(angle)) angle = 0.0f;

  const float raw = angle * ratio_ * RAD_TO_DEG * 100.0f;
  cmd_angle_increment_raw_ = static_cast<int32_t>(limit_max(raw, MAX_RAW_I32));
}

void LK_Motor::write_angle_increment(uint8_t * data) const
{
  const auto raw = static_cast<uint32_t>(cmd_angle_increment_raw_);

  data[0] = 0xA7;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = raw & 0xFF;
  data[5] = (raw >> 8) & 0xFF;
  data[6] = (raw >> 16) & 0xFF;
  data[7] = (raw >> 24) & 0xFF;
}

void LK_Motor::cmd_angle_increment_speed(float angle, float max_speed)
{
  // 无效参数按零增量、零速度处理
  if (!std::isfinite(angle) || !std::isfinite(max_speed)) {
    angle = 0.0f;
    max_speed = 0.0f;
  }

  const float angle_raw = angle * ratio_ * RAD_TO_DEG * 100.0f;
  const float speed_raw = max_speed * ratio_ * RAD_TO_DEG;

  cmd_angle_increment_speed_raw_ = static_cast<int32_t>(limit_max(angle_raw, MAX_RAW_I32));
  cmd_max_speed_raw_ = static_cast<uint16_t>(limit_min_max(speed_raw, 0.0f, 65535.0f));
}

void LK_Motor::write_angle_increment_speed(uint8_t * data) const
{
  const auto angle_raw = static_cast<uint32_t>(cmd_angle_increment_speed_raw_);
  const auto speed_raw = cmd_max_speed_raw_;

  // 速度限制按协议帧表占用两字节
  data[0] = 0xA8;
  data[1] = 0x00;
  data[2] = speed_raw & 0xFF;
  data[3] = (speed_raw >> 8) & 0xFF;
  data[4] = angle_raw & 0xFF;
  data[5] = (angle_raw >> 8) & 0xFF;
  data[6] = (angle_raw >> 16) & 0xFF;
  data[7] = (angle_raw >> 24) & 0xFF;
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

void LK_Motor::write_enable(uint8_t * data) const
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

void LK_Motor::write_disable(uint8_t * data) const
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

}  // namespace sp
