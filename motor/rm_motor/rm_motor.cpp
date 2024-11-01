#include "rm_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
constexpr int16_t M2006_MAX_RAW = 10000;
constexpr int16_t M3508_MAX_RAW = 16384;
constexpr int16_t GM6020_MAX_RAW = 16384;
constexpr int16_t GM6020_V_MAX_RAW = 25000;

constexpr float M2006_RAW_TO_TORQUE = 0.18f / M2006_P36 * 10 / M2006_MAX_RAW;
constexpr float M3508_RAW_TO_TORQUE = 0.3f / M3508_P19 * 20 / M3508_MAX_RAW;
constexpr float GM6020_RAW_TO_TORQUE = 0.741f * 3 / GM6020_MAX_RAW;

constexpr float M2006_CMD_TO_RAW = 1.0f / M2006_RAW_TO_TORQUE;
constexpr float M3508_CMD_TO_RAW = 1.0f / M3508_RAW_TO_TORQUE;
constexpr float GM6020_CMD_TO_RAW = 1.0f / GM6020_RAW_TO_TORQUE;
constexpr float GM6020_V_CMD_TO_RAW = GM6020_V_MAX_RAW / 24.0f;

uint16_t get_rx_id(uint8_t motor_id, RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
    case RM_Motors::M3508:
      return motor_id + 0x200;
    default:
      return motor_id + 0x204;
  }
}

uint16_t get_tx_id(uint8_t motor_id, RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::GM6020:
      return motor_id < 5 ? 0x1FE : 0x2FE;
    case RM_Motors::GM6020_V:
      return motor_id < 5 ? 0x1FF : 0x2FF;
    default:
      return motor_id < 5 ? 0x200 : 0x1FF;
  }
}

float get_raw_to_torque(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_RAW_TO_TORQUE;
    case RM_Motors::M3508:
      return M3508_RAW_TO_TORQUE;
    default:
      return GM6020_RAW_TO_TORQUE;
  }
}

float get_cmd_to_raw(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_CMD_TO_RAW;
    case RM_Motors::M3508:
      return M3508_CMD_TO_RAW;
    case RM_Motors::GM6020:
      return GM6020_CMD_TO_RAW;
    default:
      return GM6020_V_CMD_TO_RAW;
  }
}

int16_t get_max_raw(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_MAX_RAW;
    case RM_Motors::M3508:
      return M3508_MAX_RAW;
    case RM_Motors::GM6020:
      return GM6020_MAX_RAW;
    default:
      return GM6020_V_MAX_RAW;
  }
}

RM_Motor::RM_Motor(uint8_t motor_id, RM_Motors motor_type, float ratio, bool multi_circle)
: rx_id(get_rx_id(motor_id, motor_type)),
  tx_id(get_tx_id(motor_id, motor_type)),
  angle(0),
  speed(0),
  torque(0),
  temperature(0),
  motor_id_(motor_id),
  motor_type_(motor_type),
  ratio_(ratio),
  multi_circle_(multi_circle),
  has_read_(false),
  circle_(0),
  cmd_raw_(0)
{
}

bool RM_Motor::is_open() const { return has_read_; }

bool RM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

void RM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  // 更新时间戳
  last_read_ms_ = stamp_ms;

  // 数据解析
  uint16_t angle_ecd = (data[0] << 8) | data[1];
  int16_t speed_rpm = (data[2] << 8) | data[3];
  int16_t current_raw = (data[4] << 8) | data[5];

  // 首次读取时, 初始化last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = angle_ecd;
  }

  // 多圈编码
  if (angle_ecd - last_ecd_ > 4096)
    circle_--;
  else if (angle_ecd - last_ecd_ < -4096)
    circle_++;
  last_ecd_ = angle_ecd;

  float angle_rad = float(angle_ecd - 4095) / 8192 * 2 * PI / ratio_;

  // 更新公有属性
  this->angle = multi_circle_ ? angle_rad + circle_ * 2 * PI / ratio_ : angle_rad;
  this->speed = float(speed_rpm) / 60 * 2 * PI / ratio_;
  this->torque = float(current_raw) * get_raw_to_torque(motor_type_) * ratio_;
  this->temperature = data[6];
}

void RM_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_ >> 8;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_;
}

void RM_Motor::cmd(float value)
{
  auto raw = value / ratio_ * get_cmd_to_raw(motor_type_);
  auto max_raw = get_max_raw(motor_type_);

  if (raw > max_raw) raw = max_raw;
  if (raw < -max_raw) raw = -max_raw;

  // 更新私有属性cmd_raw_, 待write()使用
  cmd_raw_ = raw;
}

}  // namespace sp
