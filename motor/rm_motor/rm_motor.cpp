#include "rm_motor.hpp"

namespace motor
{
RM_Motor::RM_Motor(uint8_t motor_id)
: motor_id_(motor_id), has_read_(false), circle_(0), cmd_raw_(0)
{
}

bool RM_Motor::is_open() const { return has_read_; }

bool RM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

void RM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  last_read_ms_ = stamp_ms;
  auto last_ecd = angle_ecd_;

  angle_ecd_ = static_cast<uint16_t>((data[0] << 8) | data[1]);
  speed_rpm_ = static_cast<int16_t>((data[2] << 8) | data[3]);
  current_raw_ = static_cast<int16_t>((data[4] << 8) | data[5]);

  if (!has_read_) {
    has_read_ = true;
    return;
  }

  if (std::abs(angle_ecd_ - last_ecd) > 4096) circle_ += (speed_rpm_ > 0) ? 1 : -1;
}

void RM_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = static_cast<uint8_t>(cmd_raw_ >> 8);
  data[(motor_id_ - 1) % 4 * 2 + 1] = static_cast<uint8_t>(cmd_raw_);
}

float RM_Motor::angle() const
{
  return (static_cast<float>(angle_ecd_ - 4095) / 8192 + circle_) * 2 * tools::PI;
}

float RM_Motor::speed() const { return static_cast<float>(speed_rpm_) / 60 * 2 * tools::PI; }

int16_t RM_Motor::current_raw() const { return current_raw_; }

void RM_Motor::cmd_raw(int16_t raw) { cmd_raw_ = raw; }

// -------------------- GM6020 --------------------

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

float GM6020::torque() const { return current_raw() * GM6020_RAW_TO_TORQUE; }

void GM6020::cmd(float speed_or_torque)
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

// -------------------- M2006 --------------------

M2006::M2006(uint8_t motor_id) : RM_Motor(motor_id) {}

uint16_t M2006::rx_id() const { return 0x200 + motor_id_; }

uint16_t M2006::tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

float M2006::angle() const { return RM_Motor::angle() / M2006_P36; }

float M2006::speed() const { return RM_Motor::speed() / M2006_P36; }

float M2006::torque() const { return current_raw() * M2006_RAW_TO_TORQUE * M2006_P36; }

void M2006::cmd(float torque)
{
  auto raw = static_cast<int16_t>(torque / M2006_P36 / M2006_RAW_TO_TORQUE);
  if (raw > M2006_MAX_CURRENT_RAW) raw = M2006_MAX_CURRENT_RAW;
  if (raw < -M2006_MAX_CURRENT_RAW) raw = -M2006_MAX_CURRENT_RAW;
  cmd_raw(raw);
}

// -------------------- M3508 --------------------

M3508::M3508(uint8_t motor_id, float ratio) : RM_Motor(motor_id), ratio_(ratio) {}

uint16_t M3508::rx_id() const { return 0x200 + motor_id_; }

uint16_t M3508::tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

float M3508::angle() const { return RM_Motor::angle() / ratio_; }

float M3508::speed() const { return RM_Motor::speed() / ratio_; }

float M3508::torque() const { return current_raw() * M3508_RAW_TO_TORQUE * ratio_; }

void M3508::cmd(float torque)
{
  auto raw = static_cast<int16_t>(torque / ratio_ / M3508_RAW_TO_TORQUE);
  if (raw > M3508_MAX_CURRENT_RAW) raw = M3508_MAX_CURRENT_RAW;
  if (raw < -M3508_MAX_CURRENT_RAW) raw = -M3508_MAX_CURRENT_RAW;
  cmd_raw(raw);
}

}  // namespace motor
