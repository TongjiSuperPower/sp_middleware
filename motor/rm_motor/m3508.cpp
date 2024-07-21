#include "m3508.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
constexpr int16_t M3508_MAX_CURRENT_RAW = 16384;
constexpr float RAW_TO_TORQUE = 0.3 / (M3508_P19) * 20 / M3508_MAX_CURRENT_RAW;

M3508::M3508(uint8_t motor_id, float ratio) : motor_id_(motor_id), ratio_(ratio), cmd_raw_(0) {}

uint16_t M3508::rx_id() const { return 0x200 + motor_id_; }

uint16_t M3508::tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

void M3508::read(uint8_t * data)
{
  // TODO timestamp
  angle_ecd_ = static_cast<uint16_t>((data[0] << 8) | data[1]);
  speed_rpm_ = static_cast<int16_t>((data[2] << 8) | data[3]);
  current_raw_ = static_cast<int16_t>((data[4] << 8) | data[5]);
  temperature_ = data[6];
}

void M3508::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = static_cast<uint8_t>(cmd_raw_ >> 8);
  data[(motor_id_ - 1) % 4 * 2 + 1] = static_cast<uint8_t>(cmd_raw_);
}

float M3508::angle() const
{
  // TODO multicicrle
  return (static_cast<float>(angle_ecd_) / 8192 - 0.5) * 2 * tools::PI;
}

float M3508::speed() const { return static_cast<float>(speed_rpm_) / 60 * 2 * tools::PI / ratio_; }

float M3508::torque() const { return static_cast<float>(current_raw_) * RAW_TO_TORQUE * ratio_; }

void M3508::cmd(float torque)
{
  auto raw = static_cast<int16_t>(torque * ratio_ / RAW_TO_TORQUE);

  if (raw < M3508_MAX_CURRENT_RAW && raw > -M3508_MAX_CURRENT_RAW)
    cmd_raw_ = raw;
  else if (raw > 0)
    cmd_raw_ = M3508_MAX_CURRENT_RAW;
  else
    cmd_raw_ = -M3508_MAX_CURRENT_RAW;
}

}  // namespace motor
