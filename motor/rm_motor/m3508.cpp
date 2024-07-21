#include "m3508.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
constexpr int16_t M3508_MAX_CURRENT_RAW = 16384;
constexpr float M3508_RAW_TO_TORQUE = 0.3 / (M3508_P19) * 20 / M3508_MAX_CURRENT_RAW;

M3508::M3508(uint8_t motor_id, float ratio) : RM_Motor(motor_id), ratio_(ratio) {}

uint16_t M3508::rx_id() const { return 0x200 + motor_id_; }

uint16_t M3508::tx_id() const { return (motor_id_ < 5) ? 0x200 : 0x1FF; }

float M3508::angle() const
{
  // TODO multicicrle
  return static_cast<float>(angle_ecd_ - 4095) / 8192 * 2 * tools::PI;
}

float M3508::speed() const { return static_cast<float>(speed_rpm_) / 60 * 2 * tools::PI / ratio_; }

float M3508::torque() const
{
  return static_cast<float>(current_raw_) * M3508_RAW_TO_TORQUE * ratio_;
}

void M3508::cmd(float torque)
{
  auto raw = static_cast<int16_t>(torque * ratio_ / M3508_RAW_TO_TORQUE);

  if (raw > M3508_MAX_CURRENT_RAW) raw = M3508_MAX_CURRENT_RAW;
  if (raw < -M3508_MAX_CURRENT_RAW) raw = -M3508_MAX_CURRENT_RAW;

  cmd_raw(raw);
}

}  // namespace motor
