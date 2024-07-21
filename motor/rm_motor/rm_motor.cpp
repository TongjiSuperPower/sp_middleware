#include "rm_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

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

}  // namespace motor
