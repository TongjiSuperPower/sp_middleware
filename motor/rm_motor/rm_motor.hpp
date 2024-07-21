#ifndef MOTOR__RM_MOTOR_HPP
#define MOTOR__RM_MOTOR_HPP

#include <cstdint>

namespace motor
{
class RM_Motor
{
public:
  RM_Motor(uint8_t motor_id);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data) const;

  float angle() const;
  float speed() const;
  int16_t current_raw() const;

  void cmd_raw(int16_t raw);

protected:
  uint8_t motor_id_;

private:
  bool has_read_;
  uint32_t last_read_ms_;

  int32_t circle_;
  uint16_t angle_ecd_;
  int16_t speed_rpm_;
  int16_t current_raw_;

  int16_t cmd_raw_;
};

}  // namespace motor

#endif  // MOTOR__RM_MOTOR_HPP