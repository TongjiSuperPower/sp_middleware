#ifndef MOTOR__RM_MOTOR_HPP
#define MOTOR__RM_MOTOR_HPP

#include <cstdint>

namespace motor
{
class RM_Motor
{
public:
  RM_Motor(uint8_t motor_id);

  void read(uint8_t * data);
  void write(uint8_t * data) const;

  float angle() const;
  float speed() const;
  int16_t current_raw() const;

  void cmd_raw(int16_t raw);

protected:
  uint8_t motor_id_;

private:
  uint16_t angle_ecd_;
  int16_t speed_rpm_;
  int16_t current_raw_;

  int16_t cmd_raw_;
};

}  // namespace motor

#endif  // MOTOR__RM_MOTOR_HPP