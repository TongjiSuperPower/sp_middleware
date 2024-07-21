#ifndef MOTOR__M3508_HPP
#define MOTOR__M3508_HPP

#include <cstdint>

namespace motor
{
constexpr float M3508_P19 = 3591 / 187;

class M3508
{
public:
  M3508(uint8_t motor_id, float ratio = M3508_P19);

  // TODO
  // bool is_open() const;

  uint16_t rx_id() const;
  uint16_t tx_id() const;

  void read(uint8_t * data);
  void write(uint8_t * data) const;

  float angle() const;
  float speed() const;
  float torque() const;

  void cmd(float torque);

private:
  uint8_t motor_id_;
  float ratio_;

  uint16_t angle_ecd_;
  int16_t speed_rpm_;
  int16_t current_raw_;
  uint8_t temperature_;

  int16_t cmd_raw_;
};

}  // namespace motor

#endif  // MOTOR__M3508_HPP