#ifndef MOTOR__GM6020_HPP
#define MOTOR__GM6020_HPP

#include <cstdint>

namespace motor
{

class GM6020
{
public:
  GM6020(uint8_t motor_id, bool voltage_ctrl);

  // TODO
  // bool is_open() const;

  uint16_t rx_id() const;
  void read(uint8_t * data);

  uint16_t tx_id() const;
  void write(uint8_t * data) const;

  float angle() const;
  float speed() const;
  float torque() const;

  // (voltage_ctrl) ? speed : torque
  void cmd(float speed_or_torque);

private:
  uint8_t motor_id_;
  bool voltage_ctrl_;
  int16_t cmd_raw_;

  uint16_t angle_ecd_;
  int16_t speed_rpm_;
  int16_t current_raw_;
  uint8_t temperature_;
};

}  // namespace motor

#endif  // MOTOR__GM6020_HPP