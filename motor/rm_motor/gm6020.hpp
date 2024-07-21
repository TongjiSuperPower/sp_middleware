#ifndef MOTOR__GM6020_HPP
#define MOTOR__GM6020_HPP

#include "rm_motor.hpp"

namespace motor
{
class GM6020 : public RM_Motor
{
public:
  GM6020(uint8_t motor_id, bool voltage_ctrl);

  uint16_t rx_id() const;
  uint16_t tx_id() const;

  float angle() const;
  float speed() const;
  float torque() const;

  void cmd(float speed_or_torque);

private:
  bool voltage_ctrl_;
};

}  // namespace motor

#endif  // MOTOR__GM6020_HPP