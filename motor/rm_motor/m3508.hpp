#ifndef MOTOR__M3508_HPP
#define MOTOR__M3508_HPP

#include "rm_motor.hpp"

namespace motor
{
constexpr float M3508_P19 = 3591 / 187;

class M3508 : public RM_Motor
{
public:
  M3508(uint8_t motor_id, float ratio = M3508_P19);

  uint16_t rx_id() const;
  uint16_t tx_id() const;

  float angle() const;
  float speed() const;
  float torque() const;

  void cmd(float torque);

private:
  float ratio_;
};

}  // namespace motor

#endif  // MOTOR__M3508_HPP