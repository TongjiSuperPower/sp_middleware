#ifndef IO__BUZZER_HPP
#define IO__BUZZER_HPP

#include <cstdint>

#include "tim.h"

namespace io
{
class Buzzer
{
public:
  Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz);

  void start();
  void stop();
  void set(float hz, float duty_cycle = 0.5);

private:
  TIM_HandleTypeDef * htim_;
  uint16_t channel_;
  float clock_hz_;
  uint16_t arr_;
  uint16_t ccr_;
};

}  // namespace io

#endif  // IO__BUZZER_HPP