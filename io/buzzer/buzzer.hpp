#ifndef SP__BUZZER_HPP
#define SP__BUZZER_HPP

#include "tim.h"

namespace sp
{
class Buzzer
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz);

  void start();
  void stop();

  // hz: 发声频率, 单位: Hz
  // duty: 占空比, 取值范围: [0, 1], 0.5时声音最响
  void set(float hz, float duty = 0.5);

private:
  TIM_HandleTypeDef * htim_;
  uint16_t channel_;
  const float clock_hz_;
};

}  // namespace sp

#endif  // SP__BUZZER_HPP