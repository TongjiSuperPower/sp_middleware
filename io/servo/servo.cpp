#include "servo.hpp"

namespace sp
{
Servo::Servo(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz, float max_angle)
: htim_(htim), channel_(channel), clock_hz_(clock_hz), max_angle_(max_angle)
{
}

void Servo::start()
{
  HAL_TIM_PWM_Start(htim_, channel_);  // dismiss return
}

void Servo::set(float angle)
{
  __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6f - 1);                       // 预分频后为1MHz
  __HAL_TIM_SET_AUTORELOAD(htim_, 20000 - 1);                                 // 输出频率为50Hz
  __HAL_TIM_SET_COMPARE(htim_, channel_, (angle / max_angle_) * 2000 + 500);  // [500, 2500]
}

}  // namespace sp