#include "buzzer.hpp"

namespace sp
{
Buzzer::Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz)
: htim_(htim), channel_(channel), clock_hz_(clock_hz)
{
}

void Buzzer::start()
{
  HAL_TIM_PWM_Start(htim_, channel_);  // dismiss return
}

void Buzzer::stop()
{
  HAL_TIM_PWM_Stop(htim_, channel_);  // dismiss return
}

void Buzzer::set(float hz, float duty)
{
  // ref: https://www.bilibili.com/video/BV1th411z7sn?t=1393.4&p=15
  __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6 - 1);  // 预分频到1MHz
  __HAL_TIM_SET_AUTORELOAD(htim_, clock_hz_ / (htim_->Instance->PSC + 1) / hz - 1);
  __HAL_TIM_SET_COMPARE(htim_, channel_, (htim_->Instance->ARR + 1) * duty);
  __HAL_TIM_SET_COUNTER(htim_, 0);
}

}  // namespace sp
