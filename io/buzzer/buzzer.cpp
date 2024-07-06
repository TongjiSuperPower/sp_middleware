#include "buzzer.hpp"

namespace io
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

void Buzzer::set(float hz, float duty_cycle)
{
  arr_ = static_cast<uint16_t>(clock_hz_ / (htim_->Instance->PSC + 1) / hz - 1);
  __HAL_TIM_SET_AUTORELOAD(htim_, arr_);

  ccr_ = static_cast<uint16_t>((htim_->Instance->ARR + 1) * duty_cycle);
  __HAL_TIM_SET_COMPARE(htim_, channel_, ccr_);

  __HAL_TIM_SET_COUNTER(htim_, 0);
}

}  // namespace io
