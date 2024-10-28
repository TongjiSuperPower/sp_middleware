#include "led.hpp"

namespace sp
{
LED::LED(TIM_HandleTypeDef * htim) : htim_(htim) {}

void LED::start()
{
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);  // dismiss return
}

void LED::set(float r, float g, float b)
{
  __HAL_TIM_SET_AUTORELOAD(htim_, 65535);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, (htim_->Instance->ARR + 1) * b);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, (htim_->Instance->ARR + 1) * g);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, (htim_->Instance->ARR + 1) * r);
}

}  // namespace sp