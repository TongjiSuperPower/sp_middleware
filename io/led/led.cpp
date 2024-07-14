#include "led.hpp"

namespace io
{

Led::Led(TIM_HandleTypeDef * htim) : htim_(htim)
{
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void Led::set(uint16_t red, uint16_t green, uint16_t blue)
{
  __HAL_TIM_SetCompare(htim_, TIM_CHANNEL_1, blue);
  __HAL_TIM_SetCompare(htim_, TIM_CHANNEL_2, green);
  __HAL_TIM_SetCompare(htim_, TIM_CHANNEL_3, red);
}
}  // namespace io