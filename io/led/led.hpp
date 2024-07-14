#ifndef IO__LED_HPP_
#define IO__LED_HPP_

#include "tim.h"

namespace io
{
class Led
{
public:
  Led(TIM_HandleTypeDef * htim);
  void set(uint16_t red, uint16_t green, uint16_t blue);

private:
  TIM_HandleTypeDef * htim_;
};

}  // namespace io

#endif  // IO__LED_HPP_