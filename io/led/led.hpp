#ifndef IO__LED_HPP
#define IO__LED_HPP

#include "tim.h"

namespace io
{
class LED
{
public:
  LED(TIM_HandleTypeDef * htim);

  void start();

  // 取值范围: [0, 1]
  void set(float r, float g, float b);

private:
  TIM_HandleTypeDef * htim_;
};

}  // namespace io

#endif  // IO__LED_HPP