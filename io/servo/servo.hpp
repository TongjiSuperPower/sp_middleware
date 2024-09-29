#ifndef IO__SERVO_HPP
#define IO__SERVO_HPP

#include "tim.h"

namespace io
{
class Servo
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  // max_angle: 舵机角度范围, 例如90度, 180度等等
  Servo(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz, float max_angle);

  // 开启对应的PWM端口
  void start();

  // 设置旋转角度
  // angle: 取值[0, max_angle], 单位同max_angle
  void set(float angle);

private:
  TIM_HandleTypeDef * htim_;
  uint16_t channel_;
  const float clock_hz_;
  const float max_angle_;
};

}  // namespace io

#endif  // IO__SERVO_HPP