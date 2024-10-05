#ifndef IO__VT02_HPP
#define IO__VT02_HPP

#include "usart.h"

namespace io
{
constexpr size_t VT02_BUFF_SIZE = 50;

struct CustomData
{
  float yaw;
  float roll;
  float pitch;
  float roll2;
  float x;
  float y;
  float z;
  bool buttom;
};

class VT02
{
public:
  VT02(UART_HandleTypeDef * huart, bool use_dma = true);

  CustomData custom;  // 只读!

  void request();
  void update();

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  uint8_t buff_[VT02_BUFF_SIZE];

  void update_custom();
  void update_keyboard_and_mouse();
};

}  // namespace io

#endif  // IO__VT02_HPP