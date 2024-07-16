#ifndef IO__DBUS_HPP_
#define IO__DBUS_HPP_

#include "dma.h"
#include "usart.h"

namespace io
{
class Dbus
{
public:
  void init(UART_HandleTypeDef * huart, DMA_HandleTypeDef * hdma_usart_rx, uint16_t dma_buf_size);
  void sbus_to_rc(volatile const uint8_t * sbus_buf);
  struct remote_control_t
  {
    int16_t ch[5];
    int8_t s[2];
  } rc;
  struct mouse_t
  {
    int16_t x, y, z;
    uint8_t press_l, press_r;
  } mouse;
  struct key_t
  {
    uint16_t v;
  } key;
  uint8_t rx1_buf_[36], rx2_buf_[36];

private:
  constexpr static uint16_t RC_CH_VALUE_OFFSET_ = 1024;
  UART_HandleTypeDef * huart_;
  DMA_HandleTypeDef * hdma_usart_rx_;
};

}  // namespace io

#endif  // IO__DBUS_HPP_