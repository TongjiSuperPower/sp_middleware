#ifndef SP__VT03_HPP
#define SP__VT03_HPP

#include "usart.h"

namespace sp
{
struct __attribute__((packed)) RemoteData
{
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left : 2;
  uint8_t mouse_right : 2;
  uint8_t mouse_middle : 2;
  uint16_t key;
  uint16_t crc16;
};

class VT03
{
public:
  VT03(UART_HandleTypeDef * huart, bool use_dma = true);
  UART_HandleTypeDef * huart;

  // TODO: remote, mouse and keys

  void request();
  void update(uint16_t size);

private:
  const bool use_dma_;
  RemoteData remote_data_;
};

}  // namespace sp

#endif  // SP__VT03_HPP