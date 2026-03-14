#include "vt02.hpp"

#include <cstring>

#include "referee_def.h"
#include "tools/crc/crc.hpp"

namespace sp
{
VT02::VT02(UART_HandleTypeDef * huart, bool use_dma) : huart_(huart), use_dma_(use_dma) {}

void VT02::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, VT02_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, VT02_BUFF_SIZE);
  }
}

void VT02::update()
{
  if (buff_[0] != REFEREE_SOF) return;
  if (!check_crc8(buff_, REFEREE_HEAD_LEN)) return;

  size_t data_len = (buff_[2] << 8) | buff_[1];
  size_t len = REFEREE_HEAD_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN;
  if (!check_crc16(buff_, len)) return;

  uint16_t cmd_id = (buff_[6] << 8) | buff_[5];
  if (cmd_id == REFEREE_CUSTOM_CMD_ID) {
    std::memcpy(&(this->custom), buff_ + REFEREE_DATA_START, sizeof(RefereeCustomData));
  }
}

}  // namespace sp
