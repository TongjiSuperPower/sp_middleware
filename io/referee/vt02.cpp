#include "vt02.hpp"

#include "referee_def.h"
#include "tools/crc/crc.hpp"

namespace io
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
  if (!tools::check_crc8(buff_, REFEREE_HEAD_LEN)) return;

  size_t data_len = (buff_[2] << 8) | buff_[1];
  size_t len = REFEREE_HEAD_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN;
  if (!tools::check_crc16(buff_, len)) return;

  uint16_t cmd_id = (buff_[6] << 8) | buff_[5];
  switch (cmd_id) {
    case REFEREE_CUSTOM_CMD_ID:
      update_custom();
      break;
    case REFEREE_KEYBOARD_AND_MOUSE_CMD_ID:
      update_keyboard_and_mouse();
      break;
    default:
      break;
  }
}

void VT02::update_custom()
{
  // 数据解析
  int16_t yaw_int = (buff_[REFEREE_DATA_START + 0] << 8) | buff_[REFEREE_DATA_START + 1];
  int16_t roll_int = (buff_[REFEREE_DATA_START + 2] << 8) | buff_[REFEREE_DATA_START + 3];
  int16_t pitch_int = (buff_[REFEREE_DATA_START + 4] << 8) | buff_[REFEREE_DATA_START + 5];
  int16_t roll2_int = (buff_[REFEREE_DATA_START + 6] << 8) | buff_[REFEREE_DATA_START + 7];
  int16_t x_int = (buff_[REFEREE_DATA_START + 8] << 8) | buff_[REFEREE_DATA_START + 9];
  int16_t y_int = (buff_[REFEREE_DATA_START + 10] << 8) | buff_[REFEREE_DATA_START + 11];
  int16_t z_int = (buff_[REFEREE_DATA_START + 12] << 8) | buff_[REFEREE_DATA_START + 13];

  // 更新公有属性
  this->custom.yaw = float(yaw_int) / 1e3f;
  this->custom.roll = float(roll_int) / 1e3f;
  this->custom.pitch = float(pitch_int) / 1e3f;
  this->custom.roll2 = float(roll2_int) / 1e3f;
  this->custom.x = float(x_int) / 1e4f;
  this->custom.y = float(y_int) / 1e4f;
  this->custom.z = float(z_int) / 1e4f;
  this->custom.buttom = (buff_[REFEREE_DATA_START + 14] == 1);
}

void VT02::update_keyboard_and_mouse()
{
  // TODO
}

}  // namespace io
