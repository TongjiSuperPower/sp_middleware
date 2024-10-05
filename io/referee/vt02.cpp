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
    case REFEREE_MOUSE_AND_KEYS_CMD_ID:
      update_mouse_and_keys();
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
  this->custom.button = (buff_[REFEREE_DATA_START + 14] == 1);
}

void VT02::update_mouse_and_keys()
{
  // 数据解析
  int16_t mouse_vx = (buff_[REFEREE_DATA_START + 0] << 8) | buff_[REFEREE_DATA_START + 1];
  int16_t mouse_vy = (buff_[REFEREE_DATA_START + 2] << 8) | buff_[REFEREE_DATA_START + 3];
  int16_t mouse_vs = (buff_[REFEREE_DATA_START + 4] << 8) | buff_[REFEREE_DATA_START + 5];
  uint16_t keyboard_value = (buff_[REFEREE_DATA_START + 9] << 8) | buff_[REFEREE_DATA_START + 8];

  // 更新公有属性
  this->mouse.vx = mouse_vx;
  this->mouse.vy = mouse_vy;
  this->mouse.vs = mouse_vs;
  this->mouse.left = (buff_[REFEREE_DATA_START + 6] == 1);
  this->mouse.right = (buff_[REFEREE_DATA_START + 7] == 1);

  // 更新公有属性
  this->keys.w = ((keyboard_value & 0x0001) != 0);
  this->keys.s = ((keyboard_value & 0x0002) != 0);
  this->keys.a = ((keyboard_value & 0x0004) != 0);
  this->keys.d = ((keyboard_value & 0x0008) != 0);
  this->keys.shift = ((keyboard_value & 0x0010) != 0);
  this->keys.ctrl = ((keyboard_value & 0x0020) != 0);
  this->keys.q = ((keyboard_value & 0x0040) != 0);
  this->keys.e = ((keyboard_value & 0x0080) != 0);
  this->keys.r = ((keyboard_value & 0x0100) != 0);
  this->keys.f = ((keyboard_value & 0x0200) != 0);
  this->keys.g = ((keyboard_value & 0x0400) != 0);
  this->keys.z = ((keyboard_value & 0x0800) != 0);
  this->keys.x = ((keyboard_value & 0x1000) != 0);
  this->keys.c = ((keyboard_value & 0x2000) != 0);
  this->keys.v = ((keyboard_value & 0x4000) != 0);
  this->keys.b = ((keyboard_value & 0x8000) != 0);
}

}  // namespace io
