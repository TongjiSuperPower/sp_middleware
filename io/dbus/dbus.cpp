#include "dbus.hpp"

namespace sp
{
static float get_stick(uint16_t raw) { return (raw - 1024) / 660.0f; }

static DBusSwitchMode get_switch(uint8_t raw)
{
  if (raw == 1)
    return DBusSwitchMode::UP;
  else if (raw == 3)
    return DBusSwitchMode::MID;
  else
    return DBusSwitchMode::DOWN;
}

DBus::DBus(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), has_read_(false)
{
}

void DBus::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, DBUS_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, DBUS_BUFF_SIZE);
  }
}

void DBus::update(uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  // 遥控器解析

  this->stick_rh = get_stick((buff_[0] | (buff_[1] << 8)) & 0x07ff);
  this->stick_rv = get_stick(((buff_[1] >> 3) | (buff_[2] << 5)) & 0x07ff);
  this->stick_lh = get_stick(((buff_[2] >> 6) | (buff_[3] << 2) | (buff_[4] << 10)) & 0x07ff);
  this->stick_lv = get_stick(((buff_[4] >> 1) | (buff_[5] << 7)) & 0x07ff);
  this->stick_lu = get_stick(((buff_[16] | (buff_[17] << 8)) & 0x07ff));

  this->switch_r = get_switch((buff_[5] >> 4) & 0x0003);
  this->switch_l = get_switch(((buff_[5] >> 4) & 0x000C) >> 2);

  // 键鼠解析

  int16_t mouse_vx = (buff_[7] << 8) | buff_[6];
  int16_t mouse_vy = (buff_[9] << 8) | buff_[8];
  int16_t mouse_vs = (buff_[11] << 8) | buff_[10];
  uint16_t keyboard_value = (buff_[15] << 8) | buff_[14];

  this->mouse.vx = mouse_vx / 32768.0f;
  this->mouse.vy = mouse_vy / 32768.0f;
  this->mouse.vs = mouse_vs / 32768.0f;
  this->mouse.left = (buff_[12] == 1);
  this->mouse.right = (buff_[13] == 1);

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

bool DBus::is_open() const { return has_read_; }

bool DBus::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

}  // namespace sp
