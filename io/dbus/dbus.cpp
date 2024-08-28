#include "dbus.hpp"

namespace io
{
float get_stick(uint16_t raw) { return (static_cast<int16_t>(raw) - 1024) / 660.0f; }

DBusSwitchMode get_switch(uint8_t raw)
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
  has_read_ = false;

  stick_rh = 0.0f;
  stick_rv = 0.0f;
  stick_lh = 0.0f;
  stick_lv = 0.0f;
  switch_r = DBusSwitchMode::DOWN;
  switch_l = DBusSwitchMode::DOWN;
}

void DBus::restart()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, DBUS_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  } else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, DBUS_BUFF_SIZE);
  }
}

void DBus::update(uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  // TODO deadzone limit

  stick_rh = get_stick((buff_[0] | (buff_[1] << 8)) & 0x07ff);
  stick_rv = get_stick(((buff_[1] >> 3) | (buff_[2] << 5)) & 0x07ff);
  stick_lh = get_stick(((buff_[2] >> 6) | (buff_[3] << 2) | (buff_[4] << 10)) & 0x07ff);
  stick_lv = get_stick(((buff_[4] >> 1) | (buff_[5] << 7)) & 0x07ff);

  switch_r = get_switch((buff_[5] >> 4) & 0x0003);
  switch_l = get_switch(((buff_[5] >> 4) & 0x000C) >> 2);

  // TODO mouse and keyboard

  restart();
}

bool DBus::is_open() const { return has_read_; }

bool DBus::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

}  // namespace io
