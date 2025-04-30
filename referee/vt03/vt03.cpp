#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
VT03::VT03(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

void VT03::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_.data(), sizeof(buff_));
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), sizeof(buff_));
  }
}

void VT03::update(uint16_t size)
{
  if (buff_[0] == 0xA9 && buff_[1] == 0x53) {
    if (size != sizeof(VT03RemoteData)) return;
    if (!sp::check_crc16(buff_.data(), sizeof(VT03RemoteData))) return;
    std::copy(buff_.begin(), buff_.end(), reinterpret_cast<uint8_t *>(&remote_data_));
    update_remote();
  }
}

void VT03::update_remote()
{
  this->ch_rh = (remote_data_.ch_0 - 1024) / 660.0f;
  this->ch_rv = (remote_data_.ch_1 - 1024) / 660.0f;
  this->ch_lv = (remote_data_.ch_2 - 1024) / 660.0f;
  this->ch_lh = (remote_data_.ch_3 - 1024) / 660.0f;
  this->ch_lu = (remote_data_.wheel - 1024) / 660.0f;

  this->fn_l = remote_data_.fn_1;
  this->fn_r = remote_data_.fn_2;
  this->pause = remote_data_.pause;
  this->trigger = remote_data_.trigger;

  this->mode = (remote_data_.mode_sw == 2)   ? VT03Mode::S
               : (remote_data_.mode_sw == 1) ? VT03Mode::N
                                             : VT03Mode::C;
}

}  // namespace sp
