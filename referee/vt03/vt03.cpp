#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
VT03::VT03(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

void VT03::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(
      this->huart, reinterpret_cast<uint8_t *>(&remote_data_), sizeof(remote_data_));
  }
  else {
    HAL_UART_Receive_IT(
      this->huart, reinterpret_cast<uint8_t *>(&remote_data_), sizeof(remote_data_));
  }
}

void VT03::update(uint16_t size)
{
  if (remote_data_.sof_1 == 0xA9) {
    if (size != sizeof(remote_data_)) return;
    if (remote_data_.sof_2 != 0x53) return;
    if (!sp::check_crc16(reinterpret_cast<uint8_t *>(&remote_data_), sizeof(remote_data_))) return;

    // TODO
  }
}

}  // namespace sp
