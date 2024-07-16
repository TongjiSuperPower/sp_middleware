#include "dbus.hpp"

namespace io
{
void Dbus::init(
  UART_HandleTypeDef * huart, DMA_HandleTypeDef * hdma_usart_rx, uint16_t dma_buf_size)
{
  huart_ = huart;
  hdma_usart_rx_ = hdma_usart_rx;
  SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);
  __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
  __HAL_DMA_DISABLE(hdma_usart_rx);
  while (hdma_usart_rx_->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdma_usart_rx);
  }
  hdma_usart_rx_->Instance->PAR = reinterpret_cast<uint32_t>(&huart_->Instance->DR);
  hdma_usart_rx_->Instance->M0AR = reinterpret_cast<uint32_t>(rx1_buf_);
  hdma_usart_rx->Instance->M1AR = reinterpret_cast<uint32_t>(rx2_buf_);
  hdma_usart_rx_->Instance->NDTR = dma_buf_size;
  SET_BIT(hdma_usart_rx_->Instance->CR, DMA_SxCR_DBM);
  __HAL_DMA_ENABLE(hdma_usart_rx_);
}

void Dbus::sbus_to_rc(volatile const uint8_t * sbus_buf)
{
  if (sbus_buf == NULL) {
    return;
  }

  rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;         //!< Channel 0
  rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;  //!< Channel 1
  rc.ch[2] =
    ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;  //!< Channel 2
  rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;               //!< Channel 3
  rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                               //NULL
  rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                                     //!< Switch left
  rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                //!< Switch right

  mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);    //!< Mouse X axis
  mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);    //!< Mouse Y axis
  mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  //!< Mouse Z axis
  mouse.press_l = sbus_buf[12];                  //!< Mouse Left Is Press l
  mouse.press_r = sbus_buf[13];                  //!< Mouse Right Is Press r
  key.v = sbus_buf[14] | (sbus_buf[15] << 8);    // KeyBoard value

  rc.ch[0] -= RC_CH_VALUE_OFFSET_;
  rc.ch[1] -= RC_CH_VALUE_OFFSET_;
  rc.ch[2] -= RC_CH_VALUE_OFFSET_;
  rc.ch[3] -= RC_CH_VALUE_OFFSET_;
  rc.ch[4] -= RC_CH_VALUE_OFFSET_;
}
}  // namespace io