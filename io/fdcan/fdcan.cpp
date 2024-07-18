#include "fdcan.hpp"

namespace io
{
FDCAN::FDCAN(FDCAN_HandleTypeDef * hfdcan) : hfdcan_(hfdcan)
{
  for (size_t i = 0; i < TX_DATAS; i++) {
    tx_headers_[i].IdType = FDCAN_STANDARD_ID;
    tx_headers_[i].TxFrameType = FDCAN_DATA_FRAME;
    tx_headers_[i].DataLength = FDCAN_DLC_BYTES_8;  // TODO
    tx_headers_[i].ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_headers_[i].BitRateSwitch = FDCAN_BRS_OFF;
    tx_headers_[i].FDFormat = FDCAN_CLASSIC_CAN;
    tx_headers_[i].TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_headers_[i].MessageMarker = 0x00;
  }
}

void FDCAN::start()
{
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
}

void FDCAN::recv()
{
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, rx_data_);  // dismiss return
}

uint32_t FDCAN::rx_id() { return rx_header_.Identifier; }

uint8_t * FDCAN::rx_data() { return rx_data_; }

uint8_t * FDCAN::tx_data() { return tx_datas_[i_]; }

void FDCAN::send(uint32_t tx_id)
{
  tx_headers_[i_].Identifier = tx_id;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_headers_[i_], tx_datas_[i_]);  // dismiss return

  i_ += 1;
  if (i_ == TX_DATAS) i_ = 0;
}

}  // namespace io
