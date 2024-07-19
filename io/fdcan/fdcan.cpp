#include "fdcan.hpp"

namespace io
{
FDCAN::FDCAN(FDCAN_HandleTypeDef * hfdcan) : hfdcan_(hfdcan)
{
  tx_header_.IdType = FDCAN_STANDARD_ID;
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
  tx_header_.DataLength = FDCAN_DLC_BYTES_8;
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header_.MessageMarker = 0x00;
}

uint32_t FDCAN::rx_id() const { return rx_header_.Identifier; }

uint8_t * FDCAN::rx_data() { return rx_data_; }

uint8_t * FDCAN::tx_data() { return tx_data_; }

void FDCAN::start()
{
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
}

void FDCAN::recv()
{
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, rx_data_);  // dismiss return
}

void FDCAN::send(uint32_t tx_id)
{
  tx_header_.Identifier = tx_id;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, tx_data_);  // dismiss return
}

}  // namespace io
