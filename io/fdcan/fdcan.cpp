#include "fdcan.hpp"

namespace sp
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

void FDCAN::start()
{
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
  HAL_FDCAN_ActivateNotification(
    hfdcan_, FDCAN_IT_BUS_OFF,
    0);  // dismiss return ref: https://community.st.com/t5/stm32-mcus-products/stm32h7-fdcan-has-lost-the-automatic-bus-off-recovery-mechanism/td-p/187400?utm_source=chatgpt.com
}

void FDCAN::recv()
{
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, this->rx_data);  // dismiss return
  this->rx_id = rx_header_.Identifier;
}

void FDCAN::send(uint16_t tx_id)
{
  tx_header_.Identifier = tx_id;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, this->tx_data);  // dismiss return
}

}  // namespace sp
