#include "can.hpp"

namespace sp
{
CAN::CAN(CAN_HandleTypeDef * hcan) : hcan_(hcan) {}

void CAN::config()
{
  CAN_FilterTypeDef filter;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterBank = (hcan_ == &hcan1) ? 0 : 14;
  filter.SlaveStartFilterBank = 14;

  config(filter);
}

void CAN::config(const CAN_FilterTypeDef & filter)
{
  HAL_CAN_ConfigFilter(hcan_, &filter);  // dismiss return
}

void CAN::start()
{
  HAL_CAN_Start(hcan_);                                              // dismiss return
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);  // dismiss return
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);  // dismiss return
}

void CAN::recv(uint32_t rx_fifo)
{
  static CAN_RxHeaderTypeDef rx_header;

  HAL_CAN_GetRxMessage(hcan_, rx_fifo, &rx_header, rx_data);  // dismiss return

  // TODO 扩展数据帧
  rx_id = rx_header.StdId;
}

void CAN::send(uint32_t tx_id)
{
  static uint32_t mailbox;
  static CAN_TxHeaderTypeDef tx_header;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = CAN_DATA_LEN;

  // TODO 扩展数据帧
  tx_header.IDE = CAN_ID_STD;
  tx_header.StdId = tx_id;

  HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &mailbox);  // dismiss return
}

}  // namespace sp