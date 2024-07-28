#include "can.hpp"

namespace io
{
CAN::CAN(CAN_HandleTypeDef * hcan) : hcan_(hcan)
{
  tx_header_.IDE = CAN_ID_STD;
  tx_header_.RTR = CAN_RTR_DATA;
  tx_header_.DLC = 0x08;
}

void CAN::recv() { HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header_, rx_data_); }

void CAN::send(uint32_t id)
{
  tx_header_.StdId = id;
  HAL_CAN_AddTxMessage(hcan_, &tx_header_, tx_data_, &send_mail_box_);
}

}  // namespace io

extern "C" void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  can_filter_st.SlaveStartFilterBank = 14;
  can_filter_st.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}