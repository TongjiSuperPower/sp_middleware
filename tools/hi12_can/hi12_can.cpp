#include "hi12_can.hpp"

#include <cstring>

#include "fdcan.h"
#include "hi12_sdk/hipnuc_j1939.h"

namespace sp
{

bool HI12Can::prepare()
{
  FDCAN_FilterTypeDef filter = {};
  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter.FilterID1 = 0x0CFF0008;
  filter.FilterID2 = 0x1FFF00FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) return false;
  if (
    HAL_FDCAN_ConfigGlobalFilter(
      &hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
    HAL_OK)
    return false;
  return HAL_FDCAN_ActivateNotification(
           &hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) == HAL_OK;
}

void HI12Can::read(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t stamp)
{
  if (
    header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
    header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8)
    return;

  unsigned group;
  switch (header.Identifier) {
    case 0x0CFF4608:
      group = 0;
      break;
    case 0x0CFF4108:
      group = 1;
      break;
    default:
      return;
  }

  hipnuc_can_frame_t frame = {};
  frame.id = header.Identifier;
  frame.is_extended = 1;
  frame.len = 8;
  std::memcpy(frame.data, data, 8);
  hipnuc_sample_t sample;
  if (hipnuc_j1939_parse(&frame, &sample, nullptr) <= 0) return;
  switch (group) {
    case 0:
      for (unsigned i = 0; i < 4; ++i) quat[i] = sample.quat[i];
      break;
    case 1:
      yaw = sample.yaw;
      break;
  }
  stamp_ms[group] = stamp;
}
}  // namespace sp
