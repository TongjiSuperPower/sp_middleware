#ifndef _CAN_HPP_
#define _CAN_HPP_

#include "can.h"

namespace io
{
constexpr size_t DATA_LEN = 8;

class CAN
{
public:
  CAN(CAN_HandleTypeDef * hcan);

  void recv();
  void send(uint32_t tx_id);

  uint8_t rx_data_[DATA_LEN];
  uint8_t tx_data_[DATA_LEN];
  CAN_RxHeaderTypeDef rx_header_;

private:
  uint32_t send_mail_box_;
  CAN_HandleTypeDef * hcan_;
  CAN_TxHeaderTypeDef tx_header_;
};
}  // namespace io

#endif  // _CAN_HPP_
