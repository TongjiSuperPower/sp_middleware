#ifndef IO__FDCAN_HPP
#define IO__FDCAN_HPP

#include "fdcan.h"

namespace io
{
constexpr size_t DLC = 8;
constexpr size_t TX_DATAS = 10;

class FDCAN
{
public:
  FDCAN(FDCAN_HandleTypeDef * hfdcan);

  void start();

  void recv();
  uint32_t rx_id();
  uint8_t * rx_data();

  uint8_t * tx_data();
  void send(uint32_t tx_id);

private:
  FDCAN_HandleTypeDef * hfdcan_;

  FDCAN_RxHeaderTypeDef rx_header_;
  uint8_t rx_data_[DLC];

  FDCAN_TxHeaderTypeDef tx_headers_[TX_DATAS];
  uint8_t tx_datas_[TX_DATAS][DLC];
  size_t i_;
};

}  // namespace io

#endif  // IO__FDCAN_HPP