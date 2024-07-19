#ifndef IO__FDCAN_HPP
#define IO__FDCAN_HPP

#include "fdcan.h"

namespace io
{
constexpr size_t DATA_LEN = 8;

class FDCAN
{
public:
  FDCAN(FDCAN_HandleTypeDef * hfdcan);

  uint32_t rx_id() const;
  uint8_t * rx_data();
  uint8_t * tx_data();

  void start();
  void recv();
  void send(uint32_t tx_id);

private:
  FDCAN_HandleTypeDef * hfdcan_;
  FDCAN_RxHeaderTypeDef rx_header_;
  FDCAN_TxHeaderTypeDef tx_header_;
  uint8_t rx_data_[DATA_LEN];
  uint8_t tx_data_[DATA_LEN];
};

}  // namespace io

#endif  // IO__FDCAN_HPP