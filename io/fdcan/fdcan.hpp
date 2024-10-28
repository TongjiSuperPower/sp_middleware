#ifndef SP__FDCAN_HPP
#define SP__FDCAN_HPP

#include "fdcan.h"

namespace sp
{
constexpr size_t FDCAN_DATA_LEN = 8;

class FDCAN
{
public:
  FDCAN(FDCAN_HandleTypeDef * hfdcan);

  uint16_t rx_id;
  uint8_t rx_data[FDCAN_DATA_LEN];
  uint8_t tx_data[FDCAN_DATA_LEN];

  void start();
  void recv();
  void send(uint16_t tx_id);

private:
  FDCAN_HandleTypeDef * hfdcan_;
  FDCAN_RxHeaderTypeDef rx_header_;
  FDCAN_TxHeaderTypeDef tx_header_;
};

}  // namespace sp

#endif  // SP__FDCAN_HPP