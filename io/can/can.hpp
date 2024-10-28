#ifndef SP__CAN_HPP
#define SP__CAN_HPP

#include "can.h"

namespace sp
{
constexpr size_t CAN_DATA_LEN = 8;

class CAN
{
public:
  CAN(CAN_HandleTypeDef * hcan);

  uint32_t rx_id;                 // 只读! recv()的输出
  uint8_t rx_data[CAN_DATA_LEN];  // 只读! recv()的输出
  uint8_t tx_data[CAN_DATA_LEN];  // 只写! send()的输入

  // 使用C板官方示例的CAN过滤器配置
  // ref: https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/bsp/boards/bsp_can.c
  void config();

  // 手动配置CAN过滤器的过滤规则
  // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/can/bsp_can.c
  void config(const CAN_FilterTypeDef & filter);

  // 启动CAN通信
  void start();

  // 接收1帧标准数据帧或扩展数据帧
  // rx_fifo: 缓冲区位置, 默认为CAN_RX_FIFO0
  void recv(uint32_t rx_fifo = CAN_RX_FIFO0);

  // 发送1帧标准数据帧或扩展数据帧
  void send(uint32_t tx_id);

private:
  CAN_HandleTypeDef * hcan_;
};

}  // namespace sp

#endif  // SP__CAN_HPP
