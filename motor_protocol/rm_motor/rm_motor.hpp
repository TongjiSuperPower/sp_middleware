#ifndef _RM_MOTOR_HPP_
#define _RM_MOTOR_HPP_

#include "cstdint"
#include "main.h"

namespace motor_protocol
{
class RM_Motor
{
private:
  // 电机个数
  static const uint8_t RM_MOTOR_NUM = 1;
  // 电机数据
  struct motor_measure_t
  {
    uint16_t ecd;           // 电机编码器位置
    int16_t speed_rpm;      // 电机转速（转/分钟）
    int16_t given_current;  // 电机实际电流
    uint8_t temperate;      // 电机温度
  } motor_measure_[RM_MOTOR_NUM];
  // 电机CAN发送数据
  CAN_TxHeaderTypeDef motor_tx_message_;
  uint8_t motor_can_send_data_[8];

public:
  RM_Motor() {}
  ~RM_Motor() {}
  // 发送电机控制指令
  HAL_StatusTypeDef motor_cmd(
    CAN_HandleTypeDef * hcan, uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3,
    int16_t motor4);
  // 解码电机数据
  void decode_motor_measure(uint8_t motor_id, uint8_t data[8]);
};

}  // namespace motor_protocol

#endif  // _RM_MOTOR_HPP_