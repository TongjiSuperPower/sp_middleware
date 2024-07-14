#include "rm_motor.hpp"

namespace motor_protocol
{
// 发送电机控制指令
HAL_StatusTypeDef RM_Motor::motor_cmd(
  CAN_HandleTypeDef * hcan, uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3,
  int16_t motor4)
{
  uint32_t send_mail_box;
  motor_tx_message_.StdId = stdid;
  motor_tx_message_.IDE = CAN_ID_STD;
  motor_tx_message_.RTR = CAN_RTR_DATA;
  motor_tx_message_.DLC = 0x08;

  motor_can_send_data_[0] = motor1 >> 8;
  motor_can_send_data_[1] = motor1;
  motor_can_send_data_[2] = motor2 >> 8;
  motor_can_send_data_[3] = motor2;
  motor_can_send_data_[4] = motor3 >> 8;
  motor_can_send_data_[5] = motor3;
  motor_can_send_data_[6] = motor4 >> 8;
  motor_can_send_data_[7] = motor4;

  return HAL_CAN_AddTxMessage(hcan, &motor_tx_message_, motor_can_send_data_, &send_mail_box);
}

// 解码电机数据
void RM_Motor::decode_motor_measure(uint8_t motor_id, uint8_t data[8])
{
  motor_measure_[motor_id].ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
  motor_measure_[motor_id].speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);
  motor_measure_[motor_id].given_current = (int16_t)((data)[4] << 8 | (data)[5]);
  motor_measure_[motor_id].temperate = (uint8_t)(data)[6];
  return;
}

}  // namespace motor_protocol