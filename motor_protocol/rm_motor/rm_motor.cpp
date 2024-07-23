#include "rm_motor.hpp"

namespace motor_protocol
{
RM_Motor::RM_Motor(CAN_HandleTypeDef * hcan, uint32_t stdid) : hcan_(hcan), stdid_(stdid) {}

// 发送电机控制指令
HAL_StatusTypeDef RM_Motor::motor_cmd(
  int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  motor_tx_message_.StdId = stdid_;
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

  return HAL_CAN_AddTxMessage(hcan_, &motor_tx_message_, motor_can_send_data_, &send_mail_box);
}

// 解码电机数据
void RM_Motor::decode_motor_measure(uint8_t motor_id, uint8_t data[8])
{
  motor_measure_[motor_id].last_ecd = motor_measure_[motor_id].ecd;

  motor_measure_[motor_id].ecd = (uint16_t)(data[0] << 8 | data[1]);
  motor_measure_[motor_id].speed_rpm = (int16_t)(data[2] << 8 | data[3]);
  motor_measure_[motor_id].given_current = (int16_t)(data[4] << 8 | data[5]);
  motor_measure_[motor_id].temperate = (uint8_t)data[6];

  motor_measure_[motor_id].angle = (motor_measure_[motor_id].ecd - 4096) * M_PI / 4096;
  motor_measure_[motor_id].speed = motor_measure_[motor_id].speed_rpm * M_PI * 2 / 60;

  if (fabs(motor_measure_[motor_id].ecd - motor_measure_[motor_id].last_ecd) > 4096) {
    if (motor_measure_[motor_id].speed_rpm > 0) motor_measure_[motor_id].revolutions++;
    if (motor_measure_[motor_id].speed_rpm < 0) motor_measure_[motor_id].revolutions--;
  }

  motor_measure_[motor_id].rev_angle =
    motor_measure_[motor_id].revolutions * M_PI * 2 + motor_measure_[motor_id].angle;
  return;
}

}  // namespace motor_protocol