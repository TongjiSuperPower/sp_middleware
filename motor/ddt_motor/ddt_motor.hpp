#ifndef SP__DDT_MOTOR_HPP
#define SP__DDT_MOTOR_HPP

#include <cstdint>

namespace sp
{

class DDT_Motor
{
public:
  DDT_Motor(uint8_t motor_id, float torque_const);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  uint8_t error;  // 只读! 0x00: 无故障, 略
  uint8_t mode;  // 只读! 0x00: 电压开环, 0x01: 电流环, 0x02: 速度环, 0x09: 失能模式, OxOA: 使能模式

  float angle = 0;   // 只读! 单位: rad
  float speed = 0;   // 只读! 单位: rad/s
  float torque = 0;  // 只读! 单位: N·m

  void read(const uint8_t * data);
  void write(uint8_t * data) const;

  // 单位: N·m
  void cmd(float value);

private:
  const uint8_t motor_id_;
  const float torque_const_;
  int16_t cmd_raw_ = 0;
};

}  // namespace sp

#endif  // SP__DDT_MOTOR_HPP