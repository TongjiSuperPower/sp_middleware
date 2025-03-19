#ifndef SP__LK_MOTOR_HPP
#define SP__LK_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float MF9025_TORQUE_CONST = 0.32f;  // N·m/A

class LK_Motor
{
public:
  // 仅实现了多电机模式!
  LK_Motor(uint8_t motor_id, float torque_const = MF9025_TORQUE_CONST);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  float angle = 0;   // 只读! 单位: rad
  float speed = 0;   // 只读! 单位: rad/s
  float torque = 0;  // 只读! 单位: N·m
  int8_t temp = 0;   // 只读! 单位: 摄氏度

  void read(const uint8_t * data);
  void write(uint8_t * data) const;

  void cmd(float value);

private:
  const uint8_t motor_id_;
  const float torque_const_;
  int16_t cmd_raw_ = 0;
};

}  // namespace sp

#endif  // SP__LK_MOTOR_HPP