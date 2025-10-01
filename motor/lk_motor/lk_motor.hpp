#ifndef SP__LK_MOTOR_HPP
#define SP__LK_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float MG4005i10_TORQUE_CONST = 0.06f;  // N·m/A
constexpr float MG4005i10_P10 = 10.0f;           // 减速比

class LK_Motor
{
public:
  // 仅实现了多电机模式!
  LK_Motor(uint8_t motor_id, float ratio = MG4005i10_P10, float torque_const = MG4005i10_TORQUE_CONST);

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
  const float ratio_;
  const float torque_const_;
  bool has_read_;
  int16_t cmd_raw_ = 0;

  int32_t step_;
  uint16_t last_ecd_;
};

}  // namespace sp

#endif  // SP__LK_MOTOR_HPP