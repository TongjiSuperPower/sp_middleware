#ifndef SP__LK_MOTOR_HPP
#define SP__LK_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float MG4005i10_P10 = 10.0f;           // 减速比
constexpr float MG4005i10_TORQUE_CONST = 0.06f;  // N·m/A

class LK_Motor
{
public:
  // 仅实现了MG系列单电机模式!
  // motor_type: 电机型号， 取值见LK_Motors`
  // ratio: 减速比
  LK_Motor(
    uint8_t motor_id, float ratio = MG4005i10_P10, float torque_const = MG4005i10_TORQUE_CONST);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  uint8_t motorState = 0x10;
  uint8_t errorState = 0;

  float angle = 0;             // 只读! 单位: rad
  float multicycle_angle = 0;  // 只读! 单位: rad
  float speed = 0;             // 只读! 单位: rad/s
  float torque = 0;            // 只读! 单位: N·m
  int8_t temp = 0;             // 只读! 单位: 摄氏度

  // 该命令读取当前电机的温度、电压和错误状态标志
  void write_state1(uint8_t * data) const;
  void read_state1(const uint8_t * data);

  // 电机状态没有恢复正常时，错误标志无法清除
  void write_clear_error(uint8_t * data) const;

  // 该命令读取当前电机的温度、电机转矩电流、转速、编码器位置
  void write_state2(uint8_t * data) const;
  void read_state2(const uint8_t * data);

  // 将电机从开启状态切换到关闭状态，清除电机转动圈数及之前接收的控制指令，此时电机仍然可以回复控制命令，但不会执行动作
  void write_turn_off(uint8_t * data) const;
  // 将电机从关闭状态切换到开启状态
  void write_turn_on(uint8_t * data) const;
  // 停止电机，但不清除电机运行状态，再次发送控制指令即可控制电机动作
  void write_stop(uint8_t * data) const;

  // 主机发送该命令以控制电机的位置（单圈角度）
  void write_position(uint8_t * data) const;
  // 主机发送该命令以控制电机的位置增量
  void write_angle_increment(uint8_t * data) const;
  // 主机发送该命令以控制电机的转矩电流输出
  void write_torque(uint8_t * data) const;

  // 控制值 direction 设置电机转动的方向，为 uint8_t 类型，0x00 代表顺时针，0x01 代表逆时针
  void cmd_position(int32_t position, uint8_t direction);
  void cmd_angle(int32_t angle);
  void cmd_torque(float value);

private:
  const uint8_t motor_id_;
  const float ratio_;
  const float torque_const_;

  int32_t cmd_angle_;
  int16_t cmd_raw_ = 0;

  bool has_read_;
  int32_t step_;
  uint16_t last_ecd_;

  uint32_t cmd_position_;
  uint8_t spin_direction_;
};

}  // namespace sp

#endif  // SP__LK_MOTOR_HPP