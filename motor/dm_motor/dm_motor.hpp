#ifndef SP__DM_MOTOR_HPP
#define SP__DM_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float J4310_MAX_TORQUE = 7.0f;  // N·m

class DM_Motor
{
public:
  // can_id: 电机控制帧ID, 对应tx_id
  // master_id: 电机反馈帧ID, 对应rx_id
  // pmax: position最大值, 单位: rad
  // vmax: velocity最大值, 单位: rad/s
  // tmax: torque最大值, 单位: N·m
  DM_Motor(uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  // 只读! 8: 超压, 9: 欠压, A: 过流, B: MOS过温, C: 线圈过温, D: 通讯丢失, E: 过载
  uint8_t error = 0;

  float angle = 0;   // 只读! 单位: rad
  float speed = 0;   // 只读! 单位: rad/s
  float torque = 0;  // 只读! 单位: N·m

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data) const;
  void write_enable(uint8_t * data) const;
  void write_clear_error(uint8_t * data) const;

  // MIT力控模式, 单位: N·m
  void cmd(float torque);

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_ = false;
  uint32_t last_read_ms_;

  float cmd_torque_;
};

}  // namespace sp

#endif  // SP__DM_MOTOR_HPP