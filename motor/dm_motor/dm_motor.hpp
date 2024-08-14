#ifndef MOTOR__DM_MOTOR_HPP
#define MOTOR__DM_MOTOR_HPP

#include <cstdint>

namespace motor
{
class DM_Motor
{
public:
  // can_id: 给电机发送数据的ID, 对应tx_id
  // master_id: 电机反馈回来数据的ID, 对应rx_id
  DM_Motor(uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax);

  const uint16_t rx_id;
  const uint16_t tx_id;

  float angle;   // rad
  float speed;   // rad/s
  float torque;  // N·m

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data) const;
  void write_enable(uint8_t * data) const;

  // 只实现了MIT力控模式
  void cmd(float torque);

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_;
  uint32_t last_read_ms_;

  float cmd_torque_;
};

}  // namespace motor

#endif  // MOTOR__DM_MOTOR_HPP