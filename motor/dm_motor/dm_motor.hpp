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
  DM_Motor(
    uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax,
    bool multi_circle = false);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID
  const uint16_t tx_id_vel;  // 速度模式 控制帧ID (0x200 + CAN ID)

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
  void write_disable(uint8_t * data) const;
  void write_clear_error(uint8_t * data) const;

  // MIT力控模式指令缓存, 单位: N·m
  void cmd(float torque);
  // 速度模式指令缓存, 单位: rad/s
  void cmd_velocity(float velocity);
  
  void cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff);

  // MIT速度控制模式
  // velocity: 目标速度 (rad/s)
  // kd: 速度环增益 (建议根据实际负载调试，范围 0~5)
  void cmd_mit_velocity(float velocity, float kd);

  // 完整的MIT模式数据打包
  void write_mit(uint8_t * data) const;

  void cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff);

  // MIT速度控制模式
  // velocity: 目标速度 (rad/s)
  // kd: 速度环增益 (建议根据实际负载调试，范围 0~5)
  void cmd_mit_velocity(float velocity, float kd);

  // 完整的MIT模式数据打包
  void write_mit(uint8_t * data) const;

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_;
  uint32_t last_read_ms_;

  float cmd_torque_;

  //多圈计数
  float angle_raw_ = 0;
  float last_angle_raw_ = 0.0f;
  int32_t circle_;
  bool multi_circle_;

  // 新增 MIT 模式参数缓存
  float cmd_p_des_ = 0.0f;
  float cmd_v_des_ = 0.0f;
  float cmd_kp_ = 0.0f;
  float cmd_kd_ = 0.0f;
  float cmd_t_ff_ = 0.0f;
};

}  // namespace sp

#endif  // SP__DM_MOTOR_HPP