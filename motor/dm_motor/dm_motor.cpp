#include "dm_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
DM_Motor::DM_Motor(
  uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax, bool multi_circle)
: rx_id(master_id),
  tx_id(can_id),
  tx_id_vel(0x200 + can_id),
  pmax_(pmax),
  vmax_(vmax),
  tmax_(tmax),
  multi_circle_(multi_circle),
  has_read_(false),
  circle_(0)
{
}

bool DM_Motor::is_open() const { return has_read_; }

bool DM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

//multi_circle是true就用多圈
void DM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  last_read_ms_ = stamp_ms;

  this->error = data[0] >> 4;
  this->speed = uint_to_float((data[3] << 4) | (data[4] >> 4), -vmax_, vmax_, 12);
  this->torque = uint_to_float(((data[4] & 0xF) << 8) | data[5], -tmax_, tmax_, 12);

  this->angle_raw_ = uint_to_float((data[1] << 8) | data[2], -pmax_, pmax_, 16);

  if (!has_read_) {
    has_read_ = true;
    last_angle_raw_ = angle_raw_;
  }
  if (angle_raw_ - last_angle_raw_ > pmax_)
    circle_--;
  else if (angle_raw_ - last_angle_raw_ < -pmax_)
    circle_++;

  last_angle_raw_ = angle_raw_;
  this->angle = multi_circle_ ? angle_raw_ + circle_ * 2 * pmax_ : angle_raw_;
}

void DM_Motor::write(uint8_t * data) const
{
  auto torque_uint = float_to_uint(cmd_torque_, -tmax_, tmax_, 12);
  data[3] = 0x00;              // Kp[11:8]
  data[4] = 0x00;              // Kp[7:0]
  data[5] = 0x00;              // Kd[11:4]
  data[6] = torque_uint >> 8;  // Kd[3:0]
  data[7] = torque_uint;
}

void DM_Motor::write_velocity(uint8_t * data) const
{
  // 浮点数占用 4 个字节，直接内存拷贝即可满足“低位在前”的小端要求
  std::memcpy(data, &cmd_velocity_, sizeof(float));

  // 保险起见，将后4个字节清零（CAN帧通常发8字节，尽管速度模式只用前4字节）
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void DM_Motor::write_enable(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;
}

void DM_Motor::write_disable(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;
}

void DM_Motor::write_clear_error(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFB;
}

void DM_Motor::cmd(float torque) { cmd_torque_ = limit_max(torque, tmax_); }

void DM_Motor::cmd_velocity(float velocity) { cmd_velocity_ = limit_max(velocity, vmax_); }

void DM_Motor::cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff)
{
  cmd_p_des_ = limit_max(p_des, pmax_);
  cmd_v_des_ = limit_max(v_des, vmax_);

  cmd_kp_ = limit_max(kp, 500.0f);
  if (cmd_kp_ < 0.0f) cmd_kp_ = 0.0f;
  cmd_kd_ = limit_max(kd, 5.0f);
  if (cmd_kd_ < 0.0f) cmd_kd_ = 0.0f;

  cmd_t_ff_ = limit_max(t_ff, tmax_);
}

void DM_Motor::cmd_mit_velocity(float velocity, float kd)
{
  cmd_mit(0.0f, velocity, 0.0f, kd, 0.0f);
}

void DM_Motor::write_mit(uint8_t * data) const
{
  uint16_t p_int = float_to_uint(cmd_p_des_, -pmax_, pmax_, 16);
  uint16_t v_int = float_to_uint(cmd_v_des_, -vmax_, vmax_, 12);
  uint16_t kp_int = float_to_uint(cmd_kp_, 0.0f, 500.0f, 12);
  uint16_t kd_int = float_to_uint(cmd_kd_, 0.0f, 5.0f, 12);
  uint16_t t_int = float_to_uint(cmd_t_ff_, -tmax_, tmax_, 12);

  data[0] = (p_int >> 8) & 0xFF;                           // p_des[15:8]
  data[1] = p_int & 0xFF;                                  // p_des[7:0]
  data[2] = (v_int >> 4) & 0xFF;                           // v_des[11:4]
  data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);  // v_des[3:0] | Kp[11:8]
  data[4] = kp_int & 0xFF;                                 // Kp[7:0]
  data[5] = (kd_int >> 4) & 0xFF;                          // Kd[11:4]
  data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);  // Kd[3:0] | t_ff[11:8]
  data[7] = t_int & 0xFF;                                  // t_ff[7:0]
}

}  // namespace sp