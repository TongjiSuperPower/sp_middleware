#include "dm_motor.hpp"

#include <cstring>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
DM_Motor::DM_Motor(uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax)
: rx_id(master_id), tx_id(can_id), tx_id_vel(0x200 + can_id), pmax_(pmax), vmax_(vmax), tmax_(tmax)
{
}

bool DM_Motor::is_open() const { return has_read_; }

bool DM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

void DM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  this->error = data[0] >> 4;
  this->angle = uint_to_float((data[1] << 8) | data[2], -pmax_, pmax_, 16);
  this->speed = uint_to_float((data[3] << 4) | (data[4] >> 4), -vmax_, vmax_, 12);
  this->torque = uint_to_float(((data[4] & 0xF) << 8) | data[5], -tmax_, tmax_, 12);
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

// 新增：速度模式打包
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

}  // namespace sp
