#include "super_cap.hpp"

namespace sp
{
SuperCap::SuperCap(SuperCapMode mode) : mode_(mode) {}

bool SuperCap::is_alive(uint32_t now_ms) const { return (now_ms - last_read_ms_ < 100); }

void SuperCap::read(uint8_t * data, uint32_t stamp_ms)
{
  // 数据解析
  this->power_in = ((int16_t)(((data)[1] << 8) | (data)[0])) / 10.0f;
  this->power_out = ((int16_t)(((data)[3] << 8) | (data)[2])) / 10.0f;
  this->voltage = ((int16_t)(((data)[5] << 8) | (data)[4])) / 100.0f;
  this->temputer_ = (data)[6];
  this->status_ = (data)[7];

  // 计算能量
  this->cap_energy = 0.5f * CAPACITANCE * this->voltage * this->voltage;

  // 更新时间戳
  last_read_ms_ = stamp_ms;
}

void SuperCap::write(
  uint8_t * data, uint16_t chassis_power_limit, uint16_t buffer_energy, uint8_t pm02_chassis_output)
{
  data[0] = static_cast<uint8_t>(mode_);

  data[1] = chassis_power_limit >> 8;
  data[2] = chassis_power_limit;

  // data[3] = 保留
  // data[4] = 保留

  data[5] = buffer_energy >> 8;
  data[6] = buffer_energy;

  data[7] = pm02_chassis_output;
}

}  // namespace sp
