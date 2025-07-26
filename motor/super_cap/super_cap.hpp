#ifndef SP__SUPERCAP_HPP
#define SP__SUPERCAP_HPP

#include <cstdint>

namespace sp
{
enum class SuperCapMode
{
  AUTOMODE = 0x00,     // 自动模式（默认）
  DISOUTPUT,           // 只充不放
  DISCHARGE,           // 只放不充
  DISCHARGE_DISOUTPUT  // 不充不放
};

constexpr float CAPACITANCE = 5.0f;  // 电容容量

class SuperCap
{
public:
  const uint16_t rx_id = 0x301;  // 电容反馈帧ID
  const uint16_t tx_id = 0x300;  // 电容控制帧ID

  float power_in;   // 电管输出
  float power_out;  // 电容组充电功率
  float voltage;    // 电容电压（放到3V，然后3v->6V后可再次放电）

  float cap_energy;

  SuperCap(SuperCapMode mode_ = SuperCapMode::AUTOMODE);

  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data, uint16_t power_limit, uint16_t buffer_energy, uint8_t chassis_output);

private:
  SuperCapMode mode_;

  uint8_t temputer_;  // 温度(现阶段恒定为25)
  uint8_t status_;    // 状态标志位（现阶段保留为0x10）

  uint32_t last_read_ms_;  // 上一次读取数据的时间戳
};

}  // namespace sp

#endif  // IO_SUPERCAP_HPP