#ifndef SP__DM_IMU_HPP
#define SP__DM_IMU_HPP

#include <cstdint>

#include "dm_imu_defs.h"
namespace sp
{

class DM_IMU
{
public:
  // tx_id: 上位机设置的 CAN ID (用于发送请求)
  // rx_id: 上位机设置的 MST ID (用于接收应答)
  // r_ab: dm_imu原始坐标系{b}在机器人坐标系{a}下的坐标矩阵
  DM_IMU(uint32_t tx_id, uint32_t rx_id, const float r_ab[3][3]);

  const uint32_t tx_id;
  const uint32_t rx_id;

  float acc[3] = {0.0f};   // 只读! 单位: m/s^2
  float gyro[3] = {0.0f};  // 只读! 单位: rad/s
  float euler[3];          // 只读! pitch, yaw, roll
  float quat[4];           // 只读! w, x, y, z
  int8_t temp;             // 只读! 单位: 摄氏度

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  // 将接收到的 CAN 帧数据解析并更新至公有变量
  void read(uint8_t * data, uint32_t stamp_ms);

  // 根据指定寄存器，填装请求发送缓存 (不直接调用FDCAN)
  void write_request(uint8_t * data, uint8_t reg) const;

private:
  bool has_read_;
  uint32_t last_read_ms_;
  const float r_ab_[3][3];
  static float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

}  // namespace sp

#endif  // SP__DM_IMU_HPP