#ifndef SP__VISION_HPP
#define SP__VISION_HPP

#include <cstdint>

namespace sp
{
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};
static_assert(sizeof(VisionToGimbal) <= 64);

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  float supercap_power_in;
  float supercap_power_out;
  float supercap_voltage;
  uint8_t supercap_temperature;
  uint8_t supercap_status;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToHanging
{
  uint8_t head[2] = {'S', 'C'};
  uint8_t seq;
  uint8_t data[288];
  uint16_t crc16;
};
static_assert(sizeof(VisionToHanging) <= 293);

class Vision
{
public:
  bool control = false;
  bool fire = false;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;

  bool autoaim_alive = false;

  uint8_t data[288] = {0};
  uint8_t seq = 0;

  uint32_t autoaim_last_read_ms_ = 0;
  void update(uint8_t * buf, uint32_t len);
  void send(
    uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
    float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status);
  VisionToHanging rx_data_hanging_;

private:
  VisionToGimbal rx_data_;
  GimbalToVision tx_data_;

  uint8_t rx_buffer_[512] = {0};
  uint16_t rx_len_ = 0;
};

}  // namespace sp

#endif  // SP__VISION_HPP
