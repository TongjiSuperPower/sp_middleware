#ifndef SP__VISION_HPP
#define SP__VISION_HPP

#include <cstdint>

namespace sp
{
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t control;  // 0: 视觉不控制, 1: 视觉控制
  uint8_t fire;     // 0: 不开火， 1: 开火
  float yaw;
  float vyaw;
  float yaw_kp;
  float yaw_kd;
  float yaw_torque;
  float pitch;
  float vpitch;
  float pitch_kp;
  float pitch_kd;
  float pitch_torque;
  uint16_t crc16;
};

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float vyaw;
  float pitch;
  float vpitch;
  float bullet_speed;
  uint16_t crc16;
};

class Vision
{
public:
  Vision(bool reverse_yaw, bool reverse_pitch);

  bool control = false;
  bool fire = false;

  void update(uint8_t * buf, uint32_t len);
  void send(
    uint8_t mode, float q[4], float yaw, float vyaw, float pitch, float vpitch, float bullet_speed);

  float yaw_motor_torque(float yaw, float vyaw) const;
  float pitch_motor_torque(float pitch, float vpitch) const;

private:
  const float yaw_sign_;
  const float pitch_sign_;

  VisionToGimbal rx_data_;
  GimbalToVision tx_data_;
};

}  // namespace sp

#endif  // SP__VISION_HPP
