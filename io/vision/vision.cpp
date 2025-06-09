#include "vision.hpp"

#include "memory"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usbd_cdc_if.h"

namespace sp
{
Vision::Vision(bool reverse_yaw, bool reverse_pitch)
: yaw_sign_(reverse_yaw ? -1.0f : 1.0f), pitch_sign_(reverse_pitch ? -1.0f : 1.0f)
{
}

void Vision::update(uint8_t * buf, uint32_t len)
{
  if (len != sizeof(rx_data_)) return;
  if (buf[0] != 'S' || buf[1] != 'P') return;
  if (!check_crc16(buf, len)) return;

  std::copy(buf, buf + len, reinterpret_cast<uint8_t *>(&rx_data_));
  this->control = rx_data_.control != 0;
  this->fire = rx_data_.fire != 0;
}

void Vision::send(
  uint8_t mode, float q[4], float yaw, float vyaw, float pitch, float vpitch, float bullet_speed)
{
  tx_data_.mode = mode;
  tx_data_.q[0] = q[0];
  tx_data_.q[1] = q[1];
  tx_data_.q[2] = q[2];
  tx_data_.q[3] = q[3];
  tx_data_.yaw = yaw;
  tx_data_.vyaw = vyaw;
  tx_data_.pitch = pitch;
  tx_data_.vpitch = vpitch;
  tx_data_.bullet_speed = bullet_speed;
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

float Vision::yaw_motor_torque(float yaw, float vyaw) const
{
  return yaw_sign_ * (limit_angle(rx_data_.yaw - yaw) * rx_data_.yaw_kp +
                      (rx_data_.vyaw - vyaw) * rx_data_.yaw_kd + rx_data_.yaw_torque);
}

float Vision::pitch_motor_torque(float pitch, float vpitch) const
{
  return pitch_sign_ * (limit_angle(rx_data_.pitch - pitch) * rx_data_.pitch_kp +
                        (rx_data_.vpitch - vpitch) * rx_data_.pitch_kd + rx_data_.pitch_torque);
}

}  // namespace sp
