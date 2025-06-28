#include "vision.hpp"

#include "memory"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usbd_cdc_if.h"

namespace sp
{

void Vision::update(uint8_t * buf, uint32_t len)
{
  if (len != sizeof(rx_data_)) return;
  if (buf[0] != 'S' || buf[1] != 'P') return;
  if (!check_crc16(buf, len)) return;

  std::copy(buf, buf + len, reinterpret_cast<uint8_t *>(&rx_data_));

  this->control = (rx_data_.mode != 0);
  this->fire = (rx_data_.mode == 2);
  this->yaw = rx_data_.yaw;
  this->yaw_vel = rx_data_.yaw_vel;
  this->yaw_acc = rx_data_.yaw_acc;
  this->pitch = rx_data_.pitch;
  this->pitch_vel = rx_data_.pitch_vel;
  this->pitch_acc = rx_data_.pitch_acc;
}

void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count)
{
  tx_data_.mode = mode;
  tx_data_.q[0] = q[0];
  tx_data_.q[1] = q[1];
  tx_data_.q[2] = q[2];
  tx_data_.q[3] = q[3];
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.bullet_speed = bullet_speed;
  tx_data_.bullet_count = bullet_count;
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

}  // namespace sp
