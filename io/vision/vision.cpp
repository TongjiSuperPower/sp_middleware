#include "vision.hpp"

#include <cstring>

#include "cmsis_os.h"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usbd_cdc_if.h"

extern "C" {
#if defined(__GNUC__)
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len) __attribute__((weak));
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) __attribute__((weak));
#else
/* For non-GNU toolchains, declare without weak attribute; the project
  should ensure one of these is provided. */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#endif
}

static inline uint8_t CDC_Transmit_Select(uint8_t* Buf, uint16_t Len)
{
#if defined(__GNUC__)
  if (CDC_Transmit_HS) return CDC_Transmit_HS(Buf, Len);
  if (CDC_Transmit_FS) return CDC_Transmit_FS(Buf, Len);
  return 0xFF; /* no implementation available */
#else
  /* Prefer HS when both declared; fall back to FS. Linker will error
    if neither is provided for non-GNU toolchains. */
  extern uint8_t CDC_Transmit_HS(uint8_t*, uint16_t);
  extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);
  /* Try HS first */
  return CDC_Transmit_HS ? CDC_Transmit_HS(Buf, Len) : CDC_Transmit_FS(Buf, Len);
#endif
}

namespace sp
{

void Vision::update(uint8_t * buf, uint32_t len)
{
  if (len == 0 || buf == nullptr) return;

  // Append new data to the buffer, discard all if overflow
  if (rx_len_ + len > sizeof(rx_buffer_)) {
    rx_len_ = 0;
  }
  std::copy(buf, buf + len, rx_buffer_ + rx_len_);
  rx_len_ += len;

  while (rx_len_ >= 2) {
    if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'P') {
      if (rx_len_ >= sizeof(VisionToGimbal)) {
        if (check_crc16(rx_buffer_, sizeof(VisionToGimbal))) {
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal),
            reinterpret_cast<uint8_t *>(&rx_data_));

          this->autoaim_alive = (osKernelSysTick() - this->autoaim_last_read_ms_ < 100);
          this->control = (rx_data_.mode != 0);
          this->fire = (rx_data_.mode == 2);
          this->yaw = rx_data_.yaw;
          this->yaw_vel = rx_data_.yaw_vel;
          this->yaw_acc = rx_data_.yaw_acc;
          this->pitch = rx_data_.pitch;
          this->pitch_vel = rx_data_.pitch_vel;
          this->pitch_acc = rx_data_.pitch_acc;
          this->auto_aim_target = rx_data_.target;
          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          rx_len_ -= sizeof(VisionToGimbal);
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
      }
    }
    else if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'C') {
      if (rx_len_ >= sizeof(VisionToHanging)) {
        if (check_crc16(rx_buffer_, sizeof(VisionToHanging))) {
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToHanging),
            reinterpret_cast<uint8_t *>(&rx_data_hanging_));

          this->seq = rx_data_hanging_.seq;
          for (int i = 0; i < 288; i++) this->data[i] = rx_data_hanging_.data[i];

          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          rx_len_ -= sizeof(VisionToHanging);
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToHanging), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
      }
    }
    else {
      // Invalid head, shift 1 byte forward to find next 'S'
      rx_len_--;
      if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
    }
  }
}

void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status)
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
  tx_data_.supercap_power_in = supercap_power_in;
  tx_data_.supercap_power_out = supercap_power_out;
  tx_data_.supercap_voltage = supercap_voltage;
  tx_data_.supercap_temperature = supercap_temperature;
  tx_data_.supercap_status = supercap_status;
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  CDC_Transmit_Select(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status,
  uint8_t game_progress)
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
  tx_data_.supercap_power_in = supercap_power_in;
  tx_data_.supercap_power_out = supercap_power_out;
  tx_data_.supercap_voltage = supercap_voltage;
  tx_data_.supercap_temperature = supercap_temperature;
  tx_data_.supercap_status = supercap_status;
  tx_data_.game_progress = game_progress;
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  CDC_Transmit_Select(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}
}  // namespace sp