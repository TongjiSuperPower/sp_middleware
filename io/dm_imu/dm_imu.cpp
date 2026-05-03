#include "dm_imu.hpp"

#include "dm_imu_defs.h"

namespace sp
{

DM_IMU::DM_IMU(uint32_t tx_id, uint32_t rx_id)
: tx_id(tx_id), rx_id(rx_id), temp(0), has_read_(false), last_read_ms_(0)
{
  for (int i = 0; i < 3; ++i) {
    acc[i] = 0.0f;
    gyro[i] = 0.0f;
    euler[i] = 0.0f;
  }
  quat[0] = 1.0f;
  for (int i = 1; i < 4; ++i) {
    quat[i] = 0.0f;
  }
}

bool DM_IMU::is_open() const { return has_read_; }

bool DM_IMU::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

float DM_IMU::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DM_IMU::write_request(uint8_t * data, uint8_t reg) const
{
  data[0] = 0xCC;  // 请求帧头
  data[1] = reg;   // 寄存器 RID
  data[2] = 0x00;  // 操作类型读
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void DM_IMU::read(uint8_t * data, uint32_t stamp_ms)
{
  last_read_ms_ = stamp_ms;
  if (!has_read_) {
    has_read_ = true;
  }

  // 首字节区分回传的数据包种类
  switch (data[0]) {
    case DM_IMU_REG_ACCEL: {
      temp = (int8_t)data[1];

      uint16_t acc_x_int = (data[3] << 8) | data[2];
      uint16_t acc_y_int = (data[5] << 8) | data[4];
      uint16_t acc_z_int = (data[7] << 8) | data[6];

      acc[0] = uint_to_float(acc_x_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
      acc[1] = uint_to_float(acc_y_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
      acc[2] = uint_to_float(acc_z_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
      break;
    }
    case DM_IMU_REG_GYRO: {
      uint16_t gyro_x_int = (data[3] << 8) | data[2];
      uint16_t gyro_y_int = (data[5] << 8) | data[4];
      uint16_t gyro_z_int = (data[7] << 8) | data[6];

      gyro[0] = uint_to_float(gyro_x_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
      gyro[1] = uint_to_float(gyro_y_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
      gyro[2] = uint_to_float(gyro_z_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
      break;
    }
    case DM_IMU_REG_EULER: {
      uint16_t pitch_int = (data[3] << 8) | data[2];
      uint16_t yaw_int = (data[5] << 8) | data[4];
      uint16_t roll_int = (data[7] << 8) | data[6];

      euler[0] = uint_to_float(pitch_int, DM_IMU_PITCH_MIN, DM_IMU_PITCH_MAX, 16);
      euler[1] = uint_to_float(yaw_int, DM_IMU_YAW_MIN, DM_IMU_YAW_MAX, 16);
      euler[2] = uint_to_float(roll_int, DM_IMU_ROLL_MIN, DM_IMU_ROLL_MAX, 16);
      break;
    }
    case DM_IMU_REG_QUAT: {
      int w_int = (data[1] << 6) | (data[2] >> 2);
      int x_int = ((data[2] & 0x03) << 12) | (data[3] << 4) | (data[4] >> 4);
      int y_int = ((data[4] & 0x0F) << 10) | (data[5] << 2) | (data[6] >> 6);
      int z_int = ((data[6] & 0x3F) << 8) | data[7];

      quat[0] = uint_to_float(w_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[1] = uint_to_float(x_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[2] = uint_to_float(y_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[3] = uint_to_float(z_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      break;
    }
  }
}

}  // namespace sp