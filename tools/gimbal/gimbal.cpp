#include "gimbal.hpp"

#include <cmath>

namespace sp
{

Gimbal::Gimbal(
  float yaw0, float pitch0, bool reverse_yaw, bool reverse_yaw0, bool reverse_pitch,
  bool reverse_pitch0)
: yaw0_(yaw0),
  pitch0_(pitch0),
  sign_yaw_((reverse_yaw) ? -1.0f : 1.0f),
  sign_yaw0_((reverse_yaw0) ? -1.0f : 1.0f),
  sign_pitch_((reverse_pitch) ? -1.0f : 1.0f),
  sign_pitch0_((reverse_pitch0) ? -1.0f : 1.0f)
{
  this->yaw_fdb_in_joint = 0.0f;
  this->pitch_fdb_in_joint = 0.0f;
  this->yaw_set_in_joint = 0.0f;
  this->pitch_set_in_joint = 0.0f;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      this->R_base2world_[i][j] = 0.0f;
    }
  }
}

void Gimbal::update(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  //云台相对码盘值计算（正方向使用云台系右手定则）
  float yaw_realative_angle = sign_yaw_ * yaw_angle + sign_yaw0_ * yaw0_;
  float pitch_realative_angle = sign_pitch_ * pitch_angle + sign_pitch0_ * pitch0_;

  //化简后结果，详细推导见readme
  yaw_fdb_in_joint = atan2f(
    -2 * sinf(gimbal_imu.pitch) * sinf(gimbal_imu.pitch) * sinf(yaw_realative_angle) *
        cosf(pitch_realative_angle) +
      2 * sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * cosf(gimbal_imu.pitch) *
        cosf(yaw_realative_angle) +
      2 * sinf(gimbal_imu.pitch) * sinf(pitch_realative_angle) * sinf(yaw_realative_angle) *
        cosf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) +
      sinf(yaw_realative_angle) * cosf(pitch_realative_angle),
    -2 * sinf(gimbal_imu.pitch) * sinf(gimbal_imu.pitch) * cosf(pitch_realative_angle) *
        cosf(yaw_realative_angle) -
      2 * sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * sinf(yaw_realative_angle) *
        cosf(gimbal_imu.pitch) +
      2 * sinf(gimbal_imu.pitch) * sinf(pitch_realative_angle) * cosf(gimbal_imu.pitch) *
        cosf(gimbal_imu.roll) * cosf(yaw_realative_angle) +
      cosf(pitch_realative_angle) * cosf(yaw_realative_angle));
  pitch_fdb_in_joint = asinf(
    sinf(2 * gimbal_imu.pitch - pitch_realative_angle) / 2 -
    sinf(2 * gimbal_imu.pitch + pitch_realative_angle) / 2 -
    sinf(-2 * gimbal_imu.pitch + gimbal_imu.roll + pitch_realative_angle) / 4 +
    sinf(2 * gimbal_imu.pitch - gimbal_imu.roll + pitch_realative_angle) / 4 +
    sinf(2 * gimbal_imu.pitch + gimbal_imu.roll - pitch_realative_angle) / 4 +
    sinf(2 * gimbal_imu.pitch + gimbal_imu.roll + pitch_realative_angle) / 4);

  R_base2world_[0][0] = -(sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) -
                          sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll)) *
                          sinf(yaw_realative_angle) +
                        (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          sinf(pitch_realative_angle) * cosf(yaw_realative_angle) +
                        cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw) *
                          cosf(pitch_realative_angle) * cosf(yaw_realative_angle);
  R_base2world_[0][1] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) -
                         sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll)) *
                          cosf(yaw_realative_angle) +
                        (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          sinf(pitch_realative_angle) * sinf(yaw_realative_angle) +
                        sinf(yaw_realative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw) *
                          cosf(pitch_realative_angle);
  R_base2world_[0][2] = (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          cosf(pitch_realative_angle) -
                        sinf(pitch_realative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw);
  R_base2world_[1][0] = -(sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw) +
                          cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(yaw_realative_angle) +
                        (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(pitch_realative_angle) * cosf(yaw_realative_angle) +
                        sinf(gimbal_imu.yaw) * cosf(gimbal_imu.pitch) *
                          cosf(pitch_realative_angle) * cosf(yaw_realative_angle);
  R_base2world_[1][1] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw) +
                         cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          cosf(yaw_realative_angle) +
                        (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(pitch_realative_angle) * sinf(yaw_realative_angle) +
                        sinf(gimbal_imu.yaw) * sinf(yaw_realative_angle) * cosf(gimbal_imu.pitch) *
                          cosf(pitch_realative_angle);
  R_base2world_[1][2] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          cosf(pitch_realative_angle) -
                        sinf(gimbal_imu.yaw) * sinf(pitch_realative_angle) * cosf(gimbal_imu.pitch);
  R_base2world_[2][0] =
    -sinf(gimbal_imu.pitch) * cosf(pitch_realative_angle) * cosf(yaw_realative_angle) -
    sinf(gimbal_imu.roll) * sinf(yaw_realative_angle) * cosf(gimbal_imu.pitch) +
    sinf(pitch_realative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) *
      cosf(yaw_realative_angle);
  R_base2world_[2][1] =
    -sinf(gimbal_imu.pitch) * sinf(yaw_realative_angle) * cosf(pitch_realative_angle) +
    sinf(gimbal_imu.roll) * cosf(gimbal_imu.pitch) * cosf(yaw_realative_angle) +
    sinf(pitch_realative_angle) * sinf(yaw_realative_angle) * cosf(gimbal_imu.pitch) *
      cosf(gimbal_imu.roll);
  R_base2world_[2][2] =
    sinf(gimbal_imu.pitch) * sinf(pitch_realative_angle) +
    cosf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(pitch_realative_angle);
};

void Gimbal::calc(float yaw_set_in_world, float pitch_set_in_world)
{
  yaw_set_in_joint = atan2f(
    R_base2world_[0][1] * cosf(pitch_set_in_world) * cosf(yaw_set_in_world) +
      R_base2world_[1][1] * sinf(yaw_set_in_world) * cosf(pitch_set_in_world) +
      R_base2world_[2][1] * sinf(pitch_set_in_world),
    R_base2world_[0][0] * cosf(pitch_set_in_world) * cosf(yaw_set_in_world) +
      R_base2world_[1][0] * sinf(yaw_set_in_world) * cosf(pitch_set_in_world) +
      R_base2world_[2][0] * sinf(pitch_set_in_world));
  pitch_set_in_joint = asinf(
    R_base2world_[0][2] * cosf(pitch_set_in_world) * cosf(yaw_set_in_world) +
    R_base2world_[1][2] * sinf(yaw_set_in_world) * cosf(pitch_set_in_world) +
    R_base2world_[2][2] * sinf(pitch_set_in_world));
}

}  // namespace sp