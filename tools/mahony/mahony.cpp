#include "mahony.hpp"

#include <cmath>

namespace sp
{
Mahony::Mahony(float dt, float kp, float ki)
: dt_(dt),
  two_kp_(2 * kp),
  two_ki_(2 * ki),
  inited_(false),
  integral_fbx_(0.0f),
  integral_fby_(0.0f),
  integral_fbz_(0.0f)
{
}

void Mahony::update(const float acc[3], const float gyro[3])
{
  update(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}

void Mahony::update(float ax, float ay, float az, float wx, float wy, float wz)
{
  if (!inited_) {
    inited_ = true;
    init(ax, ay, az);
    return;
  }

  float gx = wx;
  float gy = wy;
  float gz = wz;

  // Normalise accelerometer measurement
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated direction of gravity and vector perpendicular to magnetic flux
  float halfvx = this->q[1] * this->q[3] - this->q[0] * this->q[2];
  float halfvy = this->q[0] * this->q[1] + this->q[2] * this->q[3];
  float halfvz = this->q[0] * this->q[0] - 0.5f + this->q[3] * this->q[3];

  // Error is sum of cross product between estimated and measured direction of gravity
  float halfex = (ay * halfvz - az * halfvy);
  float halfey = (az * halfvx - ax * halfvz);
  float halfez = (ax * halfvy - ay * halfvx);

  // Compute and apply integral feedback if enabled
  if (two_ki_ > 0.0f) {
    // integral error scaled by Ki
    integral_fbx_ += two_ki_ * halfex * dt_;
    integral_fby_ += two_ki_ * halfey * dt_;
    integral_fbz_ += two_ki_ * halfez * dt_;

    // apply integral feedback
    wx += integral_fbx_;
    wy += integral_fby_;
    wz += integral_fbz_;
  }
  else {
    // prevent integral windup
    integral_fbx_ = 0.0f;
    integral_fby_ = 0.0f;
    integral_fbz_ = 0.0f;
  }

  // Apply proportional feedback
  wx += two_kp_ * halfex;
  wy += two_kp_ * halfey;
  wz += two_kp_ * halfez;

  // Integrate rate of change of quaternion
  // pre-multiply common factors
  wx *= 0.5f * dt_;
  wy *= 0.5f * dt_;
  wz *= 0.5f * dt_;

  float q0 = this->q[0] + (-this->q[1] * wx - this->q[2] * wy - this->q[3] * wz);
  float q1 = this->q[1] + (this->q[0] * wx + this->q[2] * wz - this->q[3] * wy);
  float q2 = this->q[2] + (this->q[0] * wy - this->q[1] * wz + this->q[3] * wx);
  float q3 = this->q[3] + (this->q[0] * wz + this->q[1] * wy - this->q[2] * wx);

  // Normalise quaternion
  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  this->yaw = std::atan2(
    2.0f * (this->q[0] * this->q[3] + this->q[1] * this->q[2]),
    2.0f * (this->q[0] * this->q[0] + this->q[1] * this->q[1]) - 1.0f);
  this->pitch = std::asin(2.0f * (this->q[0] * this->q[2] - this->q[1] * this->q[3]));
  this->roll = std::atan2(
    2.0f * (this->q[0] * this->q[1] + this->q[2] * this->q[3]),
    2.0f * (this->q[0] * this->q[0] + this->q[3] * this->q[3]) - 1.0f);

  this->vyaw = -gx * std::sin(this->pitch) + gy * std::cos(this->pitch) * std::sin(this->roll) +
               gz * std::cos(this->pitch) * std::cos(this->roll);
  this->vpitch = gy * std::cos(this->roll) - gz * std::sin(this->roll);
  this->vroll = gx;
}

void Mahony::init(float ax, float ay, float az)
{
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  float pitch0 = std::atan2(-ax, az);
  float roll0 = std::atan2(ay, az);
  float yaw0 = 0.0f;

  float cr2 = std::cos(roll0 * 0.5f);
  float cp2 = std::cos(pitch0 * 0.5f);
  float cy2 = std::cos(yaw0 * 0.5f);
  float sr2 = std::sin(roll0 * 0.5f);
  float sp2 = std::sin(pitch0 * 0.5f);
  float sy2 = std::sin(yaw0 * 0.5f);

  float q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  float q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  float q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  float q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  this->yaw = yaw0;
  this->pitch = pitch0;
  this->roll = roll0;
}

}  // namespace sp
