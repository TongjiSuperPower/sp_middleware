#include "mahony.hpp"

#include <cmath>

#include "tools/gimbal/gimbal.hpp"

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

  //这个四元数是归一化之后的四元数,
  //而且我检验了,这个四元数Q_(W <- G )= {q[0],q[1],q[2],q[3]}是标量在前形式,后边虚部所表示的向量是旋转轴在地面系W下的坐标
  //这个四元数的意义是
  //1.将同一个向量在云台系下的坐标转换到地面系下的坐标即 v_world = Q * v_gimbal * Q_conjugate
  //这个向量直接转换成欧拉角,此欧拉角的含义是云台系当前姿态相对于地面系的欧拉角(内旋ZYX顺序)

  // 计算欧拉角(云台相对于底盘的欧拉角)(通过四元数转换)
  //稍微改了以下老代码(老代码是对的),数值上完全等价
  //只是用到了1=q[0]^2+q[1]^2+q[2]^2+q[3]^2这个关系简化了一些计算
  //以后的同学在用ai理解mahony算法的时候会更容易,形式更加相同

  this->yaw = std::atan2(
    2.0f * (this->q[0] * this->q[3] + this->q[1] * this->q[2]),
    1.0f - 2.0f * (this->q[2] * this->q[2] + this->q[3] * this->q[3]));
  this->pitch = std::asin(2.0f * (this->q[0] * this->q[2] - this->q[1] * this->q[3]));
  this->roll = std::atan2(
    2.0f * (this->q[0] * this->q[1] + this->q[2] * this->q[3]),
    1.0f - 2.0f * (this->q[1] * this->q[1] + this->q[2] * this->q[2]));

  culculate_yaw_pitch_roll_rates(gx, gy, gz, this->roll, this->pitch, this->yaw);

  //调用函数计算对应欧拉角的微分
  pitch_geom_calc();
}

void Mahony::culculate_yaw_pitch_roll_rates(
  float wx, float wy, float wz, float roll, float pitch, float yaw)
{
  this->vroll = wx + wy * std::sin(roll) * std::tan(pitch) + wz * std::cos(roll) * std::tan(pitch);
  this->vpitch = wy * std::cos(roll) - wz * std::sin(roll);
  this->vyaw = wy * std::sin(roll) / std::cos(pitch) + wz * std::cos(roll) / std::cos(pitch);
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
  // 需要注意的是:上电后在确定底盘相对于地面的初始yaw值的时候 要在mahony的init阶段直接将yaw_relative_angle赋值给底盘系imu的yaw0(正负自己确定)!!!

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

void Mahony::pitch_geom_calc()
{
  this->pitch_geom_last = this->pitch_geom;
  sp::Gimbal::quaternion_frame_transform(
    this->q, this->g_world, this->g_base, true);  // 将重力向量从地面系转换到底盘系

  this->pitch_geom =
    std::atan2(std::sqrt(g_base[1] * g_base[1] + g_base[0] * g_base[0]), g_base[2]);
  // 计算几何pitch角及其微分
  this->vpitch_geom = (this->pitch_geom - this->pitch_geom_last) / dt_;
}

}  // namespace sp
