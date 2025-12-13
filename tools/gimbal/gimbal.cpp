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
  sign_pitch_((reverse_pitch) ? -1.0f : 1.0f),
  yaw_relative_angle_filter(0.1f),
  pitch_relative_angle_filter(0.1f),
  yaw_relative_angle_filter0(0.1f),
  pitch_relative_angle_filter0(0.1f)
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
  float yaw_relative_angle = sign_yaw_ * (yaw_angle - yaw0_);
  float pitch_relative_angle = sign_pitch_ * (pitch_angle - pitch0_);

  // 使用低通滤波器对角度进行滤波
  yaw_relative_angle_filter0.update(yaw_relative_angle);
  pitch_relative_angle_filter0.update(pitch_relative_angle);

  yaw_relative_angle = yaw_relative_angle_filter0.out;
  pitch_relative_angle = pitch_relative_angle_filter0.out;

  //化简后结果，详细推导见readme
  yaw_fdb_in_joint = yaw_relative_angle;
  pitch_fdb_in_joint = pitch_relative_angle;

  R_base2world_[0][0] = -(sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) -
                          sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll)) *
                          sinf(yaw_relative_angle) +
                        (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          sinf(pitch_relative_angle) * cosf(yaw_relative_angle) +
                        cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw) * cosf(pitch_relative_angle) *
                          cosf(yaw_relative_angle);
  R_base2world_[0][1] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) -
                         sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll)) *
                          cosf(yaw_relative_angle) +
                        (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          sinf(pitch_relative_angle) * sinf(yaw_relative_angle) +
                        sinf(yaw_relative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw) *
                          cosf(pitch_relative_angle);
  R_base2world_[0][2] = (sinf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw) +
                         sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw)) *
                          cosf(pitch_relative_angle) -
                        sinf(pitch_relative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.yaw);
  R_base2world_[1][0] = -(sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw) +
                          cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(yaw_relative_angle) +
                        (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(pitch_relative_angle) * cosf(yaw_relative_angle) +
                        sinf(gimbal_imu.yaw) * cosf(gimbal_imu.pitch) * cosf(pitch_relative_angle) *
                          cosf(yaw_relative_angle);
  R_base2world_[1][1] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.roll) * sinf(gimbal_imu.yaw) +
                         cosf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          cosf(yaw_relative_angle) +
                        (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          sinf(pitch_relative_angle) * sinf(yaw_relative_angle) +
                        sinf(gimbal_imu.yaw) * sinf(yaw_relative_angle) * cosf(gimbal_imu.pitch) *
                          cosf(pitch_relative_angle);
  R_base2world_[1][2] = (sinf(gimbal_imu.pitch) * sinf(gimbal_imu.yaw) * cosf(gimbal_imu.roll) -
                         sinf(gimbal_imu.roll) * cosf(gimbal_imu.yaw)) *
                          cosf(pitch_relative_angle) -
                        sinf(gimbal_imu.yaw) * sinf(pitch_relative_angle) * cosf(gimbal_imu.pitch);
  R_base2world_[2][0] =
    -sinf(gimbal_imu.pitch) * cosf(pitch_relative_angle) * cosf(yaw_relative_angle) -
    sinf(gimbal_imu.roll) * sinf(yaw_relative_angle) * cosf(gimbal_imu.pitch) +
    sinf(pitch_relative_angle) * cosf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) *
      cosf(yaw_relative_angle);
  R_base2world_[2][1] =
    -sinf(gimbal_imu.pitch) * sinf(yaw_relative_angle) * cosf(pitch_relative_angle) +
    sinf(gimbal_imu.roll) * cosf(gimbal_imu.pitch) * cosf(yaw_relative_angle) +
    sinf(pitch_relative_angle) * sinf(yaw_relative_angle) * cosf(gimbal_imu.pitch) *
      cosf(gimbal_imu.roll);
  R_base2world_[2][2] = sinf(gimbal_imu.pitch) * sinf(pitch_relative_angle) +
                        cosf(gimbal_imu.pitch) * cosf(gimbal_imu.roll) * cosf(pitch_relative_angle);

  //rotation_matrix_to_euler(R_base2world_, euler_r);
};

void Gimbal::update_q(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
  const float & roll0_)
{
  /* ------------------------------------------------
   * Step 0: 编码器角度 → 云台相对底盘欧拉角
   * ------------------------------------------------ */
  float yaw_rel = sign_yaw_ * (yaw_angle - yaw0_);
  float pitch_rel = sign_pitch_ * (pitch_angle - pitch0_);
  float roll_rel = roll0_;

  // 角度低通滤波（只对可控轴）
  yaw_relative_angle_filter.update(yaw_rel);
  pitch_relative_angle_filter.update(pitch_rel);

  yaw_rel = yaw_relative_angle_filter.out;
  pitch_rel = pitch_relative_angle_filter.out;

  /* ------------------------------------------------
   * Step 1: 半角
   * ------------------------------------------------ */
  float half_yaw = 0.5f * yaw_rel;
  float half_pitch = 0.5f * pitch_rel;
  float half_roll = 0.5f * roll_rel;

  float cy = cosf(half_yaw);
  float sy = sinf(half_yaw);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);

  /* ------------------------------------------------
   * Step 2: 构造 q_G_C（底盘 → 云台）
   * ZYX intrinsic: yaw → pitch → roll
   * ------------------------------------------------ */
  float w_GC = (cy * cp * cr + sy * sp * sr);
  float x_GC = -(cy * cp * sr - sy * sp * cr);
  float y_GC = -(cy * sp * cr + sy * cp * sr);
  float z_GC = -(sy * cp * cr - cy * sp * sr);

  /* ------------------------------------------------
   * Step 3: 读取 q_W_G（云台 → 世界）
   * ------------------------------------------------ */
  const float * qWG = gimbal_imu.q;
  float w_WG = qWG[0];
  float x_WG = qWG[1];
  float y_WG = qWG[2];
  float z_WG = qWG[3];

  /* ------------------------------------------------
   * Step 4: 四元数复合
   * q_W_C = q_W_G ⊗ q_G_C
   * ------------------------------------------------ */
  float w_WC = w_WG * w_GC - x_WG * x_GC - y_WG * y_GC - z_WG * z_GC;
  float x_WC = w_WG * x_GC + x_WG * w_GC + y_WG * z_GC - z_WG * y_GC;
  float y_WC = w_WG * y_GC - x_WG * z_GC + y_WG * w_GC + z_WG * x_GC;
  float z_WC = w_WG * z_GC + x_WG * y_GC - y_WG * x_GC + z_WG * w_GC;

  /* ------------------------------------------------
   * Step 5: 归一化（防数值漂移）
   * ------------------------------------------------ */
  float norm = sqrtf(w_WC * w_WC + x_WC * x_WC + y_WC * y_WC + z_WC * z_WC);
  if (norm > 1e-12f) {
    float inv = 1.0f / norm;
    w_WC *= inv;
    x_WC *= inv;
    y_WC *= inv;
    z_WC *= inv;
  }
  else {
    // 极端异常兜底
    w_WC = 1.0f;
    x_WC = y_WC = z_WC = 0.0f;
  }

  /* ------------------------------------------------
   * Step 6: 输出（底盘 → 世界）
   * ------------------------------------------------ */
  q_chassis2world[0] = w_WC;
  q_chassis2world[1] = x_WC;
  q_chassis2world[2] = y_WC;
  q_chassis2world[3] = z_WC;
  //quaternion_to_euler(q_chassis2world, euler_q);
}

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

// 四元数转欧拉角 (内旋ZYX顺序: 先yaw->再pitch->最后roll)
// 输入: q[4] = {w, x, y, z} 表示物体系到世界系的四元数
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad

void Gimbal::quaternion_to_euler(const float q[4], float euler[3])
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  float sin_pitch = 2.0f * (w * y - x * z);

  // Clamp for numerical safety
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;

  // Gimbal lock check  一瞬间将roll+yaw 全部吸收到yaw
  if (fabsf(sin_pitch) >= 0.999999f) {
    // pitch = ±90°
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);                               // pitch (beta)
    euler[0] = 0.0f;                                                            // roll (theta)
    euler[2] = atan2f(-2.0f * (x * y - w * z), 1.0f - 2.0f * (x * x + z * z));  //yaw = yaw+roll
  }
  else {
    // Normal case
    euler[1] = asinf(sin_pitch);  // pitch (beta)
    euler[0] = atan2f(2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y));
    euler[2] = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z));
  }
}

// 旋转矩阵转欧拉角 (ZYX顺序: yaw->pitch->roll)
// 输入: R[3][3] 表示物体相对地面的旋转矩阵
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad
void Gimbal::rotation_matrix_to_euler(const float R[3][3], float euler[3])
{
  // Pitch (绕Y轴旋转)
  float sin_pitch = -R[2][0];

  // 检查万向锁情况
  if (fabsf(sin_pitch) >= 0.99999f) {
    // 万向锁情况
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);
    euler[0] = 0.0f;                       // 将roll设为0
    euler[2] = atan2f(-R[0][1], R[1][1]);  //
  }
  else {
    euler[1] = asinf(sin_pitch);

    // Roll (绕X轴旋转)
    euler[0] = atan2f(R[2][1], R[2][2]);

    // Yaw (绕Z轴旋转)
    euler[2] = atan2f(R[1][0], R[0][0]);
  }
}

}  // namespace sp
