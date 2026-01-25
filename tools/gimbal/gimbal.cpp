#include "gimbal.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{

Gimbal::Gimbal(float yaw0, float pitch0, bool reverse_yaw, bool reverse_pitch)
: yaw0_(yaw0),
  pitch0_(pitch0),
  sign_yaw_((reverse_yaw) ? -1.0f : 1.0f),
  sign_pitch_((reverse_pitch) ? -1.0f : 1.0f)
{
  this->yaw_fdb_in_joint = 0.0f;
  this->pitch_fdb_in_joint = 0.0f;
  this->yaw_set_in_joint = 0.0f;
  this->pitch_set_in_joint = 0.0f;
  this->base_yaw_in_world = 0.0f;
  this->base_pitch_in_world = 0.0f;
  this->base_roll_in_world = 0.0f;

  // 初始化为单位矩阵
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      this->R_base2world_[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void Gimbal::update(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  //云台相对码盘值计算（正方向使用云台系右手定则）
  float yaw_relative_angle = sign_yaw_ * (yaw_angle - yaw0_);
  float pitch_relative_angle = sign_pitch_ * (pitch_angle - pitch0_);

  yaw_fdb_in_joint = yaw_relative_angle;
  pitch_fdb_in_joint = pitch_relative_angle;

  float sg_p = sinf(gimbal_imu.pitch);
  float cg_p = cosf(gimbal_imu.pitch);
  float sg_r = sinf(gimbal_imu.roll);
  float cg_r = cosf(gimbal_imu.roll);
  float sg_y = sinf(gimbal_imu.yaw);
  float cg_y = cosf(gimbal_imu.yaw);

  float s_rel_y = sinf(yaw_relative_angle);
  float c_rel_y = cosf(yaw_relative_angle);
  float s_rel_p = sinf(pitch_relative_angle);
  float c_rel_p = cosf(pitch_relative_angle);

  float sr_sp = sg_r * sg_p;

  // Row 0
  R_base2world_[0][0] = -(sr_sp * cg_y - sg_y * cg_r) * s_rel_y +
                        (sr_sp * cg_y + sg_r * sg_y) * s_rel_p * c_rel_y +
                        cg_p * cg_y * c_rel_p * c_rel_y;

  R_base2world_[0][1] = (sr_sp * cg_y - sg_y * cg_r) * c_rel_y +
                        (sr_sp * cg_y + sg_r * sg_y) * s_rel_p * s_rel_y +
                        s_rel_y * cg_p * cg_y * c_rel_p;

  R_base2world_[0][2] = (sr_sp * cg_y + sg_r * sg_y) * c_rel_p - s_rel_p * cg_p * cg_y;

  // Row 1
  R_base2world_[1][0] = -(sr_sp * sg_y + cg_r * cg_y) * s_rel_y +
                        (sg_p * sg_y * cg_r - sg_r * cg_y) * s_rel_p * c_rel_y +
                        sg_y * cg_p * c_rel_p * c_rel_y;

  R_base2world_[1][1] = (sr_sp * sg_y + cg_r * cg_y) * c_rel_y +
                        (sg_p * sg_y * cg_r - sg_r * cg_y) * s_rel_p * s_rel_y +
                        sg_y * s_rel_y * cg_p * c_rel_p;

  R_base2world_[1][2] = (sg_p * sg_y * cg_r - sg_r * cg_y) * c_rel_p - sg_y * s_rel_p * cg_p;

  // Row 2
  R_base2world_[2][0] =
    -sg_p * c_rel_p * c_rel_y - sg_r * s_rel_y * cg_p + s_rel_p * cg_p * cg_r * c_rel_y;

  R_base2world_[2][1] =
    -sg_p * s_rel_y * c_rel_p + sg_r * cg_p * c_rel_y + s_rel_p * s_rel_y * cg_p * cg_r;

  R_base2world_[2][2] = sg_p * s_rel_p + cg_p * cg_r * c_rel_p;

  base_yaw_in_world = atan2f(R_base2world_[1][0], R_base2world_[0][0]);
  base_pitch_in_world = asinf(-R_base2world_[2][0]);  // 注意：这里可能需要防 NaN 处理

  // 增加安全性：
  if (R_base2world_[2][0] > 1.0f)
    base_pitch_in_world = -sp::SP_PI / 2;
  else if (R_base2world_[2][0] < -1.0f)
    base_pitch_in_world = sp::SP_PI / 2;

  base_roll_in_world = atan2f(R_base2world_[2][1], R_base2world_[2][2]);
};

void Gimbal::calc(float yaw_set_in_world, float pitch_set_in_world)
{
  float cy = cosf(yaw_set_in_world);
  float sy = sinf(yaw_set_in_world);
  float cp = cosf(pitch_set_in_world);
  float sp_val = sinf(pitch_set_in_world);

  float common_term_x =
    R_base2world_[0][0] * cp * cy + R_base2world_[1][0] * sy * cp - R_base2world_[2][0] * sp_val;

  float common_term_y =
    R_base2world_[0][1] * cp * cy + R_base2world_[1][1] * sy * cp - R_base2world_[2][1] * sp_val;

  yaw_set_in_joint = atan2f(common_term_y, common_term_x);

  float common_term_z =
    R_base2world_[0][2] * cp * cy + R_base2world_[1][2] * sy * cp - R_base2world_[2][2] * sp_val;

  // 增加安全性
  if (common_term_z > 1.0f) common_term_z = 1.0f;
  if (common_term_z < -1.0f) common_term_z = -1.0f;

  pitch_set_in_joint = -asinf(common_term_z);
}

// 四元数坐标变换：v_out = q ⊗ v_in ⊗ q*
// 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
//      v_in[3] = {x, y, z} 在坐标系B中的同一个向量的坐标
// 输出：v_out[3] = {x, y, z} 在坐标系A中的同一个向量的坐标向量
// conjugate_q: 是否对 q 取共轭（true 时使用 q* 而不是 q，相当于反向旋转）
// 注意：这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）
void Gimbal::quaternion_frame_transform(
  const float q[4], const float v_in[3], float v_out[3], bool conjugate_q)
{
  // 根据 conjugate 标志决定四元数的符号
  float w = q[0];
  float x = conjugate_q ? -q[1] : q[1];
  float y = conjugate_q ? -q[2] : q[2];
  float z = conjugate_q ? -q[3] : q[3];

  float vx = v_in[0], vy = v_in[1], vz = v_in[2];

  // 优化的四元数-向量旋转公式（避免构造完整四元数）
  // v_out = v_in + 2 * cross(q_vec, cross(q_vec, v_in) + w * v_in)

  // 第一步：t = cross(q_vec, v_in) = q_vec × v_in
  float tx = y * vz - z * vy;
  float ty = z * vx - x * vz;
  float tz = x * vy - y * vx;

  // 第二步：t = t + w * v_in
  tx += w * vx;
  ty += w * vy;
  tz += w * vz;

  // 第三步：v_out = v_in + 2 * cross(q_vec, t)
  v_out[0] = vx + 2.0f * (y * tz - z * ty);
  v_out[1] = vy + 2.0f * (z * tx - x * tz);
  v_out[2] = vz + 2.0f * (x * ty - y * tx);
}

}  // namespace sp