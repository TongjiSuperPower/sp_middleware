#include "gimbal.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{

Gimbal::Gimbal(float yaw0, float pitch0, bool reverse_yaw, bool reverse_pitch, float dt)
: yaw0_(yaw0),
  pitch0_(pitch0),
  sign_yaw_((reverse_yaw) ? -1.0f : 1.0f),
  sign_pitch_((reverse_pitch) ? -1.0f : 1.0f),
  yaw_relative_angle_filter(0.1f),
  pitch_relative_angle_filter(0.1f),
  yaw_relative_angle_filter0(1.0f),    //弃用
  pitch_relative_angle_filter0(1.0f),  //弃用
  //电机目标速度滤波器,从最开始要将他俩与电机反馈速度滤波器设置成相同
  pitch_target_relative_speed_filter(0.8f),
  yaw_target_relative_speed_filter(0.6f),
  // 电机反馈速度滤波器
  roll_relative_speed_filter(0.6f),
  pitch_relative_speed_filter(0.8f),
  yaw_relative_speed_filter(0.6f),
  //电机目标加速度滤波器
  pitch_motor_target_acc_filter(0.03f),
  yaw_motor_target_acc_filter(0.03f),
  dt_(dt)
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
void Gimbal::update_all_single(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  //更新底盘相对于地面的四元数表示和底盘的角速度
  update_q_chassis2world(gimbal_imu, yaw_angle, pitch_angle);

  //更新底盘的角加速度,并换到云台系表示
  w_last_chassis_in_worldframe[0] = w_chassis_in_worldframe[0];
  w_last_chassis_in_worldframe[1] = w_chassis_in_worldframe[1];
  w_last_chassis_in_worldframe[2] = w_chassis_in_worldframe[2];
  w_chassis_in_worldframe[0] = this->dq[1];
  w_chassis_in_worldframe[1] = this->dq[2];
  w_chassis_in_worldframe[2] = this->dq[3];
  sp::diff_vec3(
    w_chassis_in_worldframe, w_last_chassis_in_worldframe, acc_chassis_in_worldframe, dt_);
  quaternion_frame_transform(
    gimbal_imu.q, acc_chassis_in_worldframe, acc_chassis_in_gimbalframe, true);

  //底盘角速度换到云台系并更新云台相对于底盘的欧拉角变化率,可用作电机角速度反馈值
  quaternion_frame_transform(gimbal_imu.q, w_chassis_in_worldframe, w_chassis_in_gimbalframe, true);
  w_relative[0] = gimbal_imu.w[0] - w_chassis_in_gimbalframe[0];
  w_relative[1] = gimbal_imu.w[1] - w_chassis_in_gimbalframe[1];
  w_relative[2] = gimbal_imu.w[2] - w_chassis_in_gimbalframe[2];
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    w_relative, roll_rel, pitch_rel, yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed);
  // 对电机反馈速度进行滤波
  roll_relative_speed_filter.update(roll_relative_speed);
  roll_relative_speed = roll_relative_speed_filter.out;
  pitch_relative_speed_filter.update(pitch_relative_speed);
  pitch_relative_speed = pitch_relative_speed_filter.out;
  yaw_relative_speed_filter.update(yaw_relative_speed);
  yaw_relative_speed = yaw_relative_speed_filter.out;

  //底盘角速度换到底盘系w_chassis_in_chassisframe可用作小陀螺补偿
  quaternion_frame_transform(
    q_chassis2world, w_chassis_in_worldframe, w_chassis_in_chassisframe, true);

  //重力补偿
  static float g[3] = {0.0f, 0.0f, 1.0f};  //地面系下重力方向
  sp::Gimbal::quaternion_frame_transform(
    gimbal_imu.q, g, this->k_torque,
    true);  //将重力方向转换到云台系下,k_torque[2]可以用来做重力补偿
}

void Gimbal::calc_all_target_single(
  const sp::Mahony & gimbal_imu, float yaw_set_in_world, float pitch_set_in_world,
  float vyaw_set_in_world, float vpitch_set_in_world, float acc_yaw_set_in_world,
  float acc_pitch_set_in_world)
{
  gun_target_vector[0] = cosf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[1] = sinf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[2] = -sinf(pitch_set_in_world);
  quaternion_frame_transform(
    q_chassis2world, gun_target_vector, gun_target_vector_in_chassisframe, true);

  //解算云台相对于底盘目标欧拉角
  yaw_target_relative_angle = this->yaw_target_relative_angle_unwrapper.update(
    atan2f(gun_target_vector_in_chassisframe[1], gun_target_vector_in_chassisframe[0]));

  pitch_target_relative_angle = asinf(-gun_target_vector_in_chassisframe[2]);

  //解算云台相对于底盘目标欧拉角变化率
  pitch_target_relative_speed =
    cosf(this->roll_rel - gimbal_imu.roll) *
      (vpitch_set_in_world - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
       this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll)) +
    sinf(this->roll_rel - gimbal_imu.roll) *
      (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) -
       vyaw_set_in_world * cosf(gimbal_imu.pitch) +
       this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll));
  pitch_target_relative_speed_filter.update(pitch_target_relative_speed);
  pitch_target_relative_speed = pitch_target_relative_speed_filter.out;

  yaw_target_relative_speed =
    (sinf(this->roll_rel - gimbal_imu.roll) *
     (vpitch_set_in_world - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
      this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel) -
    (cosf(this->roll_rel - gimbal_imu.roll) *
     (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) -
      vyaw_set_in_world * cosf(gimbal_imu.pitch) +
      this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel);
  yaw_target_relative_speed_filter.update(yaw_target_relative_speed);
  yaw_target_relative_speed = yaw_target_relative_speed_filter.out;

  //计算A
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, gimbal_imu.vroll, gimbal_imu.vpitch,
    gimbal_imu.vyaw, A_Tdot_Edot);

  //计算B
  B_acc_chassis_in_gimbalframe[0] = acc_chassis_in_gimbalframe[0];
  B_acc_chassis_in_gimbalframe[1] = acc_chassis_in_gimbalframe[1];
  B_acc_chassis_in_gimbalframe[2] = acc_chassis_in_gimbalframe[2];

  //计算C
  sp::Gimbal::cross_product(this->w_chassis_in_gimbalframe, this->w_relative, C_cross_product);

  //计算D
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    this->roll_rel, this->pitch_rel, this->yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed, D_Jdot_E_Motor_dot);

  //合起来作为F,简化运算
  F[0] =
    A_Tdot_Edot[0] - B_acc_chassis_in_gimbalframe[0] - C_cross_product[0] - D_Jdot_E_Motor_dot[0];
  F[1] =
    A_Tdot_Edot[1] - B_acc_chassis_in_gimbalframe[1] - C_cross_product[1] - D_Jdot_E_Motor_dot[1];
  F[2] =
    A_Tdot_Edot[2] - B_acc_chassis_in_gimbalframe[2] - C_cross_product[2] - D_Jdot_E_Motor_dot[2];

  //乘以雅各比矩阵T的逆
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    F, gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, Final[0], Final[1], Final[2]);

  //给Final加上视觉发来的欧拉角期望加速度
  //后续只考虑后两个分量等价于乘了一个S矩阵
  Final[1] += acc_pitch_set_in_world;
  Final[2] += acc_yaw_set_in_world;

  //千呼万唤始出来兄弟们
  //对这个Final的后两个向量求逆就可以了!!!!!!!!!!!
  pitch_target_relative_acc =
    Final[1] * cosf(this->roll_rel - gimbal_imu.roll) -
    Final[2] * cosf(gimbal_imu.pitch) * sinf(this->roll_rel - gimbal_imu.roll);
  pitch_motor_target_acc_filter.update(pitch_target_relative_acc);
  pitch_target_relative_acc = pitch_motor_target_acc_filter.out;

  yaw_target_relative_acc =
    (Final[1] * sinf(this->roll_rel - gimbal_imu.roll) +
     Final[2] * cosf(gimbal_imu.pitch) * cosf(this->roll_rel - gimbal_imu.roll)) /
    cosf(this->pitch_rel);
  yaw_motor_target_acc_filter.update(yaw_target_relative_acc);
  yaw_target_relative_acc = yaw_motor_target_acc_filter.out;
}

void Gimbal::update(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  //云台相对码盘值计算（正方向使用云台系右手定则）
  float yaw_relative_angle = sign_yaw_ * sp::limit_angle(yaw_angle - yaw0_);
  float pitch_relative_angle = sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);

  // 使用低通滤波器对角度进行滤波
  yaw_relative_angle_filter0.update(yaw_relative_angle);
  pitch_relative_angle_filter0.update(pitch_relative_angle);

  yaw_relative_angle = yaw_relative_angle_filter0.out;
  pitch_relative_angle = pitch_relative_angle_filter0.out;

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

void Gimbal::update_q_chassis2world(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
  const float & roll0_)
{
  /* ------------------------------------------------
   * Step 0: 编码器角度 → 云台相对底盘欧拉角
   * ------------------------------------------------ */
  yaw_rel = sign_yaw_ *
            this->yaw_unwrapper_.update(
              sp::limit_angle(yaw_angle - yaw0_));  //这里要展开limit_angle,否则带来的跳变会让w炸掉
  pitch_rel =
    sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);  //限位原因无所谓不需要展开limit_angle
  roll_rel = roll0_;

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
  q_last_chassis2world[0] = q_chassis2world[0];
  q_last_chassis2world[1] = q_chassis2world[1];
  q_last_chassis2world[2] = q_chassis2world[2];
  q_last_chassis2world[3] = q_chassis2world[3];

  q_chassis2world[0] = w_WC;
  q_chassis2world[1] = x_WC;
  q_chassis2world[2] = y_WC;
  q_chassis2world[3] = z_WC;

  //quaternion_to_euler(q_chassis2world, euler_q);

  //现在下面这个dq 是等价于在地面系下的底盘w
  quaternion_multiply(q_chassis2world, q_last_chassis2world, dq, false, true);
  //这个dq一定不能归一化, 因为他就不是单位四元数

  //使用泰勒展开近似四元数增量表示的角速度(必须保证他在1ms内是小量)
  this->dq[0] *= 2.0f / dt_;
  this->dq[1] *= 2.0f / dt_;
  this->dq[2] *= 2.0f / dt_;
  this->dq[3] *= 2.0f / dt_;
}

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

// 四元数乘法：q_result = q1 ⊗ q2
// 输入/输出格式：q[4] = {w, x, y, z}
// 物理意义：坐标系变换的复合，先应用 q2，再应用 q1
// 例如：q_A_C = q_A_B ⊗ q_B_C（先从C到B，再从B到A）
// conjugate_q1: 是否对 q1 取共轭 (q1* = [w, -x, -y, -z])
// conjugate_q2: 是否对 q2 取共轭 (q2* = [w, -x, -y, -z])
void Gimbal::quaternion_multiply(
  const float q1[4], const float q2[4], float q_result[4], bool conjugate_q1, bool conjugate_q2)
{
  // 根据 conjugate 标志决定符号
  float w1 = q1[0];
  float x1 = conjugate_q1 ? -q1[1] : q1[1];
  float y1 = conjugate_q1 ? -q1[2] : q1[2];
  float z1 = conjugate_q1 ? -q1[3] : q1[3];

  float w2 = q2[0];
  float x2 = conjugate_q2 ? -q2[1] : q2[1];
  float y2 = conjugate_q2 ? -q2[2] : q2[2];
  float z2 = conjugate_q2 ? -q2[3] : q2[3];

  // Hamilton 四元数乘法公式
  q_result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;  // w
  q_result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;  // x
  q_result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;  // y
  q_result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;  // z
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

void Gimbal::cross_product(const float v1[3], const float v2[3], float result[3])
{
  // 向量叉乘公式：result = v1 × v2
  // | i    j    k   |
  // | v1x  v1y  v1z |
  // | v2x  v2y  v2z |
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void Gimbal::transform_omiga_in_body_2_euler_rates(
  const float omiga_in_body[3], const float roll, const float pitch, const float yaw, float & vroll,
  float & vpitch, float & vyaw)
{
  vroll = omiga_in_body[0] + omiga_in_body[1] * std::sin(roll) * std::tan(pitch) +
          omiga_in_body[2] * std::cos(roll) * std::tan(pitch);

  vpitch = omiga_in_body[1] * std::cos(roll) - omiga_in_body[2] * std::sin(roll);

  vyaw = omiga_in_body[1] * std::sin(roll) / std::cos(pitch) +
         omiga_in_body[2] * std::cos(roll) / std::cos(pitch);
}

void Gimbal::transform_euler_rates_2_omiga_in_body(
  const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
  const float yaw, float omiga_in_body[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告

  omiga_in_body[0] = vroll - vyaw * std::sin(pitch);

  omiga_in_body[1] = vpitch * std::cos(roll) + vyaw * std::sin(roll) * std::cos(pitch);

  omiga_in_body[2] = -vpitch * std::sin(roll) + vyaw * std::cos(roll) * std::cos(pitch);
}

void Gimbal::Transform_matrix_rates_multipy_Euler_rates(
  const float roll, const float pitch, const float yaw, const float vroll, const float vpitch,
  const float vyaw, float result[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告
  result[0] = -cosf(pitch) * vpitch * (vyaw);
  result[1] = -sinf(roll) * vroll * (vpitch) +
              (cosf(roll) * cosf(pitch) * vroll - sinf(roll) * sinf(pitch) * vpitch) * (vyaw);
  result[2] = -cosf(roll) * vroll * (vpitch) +
              (-cosf(roll) * sinf(pitch) * vpitch - sinf(roll) * cosf(pitch) * vroll) * (vyaw);
}

}  // namespace sp
