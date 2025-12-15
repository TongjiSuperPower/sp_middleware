#ifndef SP__GIMBAL_HPP
#define SP__GIMBAL_HPP

#include "tools\low_pass_filter\low_pass_filter.hpp"
#include "tools\mahony\mahony.hpp"
namespace sp
{
class Gimbal
{
public:
  Gimbal(
    float yaw0, float pitch0, bool reverse_yaw = false, bool reverse_yaw0 = false,
    bool reverse_pitch = false, bool reverse_pitch0 = false, float dt = 1e-3f);

  void update(const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);

  void update_q(
    const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
    const float & roll0_ = 0.0f);
  void calc(float yaw_set_in_world, float pitch_set_in_world);

  // 四元数转欧拉角：q[0]=w, q[1]=x, q[2]=y, q[3]=z
  // 返回数组顺序：[0]=roll, [1]=pitch, [2]=yaw，单位：rad
  static void quaternion_to_euler(const float q[4], float euler[3]);

  // 旋转矩阵转欧拉角：R[3][3]表示物体相对地面的旋转矩阵
  // 返回数组顺序：[0]=roll, [1]=pitch, [2]=yaw，单位：rad
  static void rotation_matrix_to_euler(const float R[3][3], float euler[3]);

  // 四元数乘法：q_result = q1 ⊗ q2
  // 输入/输出格式：q[4] = {w, x, y, z}
  // 表示坐标系变换的复合：先 q2 再 q1
  // conjugate_q1: 是否对 q1 取共轭
  // conjugate_q2: 是否对 q2 取共轭
  static void quaternion_multiply(
    const float q1[4], const float q2[4], float q_result[4], bool conjugate_q1 = false,
    bool conjugate_q2 = false);

  // 四元数旋转向量：v_out = q ⊗ v_in ⊗ q*
  // 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
  //      v_in[3] = {x, y, z} 在坐标系A中的向量
  // 输出：v_out[3] = {x, y, z} 在坐标系B中的向量
  // conjugate_q: 是否对 q 取共轭（即使用 q* 而不是 q）
  static void quaternion_frame_transform(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q = false);

  // 载体系角速度转欧拉角速率
  // 输入：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度,即imu测量到的角速度原始值
  //      euler[3] = {roll, pitch, yaw} 当前欧拉角 相对于A系
  // 输出：euler_rates[3] = {roll_rate, pitch_rate, yaw_rate} 欧拉角微分 相对于A系 相对于什么系就能得到相对于同一个系的欧拉角的微分
  static void transform_omiga_in_body_2_euler_rates(
    const float omiga_in_body[3], const float roll, const float pitch, const float yaw,
    float & vroll, float & vpitch, float & vyaw);

  // 欧拉角速率转载体系角速度（上述函数的逆变换）
  // 输入：vroll, vpitch, vyaw 欧拉角微分
  //      roll, pitch, yaw 当前欧拉角
  // 输出：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度
  static void transform_euler_rates_2_omiga_in_body(
    const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
    const float yaw, float omiga_in_body[3]);

  float yaw_fdb_in_joint;    //只读！ 云台yaw轴相对于码盘的反馈角度，单位：rad
  float pitch_fdb_in_joint;  //只读！ 云台pitch轴相对于码盘的反馈角度，单位：rad
  float yaw_set_in_joint;    //只读！ 云台yaw轴相对于码盘的设定角度，单位：rad
  float pitch_set_in_joint;  //只读！ 云台pitch轴相对于码盘的设定角度，单位：rad
  float euler_r[3];
  float euler_q[3];
  float R_base2world_[3]
                     [3];  //底盘系相对于地面系的旋转矩阵,已经考虑了上电瞬间世界系由云台x轴投影定义
  //后续会把旋转矩阵转换成四元数作为一个private变量并且用来计算底盘运动角速度
  float dq[4];  //只读！底盘系相对于地面系的四元数增量，表示载体系下的角速度等效四元数

private:
  float yaw0_;    //云台yaw轴码盘零点位置，单位：rad
  float pitch0_;  //云台pitch轴码盘零点位置，单位：rad
  float sign_yaw_;
  float sign_pitch_;
  float dt_;
  float q_chassis2world[4];       //底盘系相对于地面系的四元数表示
  float q_last_chassis2world[4];  //底盘系相对于地面系的上次四元数表示
  sp::LowPassFilter yaw_relative_angle_filter;
  sp::LowPassFilter pitch_relative_angle_filter;
  sp::LowPassFilter yaw_relative_angle_filter0;
  sp::LowPassFilter pitch_relative_angle_filter0;
};

}  // namespace sp

#endif