#ifndef SP__GIMBAL_HPP
#define SP__GIMBAL_HPP

#include "tools\mahony\mahony.hpp"

namespace sp
{
class Gimbal
{
public:
  Gimbal(float yaw0, float pitch0, bool reverse_yaw = false, bool reverse_pitch = false);

  void update(const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);

  //不能用于自瞄，原因是C版安装精度、上电瞬间的世界系定义误差等会导致计算误差积累，可等双imu版发布
  void calc(float yaw_set_in_world, float pitch_set_in_world);

  // 四元数旋转向量：v_out = q ⊗ v_in ⊗ q*
  // 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
  //      v_in[3] = {x, y, z} 在坐标系A中的向量
  // 输出：v_out[3] = {x, y, z} 在坐标系B中的向量
  // conjugate_q: 是否对 q 取共轭（即使用 q* 而不是 q）
  static void quaternion_frame_transform(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q = false);

  float yaw_fdb_in_joint;    //只读！ 云台yaw轴相对于码盘的反馈角度，单位：rad
  float pitch_fdb_in_joint;  //只读！ 云台pitch轴相对于码盘的反馈角度，单位：rad
  float yaw_set_in_joint;    //只读！ 云台yaw轴相对于码盘的设定角度，单位：rad
  float pitch_set_in_joint;  //只读！ 云台pitch轴相对于码盘的设定角度，单位：rad

  float base_yaw_in_world;    //只读！ 底盘系相对于地面系的yaw角，单位：rad
  float base_pitch_in_world;  //只读！ 底盘系相对于地面系的pitch角，单位：rad
  float base_roll_in_world;   //只读！ 底盘系相对于地面系的roll角，单位：rad

private:
  float yaw0_;    //云台yaw轴码盘零点位置，单位：rad
  float pitch0_;  //云台pitch轴码盘零点位置，单位：rad
  float sign_yaw_;
  float sign_pitch_;

  float R_base2world_[3]
                     [3];  //底盘系相对于地面系的旋转矩阵,已经考虑了上电瞬间世界系由云台x轴投影定义
  //后续会把旋转矩阵转换成四元数作为一个private变量并且用来计算底盘运动角速度
};

}  // namespace sp

#endif