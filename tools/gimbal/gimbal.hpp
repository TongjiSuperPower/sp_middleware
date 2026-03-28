#ifndef SP__GIMBAL_HPP
#define SP__GIMBAL_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
namespace sp
{

// 滤波器配置结构体，用于在构造时统一设置所有滤波器的 alpha 参数
// alpha 含义：为1时无滤波效果，为0时无更新效果，通常取 0~1 之间的值
struct GimbalFilterConfig
{
  // 电机相对角度低通滤波器 alpha（单IMU/双IMU均使用）
  float yaw_angle = 1.0f;    // yaw  相对角度滤波
  float pitch_angle = 1.0f;  // pitch 相对角度滤波
  float roll_angle = 1.0f;   // roll  相对角度滤波（双IMU使用）

  // 电机反馈速度低通滤波器 alpha（需与目标速度滤波器保持一致，否则速度环会有延迟）
  float yaw_fdb_speed = 1.0f;    // yaw  反馈速度滤波
  float pitch_fdb_speed = 1.0f;  // pitch 反馈速度滤波
  float roll_fdb_speed = 1.0f;   // roll  反馈速度滤波

  // 电机目标速度低通滤波器 alpha（建议与反馈速度滤波器设置相同）
  float yaw_target_speed = 1.0f;    // yaw  目标速度滤波
  float pitch_target_speed = 1.0f;  // pitch 目标速度滤波

  // 电机目标加速度低通滤波器 alpha
  float yaw_target_acc = 1.0f;    // yaw  目标加速度滤波
  float pitch_target_acc = 1.0f;  // pitch 目标加速度滤波
};

class Gimbal
{
public:
  Gimbal(
    float yaw0 = 0.0f, float pitch0 = 0.0f, bool reverse_yaw = false, bool reverse_pitch = false,
    float dt = 1e-3f, float install_roll = 0.0f,
    const GimbalFilterConfig & filter_config = GimbalFilterConfig{});

  //单imu更新函数，输入云台姿态和电机角度
  void update_all_single(
    const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);
  //双imu更新函数，输入云台姿态、底盘姿态
  void update_all_dual(const sp::Mahony & gimbal_imu, sp::Mahony & chassis_imu);

  //双imu特有的函数，用于矫正底盘imu的姿态
  void correct_chassis_imu(sp::Mahony & chassis_imu);

  void update(const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);

  // 通过云台姿态和电机角度，算出底盘在世界坐标系下的姿态四元数和角速度
  void update_q_chassis2world(
    const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
    const float & roll0_ = 0.0f);

  //通过底盘的姿态四元数和云台姿态四元数,算出底盘和云台之间的相对四元数,并将相对四元数转换成欧拉角(等效电机角度)
  void update_q_gimbal2chassis(const sp::Mahony & gimbal_imu, const sp::Mahony & chassis_imu);

  //自瞄:输入云台姿态和视觉发来的目标值;输出云台的目标角度、目标速度和目标加速度
  //键鼠:输入云台姿态和键鼠发来的目标角度和速度,无需输入目标加速度; 输出云台的目标角度和目标速度(只调用此两者就行)
  void calc_all_target(
    const sp::Mahony & gimbal_imu, float yaw_set_in_world = 0.0f, float pitch_set_in_world = 0.0f,
    float vyaw_set_in_world = 0.0f, float vpitch_set_in_world = 0.0f,
    float acc_yaw_set_in_world = 0.0f, float acc_pitch_set_in_world = 0.0f);

  void calc(float yaw_set_in_world, float pitch_set_in_world);

  // 欧拉角(ZYX顺序: yaw->pitch->roll)转四元数
  // 输入: yaw, pitch, roll 的含义为: 机体系从与世界系重合开始, 先绕世界Z轴旋转yaw, 再绕新Y轴旋转pitch, 最后绕新X轴旋转roll
  // 输出: q[4] = {w, x, y, z}, 表示将机体系下向量变换到世界系下的四元数, 即 v_world = q ⊗ v_body ⊗ q*
  static void euler_zyx_to_quaternion(float yaw, float pitch, float roll, float q[4]);

  // 四元数转欧拉角：q[0]=w, q[1]=x, q[2]=y, q[3]=z
  // 返回数组顺序：[0]=roll, [1]=pitch, [2]=yaw，单位：rad
  // 四元数定义为：q = {w, x, y, z},能将body系下的向量换回到world系 例:v_world = q ⊗ v_body ⊗ q*
  // 欧拉角定义:body系相对于world系的欧拉角旋转，按照ZYX顺序，即先绕z轴旋转yaw，再绕y轴旋转pitch，最后绕x轴旋转roll
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

  // 向量叉乘：result = v1 × v2
  // 输入：v1[3], v2[3] 两个三维向量
  // 输出：result[3] 叉乘结果
  static void cross_product(const float v1[3], const float v2[3], float result[3]);

  // 载体系角速度转欧拉角速率  对应T逆
  // 输入：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度,即imu测量到的角速度原始值
  //      euler[3] = {roll, pitch, yaw} 当前欧拉角 相对于A系
  // 输出：euler_rates[3] = {roll_rate, pitch_rate, yaw_rate} 欧拉角微分 相对于A系 相对于什么系就能得到相对于同一个系的欧拉角的微分
  static void transform_omiga_in_body_2_euler_rates(
    const float omiga_in_body[3], const float roll, const float pitch, const float yaw,
    float & vroll, float & vpitch, float & vyaw);

  // 欧拉角速率转载体系角速度 对应T (or Jacobian)
  // 输入：vroll, vpitch, vyaw 欧拉角微分
  //      roll, pitch, yaw 当前欧拉角
  // 输出：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度
  static void transform_euler_rates_2_omiga_in_body(
    const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
    const float yaw, float omiga_in_body[3]);

  // 变换矩阵的微分乘以欧拉角速率  这个函数存在的原因是微分的性质:要对所有变量对t求导数
  // 输入：roll, pitch, yaw 当前欧拉角
  //      vroll, vpitch, vyaw 欧拉角速率
  // 输出：result[3] 变换结果
  static void Transform_matrix_rates_multipy_Euler_rates(
    const float roll, const float pitch, const float yaw, const float vroll, const float vpitch,
    const float vyaw, float result[3]);

  float yaw_fdb_in_joint;    //只读！ 云台yaw轴相对于码盘的反馈角度，单位：rad
  float pitch_fdb_in_joint;  //只读！ 云台pitch轴相对于码盘的反馈角度，单位：rad
  float yaw_set_in_joint;    //只读！ 云台yaw轴相对于码盘的设定角度，单位：rad
  float pitch_set_in_joint;  //只读！ 云台pitch轴相对于码盘的设定角度，单位：rad

  float R_base2world_[3]
                     [3];  //底盘系相对于地面系的旋转矩阵,已经考虑了上电瞬间世界系由云台x轴投影定义

  float dq[4];
  //只读！底盘系相对于地面系的角速度 (在地面系下) 第一个分量是1,后三个分量才是角速度wx,wy,wz
  float q_gimbal2chassis
    [4];  //云台系相对于底盘系的四元数表示 能将云台系的向量转换到底盘系,用于计算电机相对角度
  float q_chassis2world[4];  //底盘系相对于地面系的四元数表示 能将底盘系的向量转换到地面系

  float yaw_rel;    //只读！ 云台yaw轴相对于码盘的反馈角度，单位：rad
  float pitch_rel;  //只读！ 云台pitch轴相对于码盘的反馈角度，单位：rad
  float roll_rel;   //只读！ 云台roll轴相对于码盘的反馈角度，往往是零单位：rad

  float roll_relative_speed;   //只读！ 云台roll轴相对底盘的角速度，单位：rad/s
  float pitch_relative_speed;  //只读！ 云台pitch轴相对底盘的角速度，单位：rad/s
  float yaw_relative_speed;    //只读！ 云台yaw轴相对底盘的角速度，单位：rad/s
  float k_torque[3] = {
    0.0f, 0.0f, 0.0f};  //pitch轴的力矩补偿系数 应将k_torque[2]乘mg得到pitch轴的力矩补偿值

  float yaw_target_relative_angle = 0.0f;    //只读！ 云台yaw轴目标相对角度（相对底盘），单位：rad
  float pitch_target_relative_angle = 0.0f;  //只读！ 云台pitch轴目标相对角度（相对底盘），单位：rad
  float yaw_target_relative_speed = 0.0f;  //只读！ 云台yaw轴目标相对角速度（相对底盘），单位：rad/s
  float pitch_target_relative_speed =
    0.0f;  //只读！ 云台pitch轴目标相对角速度（相对底盘），单位：rad/s
  float yaw_target_relative_acc =
    0.0f;  //只读！ 云台yaw轴目标相对角加速度（相对底盘），单位：rad/s²
  float pitch_target_relative_acc =
    0.0f;  //只读！ 云台pitch轴目标相对角加速度（相对底盘），单位：rad/s²

  float base_yaw_in_world;    //只读！ 底盘系相对于地面系的yaw角，单位：rad
  float base_pitch_in_world;  //只读！ 底盘系相对于地面系的pitch角，单位：rad
  float base_roll_in_world;   //只读！ 底盘系相对于地面系的roll角，单位：rad

private:
  float yaw0_;    //云台yaw轴码盘零点位置，单位：rad
  float pitch0_;  //云台pitch轴码盘零点位置，单位：rad
  float sign_yaw_;
  float sign_pitch_;
  float dt_;                                            //控制周期
  float euler_motor[3];                                 //电机相对于底盘的欧拉角表示
  float q_last_chassis2world[4];                        //底盘系相对于地面系的上次四元数表示
  float install_roll_ = 0.0f;                           //底盘安装roll角,用于双imu矫正底盘姿态
  float install_roll_q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  //底盘安装roll角对应的四元数表示

  float w_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};
  float w_last_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};

  float w_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};
  float w_chassis_in_chassisframe[3] = {0.0f, 0.0f, 0.0f};
  float w_relative[3] = {0.0f, 0.0f, 0.0f};

  float acc_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};
  float acc_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};

  float gun_target_vector[3] = {0.0f, 0.0f, 0.0f};
  float gun_target_vector_in_chassisframe[3] = {0.0f, 0.0f, 0.0f};

  sp::AngleUnwrapper yaw_unwrapper_;  // yaw角度展开器,将输入的yaw电机角度展开
  sp::AngleUnwrapper
    yaw_target_relative_angle_unwrapper;  // yaw电机目标角度展开器,将输出的yaw电机目标角度展开

  //电机反馈角度滤波器
  sp::LowPassFilter yaw_relative_angle_filter;
  sp::LowPassFilter pitch_relative_angle_filter;
  sp::LowPassFilter roll_relative_angle_filter;
  //从最开始要将电机目标速度滤波器与电机反馈速度滤波器设置成相同
  // 电机目标速度滤波器
  sp::LowPassFilter pitch_target_relative_speed_filter;
  sp::LowPassFilter yaw_target_relative_speed_filter;
  // 电机反馈速度滤波器
  sp::LowPassFilter roll_relative_speed_filter;
  sp::LowPassFilter pitch_relative_speed_filter;
  sp::LowPassFilter yaw_relative_speed_filter;
  // 电机目标加速度滤波器
  sp::LowPassFilter pitch_motor_target_acc_filter;
  sp::LowPassFilter yaw_motor_target_acc_filter;
  //这两个滤波器是杨少代码用的滤波器,已经弃用
  sp::LowPassFilter yaw_relative_angle_filter0;
  sp::LowPassFilter pitch_relative_angle_filter0;
  float A_Tdot_Edot[3] = {0.0f, 0.0f, 0.0f};
  float B_acc_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};
  float C_cross_product[3] = {0.0f, 0.0f, 0.0f};
  float D_Jdot_E_Motor_dot[3] = {0.0f, 0.0f, 0.0f};
  float F[3] = {0.0f, 0.0f, 0.0f};
  float Final[3] = {0.0f, 0.0f, 0.0f};
};

}  // namespace sp

#endif