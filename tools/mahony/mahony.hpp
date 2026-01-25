#ifndef SP__MAHONY_HPP
#define SP__MAHONY_HPP

namespace sp
{
class Mahony
{
public:
  Mahony(float dt, float kp = 0.5f, float ki = 0.0f);

  float q[4];    // 只读! 四元数顺序为wxyz 该四元数的含义是表示某物体相对于地面的姿态
                 //具体的作用是:将同一个向量从某一个系的坐标值转换到地面系
                 //即 v_ground = q * v_body * q_conjugate
  float yaw;     // 只读! 单位: rad
  float pitch;   // 只读! 单位: rad
  float roll;    // 只读! 单位: rad
  float vyaw;    // 只读! 单位: rad/s
  float vpitch;  // 只读! 单位: rad/s
  float vroll;   // 只读! 单位: rad/s

  float pitch_geom;  // 只读! 单位: rad 范围[-pi, pi]
  float pitch_geom_last;
  float vpitch_geom;  // 只读! 单位: rad/s

  /* *****************************************************************************
  变量说明:
  这里的vyaw, vpitch, vroll的定义是yaw/pitch/roll的微分(dt=1/控制频率1000Hz)
  但是我们计算这三个值的时候不是用欧拉角的微分来计算的, 
  而是另辟蹊径通过陀螺仪得到的角速度来反解的(在数值上是完全相等的)
  
  并且只有当欧拉角E是表示的云台相对于底盘的姿态时,
  vyaw/vpitch/vroll才有物理意义,物理意义是三个电机的角速度反馈值
  
  我们现在实际上有三个欧拉角(或者等价的四元数):
  云台相对于底盘的姿态E_cg,
  云台相对于地面的姿态E_wg,
  底盘相对于地面的姿态E_wc,
  他们三个可以任意知二求一(知道其中两个就能算出第三个)
  单imu无法直接得到E_wc, 但是可以通过E_cg(电机编码值)和E_wg反推得到
  
  双imu就三个都能直接获得

  需要注意的是:上电后在确定底盘相对于地面的初始yaw值的时候 要在mahony的init阶段直接将yaw_relative_angle赋值给底盘系的yaw0(正负自己确定)!!!


********************************************************************/

  void update(const float acc[3], const float gyro[3]);
  void update(
    float ax, float ay, float az, float wx, float wy,
    float wz);  //用于更新物体相对于地面的姿态 四元数和欧拉角以及其微分
  void culculate_yaw_pitch_roll_rates(
    float wx, float wy, float wz, float roll, float pitch, float yaw);
  //用于计算物体相对于某一个系的欧拉角微分,注意要带入相对于该系的欧拉角值

private:
  const float dt_;
  const float two_kp_;
  const float two_ki_;

  bool inited_;

  float integral_fbx_;
  float integral_fby_;
  float integral_fbz_;

  void init(float ax, float ay, float az);
  float g_world[3] = {0.0f, 0.0f, -1.0f};  // 重力加速度在地面系下的向量
  float g_base[3];
};

}  // namespace sp

#endif  // SP__MAHONY_HPP