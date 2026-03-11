#ifndef SP__MAHONY_HPP
#define SP__MAHONY_HPP

namespace sp
{
class Mahony
{
public:
  Mahony(float dt, float kp = 0.5f, float ki = 0.0f);

  // 新增：用于动态修改 kp 和 ki 的接口
  //kp表示对重力的置信度,kp越高则越相信重力的方向,并将其作为pitch和roll的方向
  //kp越低则越相信角速度积分
  void set_kp(float kp);
  void set_ki(float ki);

  float q_last[4];  // 只读! 上次更新后的四元数顺序为wxyz

  float q[4];  // 只读! 四元数顺序为wxyz 该四元数的含义是表示某物体相对于地面的姿态
               //具体的作用是:将同一个向量从某一个系的坐标值转换到地面系
               //即 v_ground = q * v_body * q_conjugate

  float dq[4];  // 只读! 四元数增量 dq = q_last* ⊗ q，表示载体系下的角速度等效四元数
  float w[3];   // 只读! 单位: rad/s 物体相对于地面的角速度 在载体系下的表示

  float yaw;     // 只读! 单位: rad
  float pitch;   // 只读! 单位: rad
  float roll;    // 只读! 单位: rad
  float vyaw;    // 只读! 单位: rad/s
  float vpitch;  // 只读! 单位: rad/s
  float vroll;   // 只读! 单位: rad/s

  float acc_euler[3];  // 只读! 单位: m/s² 由微分欧拉角角速度计算得到的欧拉角的角加速度
  float pitch_geom;    // 只读! 单位: rad 范围[-pi, pi]
  float pitch_geom_last;
  float vpitch_geom;  // 只读! 单位: rad/s

   void update(const float acc[3], const float gyro[3]);
  void update(
    float ax, float ay, float az, float wx, float wy,
    float wz);  //用于更新物体相对于地面的姿态 四元数和欧拉角以及其微分

  void culculate_yaw_pitch_roll_rates(
    float wx, float wy, float wz, float roll, float pitch, float yaw);
  //用于计算物体相对于某一个系的欧拉角微分,注意要带入相对于该系的欧拉角值
private:
  const float dt_;
  float two_kp_;
  float two_ki_;

  bool inited_;

  float integral_fbx_;
  float integral_fby_;
  float integral_fbz_;
  float base_x[3] = {1.0f, 0.0f, 0.0f};  //base系的x轴
  float base_z[3] = {0.0f, 0.0f, 1.0f};  //base系的z轴
  float world_x[3];                      //world系的x轴
  float world_z[3];                      //world系的z轴
  void quaternion_frame_transform_for_mahony(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q);

  void pitch_geom_calc();
  void init(float ax, float ay, float az);
};

}  // namespace sp

#endif  // SP__MAHONY_HPP