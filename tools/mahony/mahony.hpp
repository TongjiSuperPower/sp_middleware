#ifndef SP__MAHONY_HPP
#define SP__MAHONY_HPP

namespace sp
{
class Mahony
{
public:
  Mahony(float dt, float kp = 0.5f, float ki = 0.0f);

  float q[4];    // 只读! 四元数顺序为wxyz
  float yaw;     // 只读! 单位: rad
  float pitch;   // 只读! 单位: rad
  float roll;    // 只读! 单位: rad
  float vyaw;    // 只读! 单位: rad/s
  float vpitch;  // 只读! 单位: rad/s
  float vroll;   // 只读! 单位: rad/s

  void update(const float acc[3], const float gyro[3]);
  void update(float ax, float ay, float az, float wx, float wy, float wz);

private:
  const float dt_;
  const float two_kp_;
  const float two_ki_;

  bool inited_;

  float integral_fbx_;
  float integral_fby_;
  float integral_fbz_;

  void init(float ax, float ay, float az);
};

}  // namespace sp

#endif  // SP__MAHONY_HPP