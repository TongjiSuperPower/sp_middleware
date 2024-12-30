#ifndef SP__MOTOR_COMPOSER_HPP
#define SP__MOTOR_COMPOSER_HPP

namespace sp
{
class MotorComposer
{
public:
  // 将两个电机等效为一个电机
  // reverse_l: 左侧电机反向旋转
  // reverse_r: 右侧电机反向旋转
  MotorComposer(bool reverse_l = false, bool reverse_r = true);

  float angle;    // 只读! calc_angle()计算结果, 单位: rad
  float speed;    // 只读! calc_speed()计算结果, 单位: rad/s
  float torque;   // 只读! calc_torque()计算结果, 单位: N·m
  float speed_l;  // 只读! calc_speeds()计算结果, 单位: rad/s
  float speed_r;  // 只读! calc_speeds()计算结果, 单位: rad/s

  // 初始化各电机转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void init_angles(float angle_l, float angle_r);

  // 各电机转角 -> 等效转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void calc_angle(float angle_l, float angle_r);

  // 各电机转速 -> 等效转速
  // speed_l: 左侧电机转速, 单位: rad
  // speed_r: 右侧电机转速, 单位: rad
  void calc_speed(float speed_l, float speed_r);

  // 各电机转矩 -> 等效转矩
  // torque_l: 左侧电机转矩, 单位: N·m
  // torque_r: 右侧电机转矩, 单位: N·m
  void calc_torque(float torque_l, float torque_r);

  // 等效转速 -> 各电机转速
  // speed: 等效转速, 单位: rad/s
  void calc_speeds(float speed);

private:
  const float sign_l_;
  const float sign_r_;

  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__MOTOR_COMPOSER_HPP