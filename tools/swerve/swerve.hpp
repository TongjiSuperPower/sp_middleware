#ifndef SP__SWERVE_HPP
#define SP__SWERVE_HPP

namespace sp
{
class Swerve
{
public:
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // invert_pivot: 各舵电机倒置
  Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot = true);

  float speed_lf;  // 只读! calc()计算结果, 左前轮转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, 左后轮转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, 右前轮转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, 右后轮转速, 单位: rad/s

  float angle_lf;  // 只读! calc()计算结果, 左前舵角度, 单位: rad
  float angle_lr;  // 只读! calc()计算结果, 左后舵角度, 单位: rad
  float angle_rf;  // 只读! calc()计算结果, 右前舵角度, 单位: rad
  float angle_rr;  // 只读! calc()计算结果, 右后舵角度, 单位: rad

  // yaw_lf/lr/rf/rr: 各舵电机的初始角度, 注意此时各轮电机输出轴朝向要相同, 而不是相对! 单位: rad
  void init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

  // 底盘速度 -> 各轮转速和各舵角度
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 单位: rad/s, 正方向: 大拇指朝上右手螺旋方向
  // yaw_lf/lr/rf/rr: 各舵当前角度, 单位: rad
  void calc(float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

private:
  const float r_;
  const float l_;
  const float w_;
  const float sign_;

  float yaw_offset_lf_;
  float yaw_offset_lr_;
  float yaw_offset_rf_;
  float yaw_offset_rr_;

  // 各轮速度向量 -> 各轮转速和各舵角度
  void convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed);
};

}  // namespace sp

#endif  // SP__SWERVE_HPP