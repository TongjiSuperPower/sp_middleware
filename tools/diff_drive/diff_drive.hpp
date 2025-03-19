#ifndef SP__DIFF_DRIVE_HPP
#define SP__DIFF_DRIVE_HPP

namespace sp
{
class DiffDrive
{
public:
  DiffDrive(float half_width, float wheel_radius, bool reverse_l, bool reverse_r);

  float v = 0;  // 只读! update()更新结果, 前进速度, 单位: m/s
  float w = 0;  // 只读! update()更新结果, 旋转速度, 单位: m/s

  float speed_l = 0;  // 只读! calc()计算结果, left目标转速, 单位: rad/s
  float speed_r = 0;  // 只读! calc()计算结果, right目标转速, 单位: rad/s

  void calc(float v_set, float w_set);
  void update(float motor_speed_l, float motor_speed_r);

private:
  const float w_;
  const float r_;
  const float sign_l_;
  const float sign_r_;
};

}  // namespace sp

#endif  // SP__DIFF_DRIVE_HPP