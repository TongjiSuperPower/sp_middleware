#ifndef SP__DIFF_GEAR_HPP
#define SP__DIFF_GEAR_HPP

namespace sp
{
class DiffGear
{
public:
  // ratio: 锥齿轮减速比
  // reverse_l: left反向旋转
  // reverse_r: right反向旋转
  DiffGear(float ratio, bool reverse_l = false, bool reverse_r = true);

  float pitch;    // 只读! calc_end_angle()计算结果, 单位: rad
  float roll;     // 只读! calc_end_angle()计算结果,  单位: rad
  float v_pitch;  // 只读! calc_end_speed()计算结果, 单位: rad/s
  float v_roll;   // 只读! calc_end_speed()计算结果, 单位: rad/s
  float t_pitch;  // 只读! calc_end_torque()计算结果, 单位: N·m
  float t_roll;   // 只读! calc_end_torque()计算结果, 单位: N·m
  float v_left;   // 只读! calc_gear_speed()计算结果, left转速, 单位: rad/s
  float v_right;  // 只读! calc_gear_speed()计算结果, right转速, 单位: rad/s

  // 初始化各齿轮转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void init_gear_angle(float angle_l, float angle_r);

  // 各齿轮转角 -> 末端转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void calc_end_angle(float angle_l, float angle_r);

  // 各齿轮转速 -> 末端转速
  // speed_l: 左侧齿轮转速, 单位: rad/s
  // speed_r: 右侧齿轮转速, 单位: rad/s
  void calc_end_speed(float speed_l, float speed_r);

  // 各齿轮转矩 -> 末端转矩
  // torque_l: 左侧齿轮转矩, 单位: N·m
  // torque_r: 右侧齿轮转矩, 单位: N·m
  void calc_end_torque(float torque_l, float torque_r);

  // 末端转速 -> 各齿轮转速
  // v_pitch: 大拇指朝左，右手螺旋方向转速, 单位: rad/s
  // v_roll: 大拇指朝前，右手螺旋方向转速, 单位: rad/s
  void calc_gear_speed(float v_pitch, float v_roll);

private:
  const float ratio_;
  const float sign_l_;
  const float sign_r_;

  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__DIFF_GEAR_HPP