#ifndef TOOLS_DIFF_GEAR_HPP
#define TOOLS_DIFF_GEAR_HPP

namespace tools
{
class DiffGear
{
public:
  // reverse_l: left反向旋转
  // reverse_r: right反向旋转
  DiffGear(bool reverse_l = false, bool reverse_r = true);

  float v_left;   // 只读! calc()计算结果, left转速, 单位: rad/s
  float v_right;  // 只读! calc()计算结果, right转速, 单位: rad/s

  // 末端转速 -> 各齿轮转速
  // v_pitch: 大拇指朝左，右手螺旋方向转速, 单位: rad/s
  // v_roll: 大拇指朝前，右手螺旋方向转速, 单位: rad/s
  void calc(float v_pitch, float v_roll);

private:
  float sign_l_;
  float sign_r_;
};

}  // namespace tools

#endif  // TOOLS_DIFF_GEAR_HPP