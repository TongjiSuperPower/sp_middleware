#ifndef TOOLS_DIFF_GEAR_HPP
#define TOOLS_DIFF_GEAR_HPP

namespace tools
{
class DiffGear
{
public:
  DiffGear(bool reverse_l = false, bool reverse_r = true);

  float v_left;
  float v_right;

  void calc(float v_pitch, float v_roll);

private:
  float sign_l_;
  float sign_r_;
};

}  // namespace tools

#endif  // TOOLS_DIFF_GEAR_HPP