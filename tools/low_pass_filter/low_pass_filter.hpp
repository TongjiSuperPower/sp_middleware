#ifndef SP__LOW_PASS_FILTER_HPP
#define SP__LOW_PASS_FILTER_HPP

namespace sp
{
class LowPassFilter
{
public:
  // alpha: 为1时, 无滤波效果; 为0时, 无更新效果
  LowPassFilter(float alpha);

  float out;  // 只读! update()的计算结果

  void update(float value);

private:
  const float alpha_;
  bool inited_;
  float last_;
};

}  // namespace sp

#endif  // SP__LOW_PASS_FILTER_HPP
