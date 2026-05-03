#ifndef SP__LINEAR_DIFFERENTIATOR_HPP
#define SP__LINEAR_DIFFERENTIATOR_HPP

namespace sp
{
class LinearDifferentiator
{
public:
  /*
   * @param r 调节带宽参数，值越大跟踪越快，但对噪声的放大也越大
   * @param dt 积分步长，通常为系统的离散时间间隔（如 0.001s）
   */
  LinearDifferentiator(float r, float dt);

  float x1;  // 跟踪的平滑值
  float x2;  // 微分值（速度）

  /*
   * @param v 观测目标输入值
   */
  void update(float v);

  /*
   * @brief 重置内部状态
   */
  void init(float v);

private:
  float r_;
  float dt_;
  bool inited_;
};
}  // namespace sp

#endif  // SP__LINEAR_DIFFERENTIATOR_HPP
