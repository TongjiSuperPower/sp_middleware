#include "linear_differentiator.hpp"

namespace sp
{

LinearDifferentiator::LinearDifferentiator(float r, float dt)
: x1(0.0f), x2(0.0f), r_(r), dt_(dt), inited_(false)
{
}

void LinearDifferentiator::init(float v)
{
  x1 = v;
  x2 = 0.0f;
  inited_ = true;
}

void LinearDifferentiator::update(float v)
{
  if (!inited_) {
    init(v);
    return;
  }

  // 离散化线性跟踪微分器差分方程
  float x1_next = x1 + dt_ * x2;
  float x2_next = x2 + dt_ * (-r_ * r_ * (x1 - v) - 2.0f * r_ * x2);

  x1 = x1_next;
  x2 = x2_next;
}

}  // namespace sp