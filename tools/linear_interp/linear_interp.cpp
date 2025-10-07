#include "linear_interp.hpp"

namespace sp
{
// 基本线性插值函数
float linear_interp(float x, float x0, float y0, float x1, float y1)
{
  // 处理分母为0的情况
  if (x1 == x0) {
    return y0;
  }

  // 线性插值公式: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

// 查表线性插值类实现
LinearInterp::LinearInterp(const float * x_table, const float * y_table, size_t size)
  : out(0.0f), x_table_(x_table), y_table_(y_table), size_(size)
{
}

float LinearInterp::calc(float x)
{
  // 边界情况处理
  if (size_ == 0) {
    this->out = 0.0f;
    return this->out;
  }

  if (size_ == 1) {
    this->out = y_table_[0];
    return this->out;
  }

  // 处理超出范围的情况：返回边界值（截断）
  if (x <= x_table_[0]) {
    this->out = y_table_[0];
    return this->out;
  }

  if (x >= x_table_[size_ - 1]) {
    this->out = y_table_[size_ - 1];
    return this->out;
  }

  // 在范围内，进行二分查找
  size_t idx = binary_search(x);

  // 线性插值
  this->out = linear_interp(x, x_table_[idx], y_table_[idx], x_table_[idx + 1], y_table_[idx + 1]);

  return this->out;
}

size_t LinearInterp::binary_search(float x) const
{
  size_t left = 0;
  size_t right = size_ - 1;

  // 二分查找，找到满足 x_table_[idx] <= x < x_table_[idx+1] 的 idx
  while (right - left > 1) {
    size_t mid = (left + right) / 2;
    if (x < x_table_[mid]) {
      right = mid;
    }
    else {
      left = mid;
    }
  }

  return left;
}

}  // namespace sp

