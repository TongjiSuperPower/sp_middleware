#ifndef SP__LINEAR_INTERP_HPP
#define SP__LINEAR_INTERP_HPP

#include <cstddef>

namespace sp
{
// 基本线性插值函数
// 给定两个点 (x0, y0) 和 (x1, y1)，计算 x 处的 y 值
float linear_interp(float x, float x0, float y0, float x1, float y1);

// 查表线性插值类
// 给定一组已排序的 x-y 数据点，对任意 x 进行线性插值
// 超出范围时返回边界值（截断）
class LinearInterp
{
public:
  // 构造函数
  // x_table: x值数组（必须已排序，从小到大）
  // y_table: 对应的y值数组
  // size: 数组大小
  LinearInterp(const float * x_table, const float * y_table, size_t size);

  float out;  // 只读! calc()的计算结果

  // 计算插值
  // x: 输入的x值
  // 返回: 插值得到的y值（超出范围返回边界值）
  float calc(float x);

private:
  const float * x_table_;
  const float * y_table_;
  const size_t size_;

  // 二分查找，找到 x 所在的区间索引
  size_t binary_search(float x) const;
};

}  // namespace sp

#endif  // SP__LINEAR_INTERP_HPP

