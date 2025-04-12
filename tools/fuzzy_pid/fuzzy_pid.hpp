#ifndef SP__FUZZY_PID_HPP
#define SP__FUZZY_PID_HPP

#include <array>
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
struct FuzzyData
{
  float set = 0;  // 设定值
  float fdb = 0;  // 反馈值(feedback)

  float pout = 0;  // P项输出值
  float iout = 0;  // I项输出值
  float dout = 0;  // D项输出值

  float err[3] = {0};   // 误差缓冲区
  float err_rate = 0;   // 误差变化率
  float dbuf[3] = {0};  // D项滤波器缓冲区

  float trapezoid = 0;  // 梯形积分
  float dynamic_ki = 0; // 动态积分
  
  float fuzzy_kp = 0;  // P项系数（kp0+Δkp）
  float fuzzy_ki = 0;  // I项系数（ki0+Δki）
  float fuzzy_kd = 0;  // D项系数（kd0+Δkd）
};

class FuzzyPID
{
public:
  using RuleTable = std::array<std::array<int, 7>, 7>;

  FuzzyPID(
    float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale, float error_rate_scale,
    float alpha = 1, bool angular = false, bool dynamic = false);

  float out = 0;   // PID输出值
  FuzzyData data;  // PID数据

  void calc(float set, float fdb);

private:

  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float error_scale_, error_rate_scale_;
  const float alpha_;
  const bool angular_;  // 是否为角度控制
  const bool dynamic_;  // 是否为变速积分

  float delta_kp_ = 0;  // P项系数模糊量
  float delta_ki_ = 0;  // I项系数模糊量
  float delta_kd_ = 0;  // D项系数模糊量

  // 模糊推理+解模糊
  float fuzzy_inference(float error, float error_rate, const RuleTable & rules) const;
  // 隶属度计算
  float membership_calc(float x, int set) const;
};

}  // namespace sp

#endif  // SP__FUZZY_PID_HPP