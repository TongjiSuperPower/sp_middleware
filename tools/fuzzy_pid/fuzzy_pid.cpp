#include "fuzzy_pid.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

// 模糊论域分级常量值，对应语言变量 NB, NM, NS, ZO, PS, PM, PB
inline static constexpr int NB_ = -3;
inline static constexpr int NM_ = -2;
inline static constexpr int NS_ = -1;
inline static constexpr int ZO_ = 0;
inline static constexpr int PS_ = 1;
inline static constexpr int PM_ = 2;
inline static constexpr int PB_ = 3;

// KP模糊规则表
inline static constexpr FuzzyPID::RuleTable delta_kp_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{PB_, PB_, PM_, PM_, PS_, ZO_, ZO_}},  // NB (e ↓)
  {{PB_, PB_, PM_, PS_, PS_, ZO_, NS_}},  // NM
  {{PM_, PM_, PM_, PS_, ZO_, NS_, NS_}},  // NS
  {{PM_, PM_, PS_, ZO_, NS_, NM_, NM_}},  // ZO
  {{PS_, PS_, ZO_, NS_, NS_, NM_, NM_}},  // PS
  {{PS_, ZO_, NS_, NM_, NM_, NM_, NB_}},  // PM
  {{ZO_, ZO_, NM_, NM_, NM_, NB_, NB_}}   // PB
}};
inline static constexpr FuzzyPID::RuleTable delta_ki_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{NB_, NB_, NM_, NM_, NS_, ZO_, ZO_}},  // NB (e ↓)
  {{NB_, NB_, NM_, NS_, NS_, ZO_, ZO_}},  // NM
  {{NB_, NM_, NS_, NS_, ZO_, PS_, PS_}},  // NS
  {{NM_, NM_, NS_, ZO_, PS_, PM_, PM_}},  // ZO
  {{NM_, NS_, ZO_, PS_, PS_, PM_, PB_}},  // PS
  {{ZO_, ZO_, PS_, PS_, PM_, PB_, PB_}},  // PM
  {{ZO_, ZO_, PS_, PM_, PM_, PB_, PB_}}   // PB
}};
inline static constexpr FuzzyPID::RuleTable delta_kd_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{PS_, NS_, NB_, NB_, NB_, NM_, PS_}},  // NB (e ↓)
  {{PS_, NS_, NB_, NM_, NM_, NS_, ZO_}},  // NM
  {{ZO_, NS_, NM_, NM_, NS_, NS_, ZO_}},  // NS
  {{ZO_, NS_, NS_, NS_, NS_, NS_, ZO_}},  // ZO
  {{ZO_, ZO_, ZO_, ZO_, ZO_, ZO_, ZO_}},  // PS
  {{PB_, NS_, PS_, PS_, PS_, PS_, PB_}},  // PM
  {{PB_, PM_, PM_, PM_, PS_, PS_, PB_}}   // PB
}};

// clang-format off
// 模糊隶属度函数参数
inline static constexpr float membership_params_[7][3] = {
  {-3,-3,-2},    // NB
  {-2,-2,-1},    // NM
  {-1,-1, 0},    // NS
  {-1, 0, 1},    // ZO
  { 0, 1, 1},    // PS
  { 1, 2, 2},    // PM
  { 2, 3, 3}     // PB
};
// clang-format on

FuzzyPID::FuzzyPID(
  float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale,float error_rate_scale,
  float alpha, bool angular, bool dynamic) 
: dt_(dt),
  kp_(kp0),
  ki_(ki0),
  kd_(kd0),
  max_out_(max_out),
  max_iout_(max_iout),
  error_scale_(error_scale),
  error_rate_scale_(error_rate_scale),
  alpha_(alpha),
  angular_(angular),
  dynamic_(dynamic)
{
}

float FuzzyPID::membership_calc(float x, int set) const
{
  const auto & params = membership_params_[static_cast<int>(set)];

  // 三角形隶属度函数
  if (x <= params[0] || x >= params[2]) return 0.0f;
  if (x <= params[1]) 
    return (x - params[0]) / (params[1] - params[0]);
  return (params[2] - x) / (params[2] - params[1]);
}

float FuzzyPID::fuzzy_inference(float error, float error_rate, const RuleTable & rules) const
{
  // 输入归一化到[-3, 3]
  float e = error / error_scale_ * 3.f;
  float ec = error_rate / error_rate_scale_ * 3.f;

  // 限制输入范围
  e = fabs(e) > 3.0f ? (e > 0.0f ? 3.0f : -3.0f) : e;
  ec = fabs(ec) > 3.0f ? (ec > 0.0f ? 3.0f : -3.0f) : ec;
  
  // 计算隶属度
  std::array<float, 7> e_degrees, ec_degrees;
  for (int i = 0; i < 7; ++i) {
    e_degrees[i] = membership_calc(e, static_cast<int>(i));
    ec_degrees[i] = membership_calc(ec, static_cast<int>(i));
  }

  // 规则推理
  float fuzzy_rule = 0.f, weight_sum = 0.f;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      // 计算规则激活强度
      const float w = e_degrees[i] * ec_degrees[j];
      if (w <= 0.0f) continue;
      // 更新输出隶属度
      fuzzy_rule += w * rules[i][j];
      weight_sum += w;
    }
  }

  // 防止除零
  if (weight_sum == 0.0f) {
    weight_sum = 1e-6f;
  }
  // 计算模糊推理结果的加权平均值
  fuzzy_rule /= weight_sum;

  return fuzzy_rule / 3.f;
}

void FuzzyPID::calc(float set, float fdb)
{
  // 微分先行
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);

  // 滤波
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.err_rate = (this->data.err[0] - this->data.err[1]) / dt_;

  this->data.set = set;
  this->data.fdb = fdb;

  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0])); 

  // 假设fuzzy_out的范围是PID参数的1/2
  delta_kp_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kp_rule_) * kp_ / 2.f;
  delta_ki_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_ki_rule_) * (dynamic_ ? this->data.dynamic_ki : ki_) / 2.f;
  delta_kd_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kd_rule_) * kd_ / 2.f;

  this->data.fuzzy_kp = kp_ + delta_kp_;
  this->data.fuzzy_ki = (dynamic_ ? this->data.dynamic_ki : ki_) + delta_ki_;
  this->data.fuzzy_kd = kd_ + delta_kd_;

  this->data.pout = this->data.fuzzy_kp * this->data.err[0];

  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  this->data.iout =
    limit_max(this->data.iout + this->data.fuzzy_ki * this->data.trapezoid, max_iout_);

  this->data.dout = this->data.fuzzy_kd * this->data.dbuf[0];

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}
}  // namespace sp