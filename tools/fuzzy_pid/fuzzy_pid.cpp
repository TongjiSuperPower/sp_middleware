#include "fuzzy_pid.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

// ģ������ּ�����ֵ����Ӧ���Ա��� NB, NM, NS, ZO, PS, PM, PB
inline static constexpr int NB_ = -3;
inline static constexpr int NM_ = -2;
inline static constexpr int NS_ = -1;
inline static constexpr int ZO_ = 0;
inline static constexpr int PS_ = 1;
inline static constexpr int PM_ = 2;
inline static constexpr int PB_ = 3;

// KPģ�������
inline static constexpr FuzzyPID::RuleTable delta_kp_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de ��)
  {{PB_, PB_, PM_, PM_, PS_, ZO_, ZO_}},  // NB (e ��)
  {{PB_, PB_, PM_, PS_, PS_, ZO_, NS_}},  // NM
  {{PM_, PM_, PM_, PS_, ZO_, NS_, NS_}},  // NS
  {{PM_, PM_, PS_, ZO_, NS_, NM_, NM_}},  // ZO
  {{PS_, PS_, ZO_, NS_, NS_, NM_, NM_}},  // PS
  {{PS_, ZO_, NS_, NM_, NM_, NM_, NB_}},  // PM
  {{ZO_, ZO_, NM_, NM_, NM_, NB_, NB_}}   // PB
}};
inline static constexpr FuzzyPID::RuleTable delta_ki_rule_ = {{
  {{NB_, NB_, NM_, NM_, NS_, ZO_, ZO_}},  // NB (e ��)
  {{NB_, NB_, NM_, NS_, NS_, ZO_, ZO_}},  // NM
  {{NB_, NM_, NS_, NS_, ZO_, PS_, PS_}},  // NS
  {{NM_, NM_, NS_, ZO_, PS_, PM_, PM_}},  // ZO
  {{NM_, NS_, ZO_, PS_, PS_, PM_, PB_}},  // PS
  {{ZO_, ZO_, PS_, PS_, PM_, PB_, PB_}},  // PM
  {{ZO_, ZO_, PS_, PM_, PM_, PB_, PB_}}   // PB
}};
inline static constexpr FuzzyPID::RuleTable delta_kd_rule_ = {{
  {{PS_, NS_, NB_, NB_, NB_, NM_, PS_}},  // NB (e ��)
  {{PS_, NS_, NB_, NM_, NM_, NS_, ZO_}},  // NM
  {{ZO_, NS_, NM_, NM_, NS_, NS_, ZO_}},  // NS
  {{ZO_, NS_, NS_, NS_, NS_, NS_, ZO_}},  // ZO
  {{ZO_, ZO_, ZO_, ZO_, ZO_, ZO_, ZO_}},  // PS
  {{PB_, NS_, PS_, PS_, PS_, PS_, PB_}},  // PM
  {{PB_, PM_, PM_, PM_, PS_, PS_, PB_}}   // PB
}};

// ģ�������Ⱥ�������
inline static constexpr float membership_params_[7][3] = {
  {-1.0, -1.0, -0.666},    // NB
  {-1.0, -0.666, -0.333},  // NM
  {-0.666, -0.333, 0.0},   // NS
  {-0.333, 0.0, 0.333},    // ZO
  {0.0, 0.333, 0.666},     // PS
  {0.333, 0.666, 1.0},     // PM
  {0.666, 1.0, 1.0}};      // PB

FuzzyPID::FuzzyPID(
  float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale,float error_rate_scale,
  float alpha, bool angular)
: dt_(dt),
  kp_(kp0),
  ki_(ki0),
  kd_(kd0),
  max_out_(max_out),
  max_iout_(max_iout),
  error_scale_(error_scale),
  error_rate_scale_(error_rate_scale_),
  alpha_(alpha),
  angular_(angular)
{
}

float FuzzyPID::membership_calc(float x, int set) const
{
  const auto & params = membership_params_[static_cast<int>(set)];

  if (x <= params[0]) return 0.0f;
  if (x >= params[2]) return 0.0f;
  if (x <= params[1]) return (x - params[0]) / (params[1] - params[0]);
  return (params[2] - x) / (params[2] - params[1]);
}

float FuzzyPID::fuzzy_inference(float error, float error_rate, const RuleTable & rules) const
{
  // �����һ��
  const float e = error / error_scale_;
  const float ec = error_rate / error_rate_scale_;

  // ����������
  std::array<float, 7> e_degrees, ec_degrees;
  for (int i = 0; i < 7; ++i) {
    e_degrees[i] = membership_calc(e, static_cast<int>(i));
    ec_degrees[i] = membership_calc(ec, static_cast<int>(i));
  }

  // ��������
  float fuzzy_rule = 0.f, weight_sum = 0.f;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      // ������򼤻�ǿ��
      const float w = e_degrees[i] * ec_degrees[j];
      if (w <= 0.0f) continue;
      // �������������
      fuzzy_rule += w * rules[i][j];
      weight_sum += w;
    }
  }

  // ��ֹ����
  if (weight_sum == 0.0f) {
    weight_sum = 1e-6f;
  }
  // ����ģ���������ļ�Ȩƽ��ֵ
  fuzzy_rule /= weight_sum;

  return fuzzy_rule / 3.f;
}

void FuzzyPID::calc(float set, float fdb)
{
  // ΢������
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);

  // �˲�
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.err_rate = (this->data.err[0] - this->data.err[1]) / dt_;

  this->data.set = set;
  this->data.fdb = fdb;

  // ����fuzzy_out�ķ�Χ��PID������1/2
  delta_kp_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kp_rule_) * kp_ / 2.f;
  delta_ki_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_ki_rule_) * ki_ / 2.f;
  delta_kd_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kd_rule_) * kd_ / 2.f;

  this->data.fuzzy_kp = kp_ + delta_kp_;
  this->data.fuzzy_ki = ki_ + delta_ki_;
  this->data.fuzzy_kd = kd_ + delta_kd_;

  this->data.pout = this->data.fuzzy_kp * this->data.err[0];

  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // ���λ���
  this->data.iout =
    limit_max(this->data.iout + this->data.fuzzy_ki * this->data.trapezoid, max_iout_);

  this->data.dout = this->data.fuzzy_kd * this->data.dbuf[0];

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}
}  // namespace sp