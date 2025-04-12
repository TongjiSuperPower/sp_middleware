#ifndef SP__FUZZY_PID_HPP
#define SP__FUZZY_PID_HPP

#include <array>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
struct FuzzyData
{
  float set = 0;  // �趨ֵ
  float fdb = 0;  // ����ֵ(feedback)


  float pout = 0;  // P�����ֵ
  float iout = 0;  // I�����ֵ
  float dout = 0;  // D�����ֵ

  float err[3] = {0};   // ������
  float err_rate = 0;   // ���仯��
  float dbuf[3] = {0};  // D���˲���������

  float trapezoid = 0;  // ���λ���
  float dynamic_ki = 0; // ��̬����
  
  float fuzzy_kp = 0;  // P��ϵ����kp0+��kp��
  float fuzzy_ki = 0;  // I��ϵ����ki0+��ki��
  float fuzzy_kd = 0;  // D��ϵ����kd0+��kd��
};

class FuzzyPID
{
public:
  using RuleTable = std::array<std::array<int, 7>, 7>;

  FuzzyPID(
    float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale, float error_rate_scale,
    float alpha = 1, bool angular = false, bool dynamic = false);

  float out = 0;   // PID���ֵ
  FuzzyData data;  // PID����

  void calc(float set, float fdb);

private:

  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float error_scale_, error_rate_scale_;
  const float alpha_;
  const bool angular_;  // �Ƿ�Ϊ�Ƕȿ���
  const bool dynamic_;  // �Ƿ�Ϊ���ٻ���

  float delta_kp_ = 0;  // P��ϵ��ģ����
  float delta_ki_ = 0;  // I��ϵ��ģ����
  float delta_kd_ = 0;  // D��ϵ��ģ����

  float fuzzy_inference(float error, float error_rate, const RuleTable & rules) const;
  float membership_calc(float x, int set) const;
};

}  // namespace sp

#endif  // SP__FUZZY_PID_HPP