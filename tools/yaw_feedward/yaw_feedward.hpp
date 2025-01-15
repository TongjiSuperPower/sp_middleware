#ifndef SP__YAW_FEEDWARD_HPP
#define SP__YAW_FEEDWARD_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"

namespace sp
{
class YawFeedward
{
private:
  const float dt_;       // �������ڣ���λs
  const float inertia_;  // ת����������λkg��m^2
  const float k_ff_;     // ǰ�����棬��λN��m/(rad/s)
  float alpha_set_;      // Ŀ��Ǽ��ٶȣ���λrad/s

  sp::LowPassFilter speed_filter{0.1f};  // �ٶȵ�ͨ�˲���

public:
  float out = 0;  // ������أ���λN��m��ֻ����
  YawFeedward(float dt, float inertia, float k_ff = 1.0f);
  void calc(float speed_set);
};

}  // namespace sp

#endif  // YAW_FEEDWARD_HPP