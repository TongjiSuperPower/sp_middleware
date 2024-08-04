#ifndef PID_HPP
#define PID_HPP

#include "tools/math_tools/math_tools.hpp"

namespace tools
{
struct PIDData
{
  float set;      // 设定值
  float fdb;      // 反馈值
  float pout;     // P项的输出值
  float iout;     // I项的输出值
  float dout;     // D项的输出值
  float err[3];   // 误差缓冲区
  float dbuf[3];  // D项的滤波器缓冲区
};

class PID
{
public:
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha = 1.0f);

  float out;     // PID的输出值
  PIDData data;  // debug only

  void calc(float set, float fdb);

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float alpha_;
};

}  // namespace tools

#endif  // PID_HPP