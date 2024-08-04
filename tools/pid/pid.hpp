#ifndef PID_HPP
#define PID_HPP

#include "tools/math_tools/math_tools.hpp"

namespace tools
{
enum class PIDSubtractMode
{
  LINEAR,
  ANGUlAR
};

class PID
{
public:
  PID(
    float kp, float ki, float kd, float max_out, float max_iout, float alpha = 1.0f,
    PIDSubtractMode mode = PIDSubtractMode::LINEAR);

  PID(
    float pid[3], float max_out, float max_iout, float alpha = 1.0f,
    PIDSubtractMode mode = PIDSubtractMode::LINEAR);

  float out;  // PID的输出值

  void calc(float set, float fdb);

private:
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float alpha_;
  const PIDSubtractMode mode_;

  struct pid_param_t
  {
    float pout, iout, dout;  // PID的输出，P项输出，I项输出，D项输出
    float dbuf[3], err[3];   // D项的滤波器缓冲区，误差缓冲区
    float set, fdb;          // 设定值，反馈值
  } pid_data_;
};

}  // namespace tools

#endif  // PID_HPP