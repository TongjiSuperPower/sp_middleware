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
private:
  PIDSubtractMode mode_;
  struct pid_param_t
  {
    float kp, ki, kd;         // PID的三个参数
    float pout, iout, dout;   // PID的输出，P项输出，I项输出，D项输出
    float max_out, max_iout;  // PID的输出限制，I项输出限制
    float dbuf[3], err[3];    // D项的滤波器缓冲区，误差缓冲区
    float set, fdb;           // 设定值，反馈值
    float alpha;              // D项滤波器系数
  } pid_data_;

public:
  float pid_out_;  // PID的输出
  PID(PIDSubtractMode mode, const float pid[3], float max_out, float max_iout, float alpha);
  ~PID() {}
  float pid_calc(float set, float fdb);
};

}  // namespace tools

#endif  // PID_HPP