#ifndef _PID_HPP
#define _PID_HPP

#include <cmath>

#include "cstdint"

namespace tools
{
//区分pos模式和delta模式
enum class PIDMode
{
  POSITION = 0,
  DELTA
};

class PID
{
private:
  struct pid_param_t
  {
    PIDMode mode;             // PID模式
    float kp, ki, kd;         // PID的三个参数
    float pout, iout, dout;   // PID的输出，P项输出，I项输出，D项输出
    float max_out, max_iout;  // PID的输出限制，I项输出限制
    float dbuf[3], err[3];    // D项的滤波器缓冲区，误差缓冲区
    float set, fdb;           // 设定值，反馈值
    float alpha;              // D项滤波器系数
  } pid_data_;

  // 限制输入值在最大值和最小值之间
  inline void limitMax(float & input, float max)
  {
    if (input > max)
      input = max;
    else if (input < -max)
      input = -max;
  }

public:
  float pid_out_;  // PID的输出
  PID(PIDMode mode, const float pid[3], float max_out, float max_iout, float alpha);
  ~PID() {}
  float pid_calc(float set, float fdb);
};

}  // namespace tools

#endif  // _PID_HPP