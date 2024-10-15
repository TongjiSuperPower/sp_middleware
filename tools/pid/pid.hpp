#ifndef PID_HPP
#define PID_HPP

#include "tools/math_tools/math_tools.hpp"

namespace tools
{
struct PIDData
{
  float set;  // 设定值
  float fdb;  // 反馈值(feedback)

  float speed_set;  // 原外环输出值
  // float speed_fdb;  // 外环输出值

  float pout;     // P项输出值
  float iout;     // I项输出值
  float dout;     // D项输出值
  float err[3];   // 误差缓冲区
  float dbuf[3];  // D项滤波器缓冲区
};

class PID
{
public:
  // dt: 控制周期, 单位: s
  // kp: P项系数
  // ki: I项系数
  // kd: D项系数
  // max_out: PID最大输出值
  // max_iout I项最大输出值
  // alpha: D项滤波系数, alpha=1时不滤波
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha = 1.0f);

  float out;  // 只读! PID输出值
  float speed_fdb;
  PIDData data;  // 只读! PID计算数据

  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  void calc(float set, float fdb);
  void calc_swerve(float set, float fdb, float speed_set);
  int8_t reversal;

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float alpha_;
};

}  // namespace tools

#endif  // PID_HPP