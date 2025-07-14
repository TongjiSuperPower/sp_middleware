#ifndef SP__PID_HPP
#define SP__PID_HPP

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
struct PIDData
{
  float set = 0;  // 设定值
  float fdb = 0;  // 反馈值(feedback)

  float pout = 0;  // P项输出值
  float iout = 0;  // I项输出值
  float dout = 0;  // D项输出值

  float err[3] = {0};   // 误差缓冲区
  float dbuf[3] = {0};  // D项滤波器缓冲区

  float trapezoid = 0;   // 梯形积分
  float dynamic_ki = 0;  // 动态积分
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
  PID(
    float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha = 1,
    bool angular = false, bool dynamic = false);

  float out = 0;  // 只读! PID输出值
  PIDData data;   // 只读! PID计算数据

  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  void calc(float set, float fdb);

  void calc(float set, float fdb, float integral_pause_threshold);

  void calc(float set, float fdb, float set_dot, float fdb_dot);

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float alpha_;
  const bool angular_;  // 是否为角度控制
  const bool dynamic_;  // 是否为变速积分
};

}  // namespace sp

#endif  // SP__PID_HPP