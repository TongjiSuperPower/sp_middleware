#include "pid.hpp"

namespace tools
{
PID::PID(PIDSubtractMode mode, const float pid[3], float max_out, float max_iout, float alpha)
: mode_(mode)
{
  pid_data_.kp = pid[0];
  pid_data_.ki = pid[1];
  pid_data_.kd = pid[2];
  pid_data_.max_out = max_out;
  pid_data_.max_iout = max_iout;
  pid_data_.alpha = alpha;

  pid_out_ = pid_data_.pout = pid_data_.iout = pid_data_.dout = 0.0f;
  pid_data_.dbuf[0] = pid_data_.dbuf[1] = pid_data_.dbuf[2] = 0.0f;
  pid_data_.err[0] = pid_data_.err[1] = pid_data_.err[2] = 0.0f;
  pid_data_.set = pid_data_.fdb = 0.0f;
}

float PID::pid_calc(float set, float fdb)
{
  if (set != pid_data_.set) pid_data_.iout /= 2.0f;

  // 微分先行
  pid_data_.dbuf[2] = pid_data_.dbuf[1];
  pid_data_.dbuf[1] = pid_data_.dbuf[0];
  pid_data_.dbuf[0] = pid_data_.fdb - fdb;
  // 滤波
  pid_data_.dbuf[0] =
    pid_data_.alpha * pid_data_.dbuf[0] + (1.0f - pid_data_.alpha) * pid_data_.dbuf[1];

  pid_data_.err[2] = pid_data_.err[1];
  pid_data_.err[1] = pid_data_.err[0];
  pid_data_.err[0] = (mode_ == PIDSubtractMode::LINEAR) ? set - fdb : limit_angle(set - fdb);

  pid_data_.set = set;
  pid_data_.fdb = fdb;

  // Kp
  pid_data_.pout = pid_data_.kp * pid_data_.err[0];
  // Ki,梯形积分
  pid_data_.iout += pid_data_.ki * (pid_data_.err[0] + pid_data_.err[1]) / 2.0f;
  pid_data_.iout = limit_max(pid_data_.iout, pid_data_.max_iout);
  // Kd
  pid_data_.dout = pid_data_.kd * pid_data_.dbuf[0];

  pid_out_ = pid_data_.pout + pid_data_.iout + pid_data_.dout;
  pid_out_ = limit_max(pid_out_, pid_data_.max_out);

  return pid_out_;
}
}  // namespace tools