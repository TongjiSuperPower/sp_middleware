#include "pid.hpp"

namespace tools
{
PID::PID(
  float kp, float ki, float kd, float max_out, float max_iout, float alpha, PIDSubtractMode mode)
: kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout), alpha_(alpha), mode_(mode)
{
  this->out = pid_data_.pout = pid_data_.iout = pid_data_.dout = 0.0f;
  pid_data_.dbuf[0] = pid_data_.dbuf[1] = pid_data_.dbuf[2] = 0.0f;
  pid_data_.err[0] = pid_data_.err[1] = pid_data_.err[2] = 0.0f;
  pid_data_.set = pid_data_.fdb = 0.0f;
}

PID::PID(float pid[3], float max_out, float max_iout, float alpha, PIDSubtractMode mode)
: PID(pid[0], pid[1], pid[2], max_out, max_iout, alpha, mode)
{
}

void PID::calc(float set, float fdb)
{
  if (set != pid_data_.set) pid_data_.iout /= 2.0f;

  // 微分先行
  pid_data_.dbuf[2] = pid_data_.dbuf[1];
  pid_data_.dbuf[1] = pid_data_.dbuf[0];
  pid_data_.dbuf[0] = pid_data_.fdb - fdb;
  // 滤波
  pid_data_.dbuf[0] = alpha_ * pid_data_.dbuf[0] + (1.0f - alpha_) * pid_data_.dbuf[1];

  pid_data_.err[2] = pid_data_.err[1];
  pid_data_.err[1] = pid_data_.err[0];
  pid_data_.err[0] = (mode_ == PIDSubtractMode::LINEAR) ? set - fdb : limit_angle(set - fdb);

  pid_data_.set = set;
  pid_data_.fdb = fdb;

  // Kp
  pid_data_.pout = kp_ * pid_data_.err[0];
  // Ki,梯形积分
  pid_data_.iout += ki_ * (pid_data_.err[0] + pid_data_.err[1]) / 2.0f;
  pid_data_.iout = limit_max(pid_data_.iout, max_iout_);
  // Kd
  pid_data_.dout = kd_ * pid_data_.dbuf[0];

  this->out = limit_max(pid_data_.pout + pid_data_.iout + pid_data_.dout, max_out_);
}

}  // namespace tools