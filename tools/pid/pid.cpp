#include "pid.hpp"

// some change ...

namespace tools
{
PID::PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha)
: dt_(dt), kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout), alpha_(alpha)
{
  this->out = 0.0f;

  this->data.set = this->data.fdb = 0.0f;
  this->data.pout = this->data.iout = this->data.dout = 0.0f;
  this->data.err[0] = this->data.err[1] = this->data.err[2] = 0.0f;
  this->data.dbuf[0] = this->data.dbuf[1] = this->data.dbuf[2] = 0.0f;
}

void PID::calc(float set, float fdb)
{
  if (set != this->data.set) this->data.iout /= 2.0f;

  // 微分先行
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = (this->data.fdb - fdb) / dt_;

  // 滤波
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1.0f - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = set - fdb;

  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  this->data.pout = kp_ * this->data.err[0];
  // Ki,梯形积分
  this->data.iout += ki_ * (this->data.err[0] + this->data.err[1]) / 2.0f;
  this->data.iout = limit_max(this->data.iout, max_iout_);
  // Kd
  this->data.dout = kd_ * this->data.dbuf[0];

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

}  // namespace tools