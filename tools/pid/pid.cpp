#include "pid.hpp"

namespace sp
{
PID::PID(
  float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha, bool angular,
  bool dynamic)
: dt_(dt),
  kp_(kp),
  ki_(ki),
  kd_(kd),
  max_out_(max_out),
  max_iout_(max_iout),
  alpha_(alpha),
  angular_(angular),
  dynamic_(dynamic)
{
}

void PID::calc(float set, float fdb)
{
  // 微分先行
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);

  // 滤波
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));           // 变速积分
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  this->data.dout = kd_ * this->data.dbuf[0] / dt_;

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

void PID::calc(float set, float fdb, float set_dot, float fdb_dot)
{
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  this->data.iout = limit_max(ki_ * this->data.trapezoid + this->data.iout, max_iout_);

  // Kd
  this->data.dout = kd_ * (set_dot - fdb_dot);

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

}  // namespace sp