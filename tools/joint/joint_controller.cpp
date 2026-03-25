#include "joint_controller.hpp"

#include "tools/math_tools/math_tools.hpp"

template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float mid, float min, float max, bool reverse, MotorType & motor, sp::PID & pid,
  sp::PID & motor_speed_pid, bool feedforward)
: mid_(mid),
  min_(min),
  max_(max),
  sign_(reverse ? -1.0f : 1.0f),
  feedforward_(feedforward),
  motor_(motor),
  pid_(pid),
  motor_speed_pid_(motor_speed_pid)
{
}

template <typename MotorType>
void JointMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  set_ = pos;
}

template <typename MotorType>
void JointMotorController<MotorType>::add(float value)
{
  cmd(sp::limit_angle(set_ + value));
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd(float value)
{
  mode_ = ControlMode::POSITION;
  set_ = value;
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = value;
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = sign_ * value;
  set_ = pos;
}

template <typename MotorType>
void JointMotorController<MotorType>::set_feedforward(float value)
{
  this->feedforward_t_ = feedforward_ ? value : 0;
}

template <typename MotorType>
void JointMotorController<MotorType>::control()
{
  this->pos = sign_ * motor_.angle - mid_;
  if (this->pos > max_) this->pos -= 2 * sp::SP_PI;
  if (this->pos < min_) this->pos += 2 * sp::SP_PI;

  this->vel = sign_ * motor_.speed;
  this->torque_fdb = sign_ * motor_.torque;

  if (mode_ == ControlMode::DISABLE) {
    motor_.cmd(0);
    return;
  }

  if (mode_ == ControlMode::TORQUE) {
    motor_.cmd(t_set_);
  }
  else {
    if (mode_ == ControlMode::POSITION) {
      pid_.calc(set_, sp::limit_angle(pos));
      motor_speed_pid_.calc(pid_.out, this->vel);
    }
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel);
    }

    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    motor_.cmd(torque_cmd);
  }
}

template class JointMotorController<sp::DM_Motor>;
template class JointMotorController<sp::RM_Motor>;