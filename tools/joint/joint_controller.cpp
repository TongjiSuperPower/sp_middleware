#include "joint_controller.hpp"

#include "tools/math_tools/math_tools.hpp"

template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
  MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward, bool limited)
: mid_(mid),
  min_(min),
  max_(max),
  min_m_(min_m),
  max_m_(max_m),
  max_v_(max_v),
  sign_(reverse ? -1.0f : 1.0f),
  motor_(motor),
  pid_(pid),
  motor_speed_pid_(motor_speed_pid),
  feedforward_(feedforward),
  limited_(limited)
{
}

template <typename MotorType>
void JointMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  set_ = pos;
}

template <typename MotorType>
void JointMotorController<MotorType>::init()
{
  init_angle_ = motor_.angle;
}

template <typename MotorType>
void JointMotorController<MotorType>::add(float value)
{
  cmd(set_ + value);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd(float value)
{
  mode_ = ControlMode::POSITION;
  set_ = sp::limit_min_max(value, min_, max_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_max()
{
  cmd(max_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_min()
{
  cmd(min_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = sp::limit_max(value, max_v_);
  set_ = this->pos;
}

template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_t(float value, float t_threshold)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = sp::limit_max(value, max_v_);
  set_ = this->pos;
  if (fabs(this->torque_fdb) > t_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
  }
  return fabs(this->torque_fdb) > t_threshold;
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = sign_ * value;
  set_ = this->pos;
  v_set_ = this->vel;
}

template <typename MotorType>
void JointMotorController<MotorType>::set_feedforward(float value)
{
  this->feedforward_t_ = feedforward_ ? value : 0;
}

template <typename MotorType>
void JointMotorController<MotorType>::control()
{
  this->pos = sign_ * motor_.angle - mid_ - init_angle_;
  if (limited_) {
    if (this->pos > max_m_) this->pos -= 2 * sp::SP_PI;
    if (this->pos < min_m_) this->pos += 2 * sp::SP_PI;
  }

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
      pid_.calc(set_, this->pos);
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