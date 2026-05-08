#include "joint_controller.hpp"

#include "tools/math_tools/math_tools.hpp"

template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
  MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward, bool limited,
  float pos_filter, float vel_filter)
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
  limited_(limited),
  pos_filter_(pos_filter),
  vel_filter_(vel_filter)
{
}

template <typename MotorType>
void JointMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  set_ = pos;
  v_set_ = 0;
  t_set_ = 0;
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
  v_set_ = 0;
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
  if (fabs(this->torque_fdb) > t_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_stuck(
  float value, float t_threshold, float v_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold && fabs(this->vel) < v_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

template <typename MotorType>
bool JointMotorController<MotorType>::cmd_pos_until_t(float value, float t_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::POSITION;
    v_set_ = 0;
    set_ = sp::limit_min_max(value, min_, max_);
    return false;
  }
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
  motor_pos_filter_.update(this->pos);
  this->pos_filtered = motor_pos_filter_.out;
  if (limited_) {
    if (this->pos > max_m_) this->pos -= 2 * sp::SP_PI;
    if (this->pos < min_m_) this->pos += 2 * sp::SP_PI;
  }

  this->vel = sign_ * motor_.speed;
  motor_vel_filter_.update(this->vel);
  this->vel_filtered = motor_vel_filter_.out;
  this->torque_fdb = sign_ * motor_.torque;

  if (mode_ == ControlMode::DISABLE) {
    torque_cmd = 0.0f;
    motor_.cmd(0);
    return;
  }

  if (mode_ == ControlMode::TORQUE) {
    torque_cmd = t_set_;
    motor_.cmd(torque_cmd);
  }
  else {
    if (mode_ == ControlMode::POSITION) {
      pid_.calc(set_, this->pos_filtered);
      motor_speed_pid_.calc(pid_.out, this->vel_filtered);
    }
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel_filtered);
    }

    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    motor_.cmd(torque_cmd);
  }
}

template class JointMotorController<sp::DM_Motor>;
template class JointMotorController<sp::RM_Motor>;