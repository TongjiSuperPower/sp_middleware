#include "rotor_controller.hpp"

#ifdef HOST_SIM
#include "motor/sim_motor/sim_motor.hpp"
#endif
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
template <typename MotorType>
RotorMotorController<MotorType>::RotorMotorController(
  MotorType & motor, bool reverse, const RotorSlot (&slots)[kSlotCount], float vel_filter)
: motor_(motor),
  sign_(reverse ? -1.0f : 1.0f),
  vel_filter_(vel_filter)
{
  for (int i = 0; i < kSlotCount; ++i) slots_[i] = slots[i];
}

template <typename MotorType>
RotorMotorController<MotorType>::RotorMotorController(
  MotorType & motor, bool reverse, sp::PID & pid, sp::RotorFeedforward * feedforward, float max_v,
  float vel_filter)
: motor_(motor),
  sign_(reverse ? -1.0f : 1.0f),
  vel_filter_(vel_filter)
{
  slots_[0] = {&pid, feedforward, max_v};
}

template <typename MotorType>
void RotorMotorController<MotorType>::set_slot(int index, const RotorSlot & s)
{
  if (index < 0 || index >= kSlotCount) return;
  slots_[index] = s;
}

template <typename MotorType>
void RotorMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  v_set_ = 0.0f;
  t_set_ = 0.0f;
}

template <typename MotorType>
void RotorMotorController<MotorType>::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  // 限速取当前槽位的: 每个槽位允许的最高转速可以不一样 (比如转向时压低)。
  const float limit = slots_[slot].max_v;
  v_set_ = limit > 0.0f ? sp::limit_max(value, limit) : value;
}

template <typename MotorType>
void RotorMotorController<MotorType>::cmd_v(float value, int index)
{
  if (index >= 0 && index < kSlotCount) {
    if (index != slot) {
      // 第一次进这个槽位, PID 的微分历史属于上一个槽位的工况, 丢掉重来。
      // 不清的话切槽位那一拍会出现一个假的 D 项尖峰。
      if (slots_[index].pid != nullptr) slots_[index].pid->clear();
      slot = index;
    }
  }
  cmd_v(value);
}

template <typename MotorType>
void RotorMotorController<MotorType>::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = value;
}

template <typename MotorType>
void RotorMotorController<MotorType>::set_feedforward(float value)
{
  this->feedforward_t_ = value;
}

template <typename MotorType>
void RotorMotorController<MotorType>::control()
{
  this->vel = sign_ * motor_.speed;
  motor_vel_filter_.update(this->vel);
  this->vel_filtered = motor_vel_filter_.out;
  this->torque_fdb = sign_ * motor_.torque;

  if (mode_ == ControlMode::DISABLE) {
    torque_cmd = 0.0f;
    ff_torque = 0.0f;
    pid_torque = 0.0f;
    motor_.cmd(0);
    return;
  }

  if (mode_ == ControlMode::TORQUE) {
    torque_cmd = t_set_;
    ff_torque = 0.0f;
    pid_torque = 0.0f;
    motor_.cmd(sign_ * torque_cmd);
    return;
  }

  // 速度环: PID + 槽位前馈 + 外部前馈。
  const RotorSlot & s = slots_[slot];

  ff_torque = 0.0f;
  if (s.feedforward != nullptr) {
    // 前馈按指令转速 (v_set_) 算, 不用反馈。前馈要跟的是"我要它去的地方",
    // 用反馈算的话系统没跟上的部分会被前馈再补一遍, 等于加大了回路增益。
    ff_torque = s.feedforward->calc(v_set_);
  }

  pid_torque = 0.0f;
  if (s.pid != nullptr) {
    s.pid->calc(v_set_, this->vel_filtered);
    pid_torque = s.pid->out;
  }

  torque_cmd = pid_torque + ff_torque + feedforward_t_;
  motor_.cmd(sign_ * torque_cmd);
}

#ifdef HOST_SIM
template class RotorMotorController<sp::SimMotor>;
#else
template class RotorMotorController<sp::DM_Motor>;
template class RotorMotorController<sp::RM_Motor>;
#endif

}  // namespace sp
