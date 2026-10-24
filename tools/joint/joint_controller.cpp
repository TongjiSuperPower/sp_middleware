#include "joint_controller.hpp"
#include <cmath>

#ifdef HOST_SIM
#include "motor/sim_motor/sim_motor.hpp"
#endif
#include "tools/math_tools/math_tools.hpp"

template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float min, float max, float max_v, bool reverse, MotorType & motor, sp::PID & pid,
  sp::PID & motor_speed_pid, bool feedforward, float pos_filter, float vel_filter)
: min_(min),
  max_(max),
  max_v_(max_v),
  sign_(reverse ? -1.0f : 1.0f),
  motor_(motor),
  pid_(pid),
  motor_speed_pid_(motor_speed_pid),
  feedforward_(feedforward),
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
void JointMotorController<MotorType>::init(float zero)
{
  // pos = sign_ * (motor.angle - init_angle_), zero 就是"读到这里算 0 rad"的那个读数.
  // 纯算术: 不读 motor_, 和当前姿态无关, 上电前/没收到反馈时调用都行。
  init_angle_ = zero;
}

template <typename MotorType>
void JointMotorController<MotorType>::add(float value)
{
  cmd(set_ + value);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd(float value)
{
  if (trajectory_position_) pid_.clear(); // 模式切换后不能沿用旧模式的位置微分历史
  trajectory_position_ = false;
  mode_ = ControlMode::POSITION;
  v_set_ = 0;
  set_ = sp::limit_min_max(value, min_, max_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_trajectory(float position, float velocity)
{
  if (!trajectory_position_ || mode_ != ControlMode::POSITION) pid_.clear();
  trajectory_position_ = true;
  mode_ = ControlMode::POSITION;
  set_ = sp::limit_min_max(position, min_, max_);
  v_set_ = 0;
  // 位置被限位截断时清除全部速度前馈；恰在边界时也禁止朝边界外侧的速度。
  if (std::isfinite(velocity) && position == set_ &&
      !((set_ >= max_ && velocity > 0) || (set_ <= min_ && velocity < 0)))
    v_set_ = sp::limit_max(velocity, max_v_);
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
    cmd(value);
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
  // init_angle_ 和 raw 是同一个坐标 (电机的直接读数), 这里只是把它减掉再翻方向.
  //
  // 差值要折回 (-π, π]: 达妙 MIT 模式下反馈的 pos 是折到 ±PMAX 的绝对值, 零点一旦
  // 靠近 ±PMAX (LF 的零点离 π 只有 0.12 rad), 摇臂往正向走就会过圈, 读数从 +π
  // 跳到 -π; 上电时若已经过圈, 固件根本看不到跨越, 直接算减就整整差一圈.
  // 摇臂行程 (-20°..100°) 远小于一圈, 折圈不会和真实姿态冲突.
  float delta = sign_ * (motor_.angle - init_angle_);
  while (delta > sp::SP_PI) delta -= 2.0f * sp::SP_PI;
  while (delta <= -sp::SP_PI) delta += 2.0f * sp::SP_PI;
  this->pos = delta;
  motor_pos_filter_.update(this->pos);
  this->pos_filtered = motor_pos_filter_.out;
  
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
      if (trajectory_position_)
        pid_.calc(set_, this->pos_filtered, v_set_, this->vel_filtered);
      else
        pid_.calc(set_, this->pos_filtered);
      motor_speed_pid_.calc(sp::limit_max(pid_.out + v_set_, max_v_), this->vel_filtered);
    }
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel_filtered);
    }

    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    motor_.cmd(torque_cmd);
  }
}

#ifdef HOST_SIM
template class JointMotorController<sp::SimMotor>;
#else
template class JointMotorController<sp::DM_Motor>;
template class JointMotorController<sp::RM_Motor>;
#endif
