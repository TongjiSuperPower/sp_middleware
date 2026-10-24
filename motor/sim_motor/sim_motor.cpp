#include "sim_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{

SimMotor::SimMotor(float inertia, float damping, float tmax)
: inertia_(inertia), damping_(damping), tmax_(tmax)
{
}

void SimMotor::cmd(float torque)
{
  cmd_torque_ = sp::limit_min_max(torque, -tmax_, tmax_);
}

bool SimMotor::is_alive(uint32_t now_ms) const
{
  (void)now_ms;
  return true;
}

void SimMotor::set_load_torque(float t) { load_torque_ = t; }

void SimMotor::set_hard_stop(float min_angle, float max_angle)
{
  has_hard_stop_ = true;
  stop_min_ = min_angle;
  stop_max_ = max_angle;
}

void SimMotor::clear_hard_stop() { has_hard_stop_ = false; }

void SimMotor::update(float dt)
{
  // J*qdd = t_cmd - t_load - d*qd
  const float net = cmd_torque_ - load_torque_ - damping_ * speed;
  const float accel = net / inertia_;

  // 半隐式欧拉
  speed += accel * dt;
  angle += speed * dt;

  // 硬限位: 完全塑性碰撞(截断位置并清零冲向限位的速度), 任意步长下稳定.
  // 堵转时电机仍输出指令力矩, 故 cmd_v_until_t 等堵转检测行为与实物一致.
  if (has_hard_stop_) {
    if (angle < stop_min_) {
      angle = stop_min_;
      if (speed < 0) speed = 0;
    }
    else if (angle > stop_max_) {
      angle = stop_max_;
      if (speed > 0) speed = 0;
    }
  }

  torque = cmd_torque_;
}

void SimMotor::set_state(float new_angle, float new_speed)
{
  angle = new_angle;
  speed = new_speed;
}

void SimMotor::set_applied_torque(float t) { torque = t; }

float SimMotor::cmd_torque() const { return cmd_torque_; }

}  // namespace sp
