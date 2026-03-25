#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

#include "controllers/control_mode.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

class JointControllerBase
{
public:
  virtual ~JointControllerBase() = default;

  float pos = 0;
  float vel = 0;
  float torque_fdb = 0;
  float torque_cmd = 0;

  virtual void cmd(float value) = 0;
};

template <typename MotorType>
class JointMotorController : public JointControllerBase
{
public:
  JointMotorController(
    float mid, float min, float max, bool reverse, MotorType & motor, sp::PID & pid,
    sp::PID & motor_speed_pid, bool feedforward = false);

  float pos;         // 只读! 单位: rad
  float vel;         // 只读! 单位: rad/s
  float torque_fdb;  // 只读! 单位: N·m
  float torque_cmd;  // 只读! 单位: N·m

  void disable();
  void add(float value);

  void cmd(float value);              // pos
  void cmd_v(float value);            // vel
  void set_feedforward(float value);  // feedforward_t
  void cmd_t(float value);            // torque
  void control();
  float feedforward_t_ = 0;

  const float mid_;
  const float min_;
  const float max_;
  const float sign_;
  const float feedforward_;

  MotorType & motor_;

  sp::PID & pid_;
  sp::PID & motor_speed_pid_;

  ControlMode mode_ = ControlMode::DISABLE;
  float set_ = 0;
  float v_set_ = 0;
  float t_set_ = 0;

private:
};

#endif  // JOINT_MOTOR_CONTROLLER_HPP