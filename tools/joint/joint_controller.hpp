#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/control_mode.hpp"
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
  virtual void disable() = 0;
  virtual void add(float value) = 0;
  virtual void init() = 0;
  virtual void cmd_max() = 0;                     //pos
  virtual void cmd_min() = 0;                     //pos
  virtual void cmd_v(float value) = 0;            // vel
  virtual void set_feedforward(float value) = 0;  // feedforward_t
  virtual void cmd_t(float value) = 0;            // torque
  virtual void control() = 0;
};

template <typename MotorType>
class JointMotorController : public JointControllerBase
{
public:
  JointMotorController(
    float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
    MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward = false,
    bool limited = true);

  void disable() override;
  void add(float value) override;
  void init() override;

  void cmd(float value) override;                      // pos
  void cmd_max() override;                             //pos
  void cmd_min() override;                             //pos
  void cmd_v(float value) override;                    // vel
  bool cmd_v_until_t(float value, float t_threshold);  // vel with torque threshold
  bool cmd_v_until_stuck(float value, float t_threshold, float v_threshold);
  bool cmd_pos_until_t(float value, float t_threshold);  // pos with torque threshold
  void set_feedforward(float value) override;            // feedforward_t
  void cmd_t(float value) override;                      // torque
  void control() override;

  const float mid_;
  const float min_;
  const float max_;
  const float min_m_;
  const float max_m_;
  const float max_v_;
  const float sign_;

  MotorType & motor_;

  sp::PID & pid_;
  sp::PID & motor_speed_pid_;

  ControlMode mode_ = ControlMode::DISABLE;
  float set_ = 0;
  float v_set_ = 0;
  float t_set_ = 0;
  float feedforward_t_ = 0;
  float init_angle_ = 0.0f;

private:
  const bool feedforward_;
  bool limited_;
};

#endif  // JOINT_CONTROLLER_HPP