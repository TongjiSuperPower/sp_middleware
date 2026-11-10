#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/control_mode.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/pid/pid.hpp"

// MotorType 需提供 public: angle, speed, torque 与 cmd(float)。
// 固件实例化 DM_Motor / RM_Motor; HOST_SIM 实例化 sp::SimMotor。

class JointControllerBase
{
public:
  virtual ~JointControllerBase() = default;

  float pos = 0;
  float vel = 0;
  float pos_filtered = 0;
  float vel_filtered = 0;
  float torque_fdb = 0;
  float torque_cmd = 0;

  virtual void cmd(float value) = 0;
  // 位置指令, 并临时指定这一次走的位置环限速 (rad/s)。不传就沿用构造函数里的
  // JOINT_MAX_V。默认实现忽略限速, 兼容尚未支持的其他关节控制器。
  virtual void cmd(float value, float /*max_v*/) { cmd(value); }
  // 轨迹位置指令（rad）与速度前馈（rad/s）。派生控制器可实现位置/速度协同跟踪；
  // 默认实现只下发位置，兼容尚未实现速度前馈的其他关节控制器。
  virtual void cmd_trajectory(float position, float /*velocity*/) { cmd(position); }
  virtual void disable() = 0;
  virtual void add(float value) = 0;
  // 设定零点: 参数是电机的直接读数 (和 joint/xx/raw 一致), 读到它时 pos == 0.
  // 纯算术, 不读电机, 和当前姿态无关。
  virtual void init(float zero = 0.0f) = 0;
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
    float min, float max, float max_v, bool reverse, MotorType & motor, sp::PID & pid,
    sp::PID & motor_speed_pid, bool feedforward = false, float pos_filter = 1.0f,
    float vel_filter = 1.0f);

  void disable() override;
  void add(float value) override;
  // 设定零点: 之后 pos = sign_ * (motor.angle - zero), zero 与 raw 同一坐标。
  void init(float zero = 0.0f) override;

  void cmd(float value) override;                      // pos
  // 位置指令 + 这一次的位置环限速 (rad/s); 传正数才生效, <=0 沿用构造函数的值。
  void cmd(float value, float max_v) override;
  void cmd_trajectory(float position, float velocity) override;
  void cmd_max() override;                             //pos
  void cmd_min() override;                             //pos
  void cmd_v(float value) override;                    // vel
  bool cmd_v_until_t(float value, float t_threshold);  // vel with torque threshold
  bool cmd_v_until_stuck(float value, float t_threshold, float v_threshold);
  bool cmd_pos_until_t(float value, float t_threshold);  // pos with torque threshold
  void set_feedforward(float value) override;            // feedforward_t
  void cmd_t(float value) override;                      // torque
  void control() override;

  const float min_;
  const float max_;
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
  bool trajectory_position_ = false;
  // 这一次位置指令要用的限速. <=0 表示用构造函数传入的 max_v_.
  float cmd_max_v_ = 0.0f;
  const bool feedforward_;
  const float pos_filter_;
  const float vel_filter_;
  sp::LowPassFilter motor_pos_filter_{pos_filter_};
  sp::LowPassFilter motor_vel_filter_{vel_filter_};
};

#endif  // JOINT_CONTROLLER_HPP
