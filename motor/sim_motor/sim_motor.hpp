#ifndef SP__SIM_MOTOR_HPP
#define SP__SIM_MOTOR_HPP

#include <cstdint>

namespace sp
{

/**
 * @brief 仿真电机, 与 DM_Motor/RM_Motor 拥有相同的控制接口(angle/speed/torque/cmd),
 *        可直接作为 JointMotorController<MotorType> 的 MotorType 使用.
 *
 * 两种用法:
 * 1. 单关节自积分(适合 z轴/夹爪/图传/uwb 等独立关节):
 *      motor.cmd(torque); 电机力矩指令
 *      motor.set_load_torque(t); 外部负载(重力等)
 *      motor.update(dt); 按 J*qdd = t_cmd - t_load - d*qd + t_stop 积分
 * 2. 外部耦合积分器(适合机械臂等多体耦合系统):
 *      外部动力学解算 qdd 后调用 set_state(angle, speed) 回写,
 *      并用 set_applied_torque(t) 设置实际执行力矩作为力矩反馈.
 *
 * 力矩反馈语义与真实电机一致: torque = 电机实际输出力矩(限幅后的指令值).
 * 当关节顶到硬限位堵转时, 电机仍输出指令力矩, 因此 cmd_v_until_t 等
 * 堵转检测逻辑在仿真中行为与实物一致.
 */
class SimMotor
{
public:
  // inertia: 折算到电机输出端的等效转动惯量 kg·m^2
  // damping: 粘性阻尼 N·m·s/rad
  // tmax:    力矩限幅 N·m
  SimMotor(float inertia, float damping, float tmax);

  // ---------------- 与 DM_Motor 对齐的接口 ----------------
  float angle = 0;   // rad
  float speed = 0;   // rad/s
  float torque = 0;  // N·m, 实际输出力矩(反馈)

  void cmd(float torque);  // 缓存力矩指令, 单位: N·m
  bool is_alive(uint32_t now_ms) const;

  // ---------------- 仅仿真使用的接口 ----------------
  void set_load_torque(float t);  // 外部负载力矩, 与 cmd 同号为正
  void set_hard_stop(float min_angle, float max_angle);  // 机械硬限位(rad)
  void clear_hard_stop();

  void update(float dt);  // 单关节自积分

  void set_state(float angle, float speed);  // 外部积分器回写状态
  void set_applied_torque(float t);          // 外部积分器设置实际执行力矩
  float cmd_torque() const;                  // 读取限幅后的力矩指令

private:
  const float inertia_;
  const float damping_;
  const float tmax_;

  float cmd_torque_ = 0;
  float load_torque_ = 0;

  bool has_hard_stop_ = false;
  float stop_min_ = 0;
  float stop_max_ = 0;
};

}  // namespace sp

#endif  // SP__SIM_MOTOR_HPP
