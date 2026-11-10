#ifndef SP__ROTOR_CONTROLLER_HPP
#define SP__ROTOR_CONTROLLER_HPP

#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/control_mode.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/pid/pid.hpp"
#include "tools/rotor_feedforward/rotor_feedforward.hpp"

// MotorType 需提供 public: speed, torque 与 cmd(float)。
// 固件实例化 DM_Motor / RM_Motor; HOST_SIM 实例化 sp::SimMotor。
//
// 和 JointMotorController 的分工: 那个以位置环为主 (关节要跟轨迹), 这个以速度环
// 为主 (轮子/转子要跟转速), 并多一层可变参数槽位。
//
// 上电后 mode_ 是 DISABLE (不出力矩)。速度环是它的常用模式, 用 cmd_v() 进入;
// 要一直按住某个速度就每拍调一次, 不要只调一次 —— 这个类不做指令保活。

namespace sp
{
// 一个参数槽位: 速度环 PID + 可选的前馈模型 + 这个槽位自己的指令限速。
//
// 为什么做成槽位: 同一个转子在不同工况下要用的参数不一样 (空载/带载、平地/上坡、
// 转向/直行), 与其在运行中改 PID 内部状态, 不如准备几套现成的、按需切换。
// 每个槽位的 PID 和前馈各自保留自己的历史, 切回去还是接着上次算。
struct RotorSlot
{
  sp::PID * pid = nullptr;                       // 速度环, 必填
  sp::RotorFeedforward * feedforward = nullptr;  // 前馈模型, 可空 (空则不加前馈)
  float max_v = 0.0f;                            // 速度指令限幅, rad/s; <=0 表示不限
};

class RotorControllerBase
{
public:
  static constexpr int kSlotCount = 4;

  virtual ~RotorControllerBase() = default;

  float vel = 0;           // 只读! 反馈转速, rad/s (已按 reverse 翻成车体系)
  float vel_filtered = 0;  // 只读! 滤波后转速, rad/s
  float torque_fdb = 0;    // 只读! 反馈力矩, N·m (车体系)
  float torque_cmd = 0;    // 只读! 这一拍下发力矩, N·m (车体系)
  float ff_torque = 0;     // 只读! 这一拍前馈贡献的力矩, N·m
  float pid_torque = 0;    // 只读! 这一拍速度环贡献的力矩, N·m
  int slot = 0;            // 只读! 当前槽位

  virtual void disable() = 0;
  // 速度指令 (rad/s), 沿用当前槽位 (初始是 0 号)
  virtual void cmd_v(float value) = 0;
  // 速度指令, 并切到指定槽位
  virtual void cmd_v(float value, int index) = 0;
  // 直接给力矩 (N·m, 车体系), 绕过速度环和他的前馈
  virtual void cmd_t(float value) = 0;
  // 额外的力矩前馈 (N·m, 车体系), 叠加在速度环之上。
  // 和槽位里那个 RotorFeedforward 不是一回事: 那个由目标转速算出来, 这个是外部给的
  // (已知的重力项、摩擦补偿等)。不设就一直是 0。
  virtual void set_feedforward(float value) = 0;
  virtual void control() = 0;
};

template <typename MotorType>
class RotorMotorController : public RotorControllerBase
{
public:
  // motor:     被控电机
  // reverse:    电机装反时为 true; 之后所有接口都用"车体系"(正 = 车往前走),
  //             翻方向在内部做
  // slots:      四个参数槽位, 见 RotorSlot
  // vel_filter: 转速低通系数, 1 = 不滤波
  RotorMotorController(
    MotorType & motor, bool reverse, const RotorSlot (&slots)[kSlotCount],
    float vel_filter = 1.0f);

  // 常用情形: 只有一套速度环 (+ 可选前馈)。这两个对象直接装到 0 号槽位, 其余槽位
  // 留空 (pid 为空时该槽位不出力), 所以 cmd_v(value) 和 cmd_v(value, 0) 等价。
  //
  // feedforward 传 nullptr 表示不加前馈; max_v <= 0 表示不限速。
  RotorMotorController(
    MotorType & motor, bool reverse, sp::PID & pid, sp::RotorFeedforward * feedforward = nullptr,
    float max_v = 0.0f, float vel_filter = 1.0f);

  void disable() override;
  void cmd_v(float value) override;
  void cmd_v(float value, int index) override;
  void cmd_t(float value) override;
  void set_feedforward(float value) override;
  void control() override;

  // 在线换某个槽位的配置 (调 max_v、换 PID 等)。越界忽略。
  void set_slot(int index, const RotorSlot & s);

  MotorType & motor_;
  const float sign_;  // reverse ? -1 : +1

private:
  RotorSlot slots_[kSlotCount];
  ControlMode mode_ = ControlMode::DISABLE;
  float v_set_ = 0.0f;
  float t_set_ = 0.0f;
  float feedforward_t_ = 0.0f;
  const float vel_filter_;
  sp::LowPassFilter motor_vel_filter_{vel_filter_};
};

}  // namespace sp

#endif  // SP__ROTOR_CONTROLLER_HPP
