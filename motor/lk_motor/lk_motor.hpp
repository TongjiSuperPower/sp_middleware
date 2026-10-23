#ifndef SP_LK_MOTOR_HPP
#define SP_LK_MOTOR_HPP

#include <cstdint>

namespace sp
{

enum class LK_Motors
{
  MG4005,     // 原装减速比10:1
  MHF9025,    // 原装减速比1:1
  MF7025_28T  // 28匝绕组，原装减速比1:1
};

class LK_Motor
{
public:
  // motor_id: 电机ID，取值1~32
  // motor_type: 电机型号，减速比由型号确定
  // multi_circle: true使用多圈角度，false使用单圈角度
  LK_Motor(uint8_t motor_id, LK_Motors motor_type, bool multi_circle = true);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  float angle = 0;               // 只读! 输出端角度，单位rad
  float multicycle_angle = 0;    // 只读! 输出端多圈角度，单位rad
  float speed = 0;               // 只读! 输出端速度，单位rad/s
  float torque = 0;              // 只读! 输出端估算力矩，单位N·m
  float current = 0;             // 只读! 转矩电流，单位A
  int8_t temperature = 0;        // 只读! 电机温度，单位℃
  float bus_voltage = 0;         // 只读! 母线电压，单位V
  float bus_current = 0;         // 只读! 母线电流，单位A
  uint16_t encoder_raw = 0;      // 只读! 编码器原始值
  uint16_t encoder_raw_max = 0;  // 只读! 已收到的编码器最大值

  uint8_t motor_state = 0x10;  // 只读! 0x00开启，0x10关闭

  // 只读! 各位0正常、1故障，可同时置位
  // bit0低压，bit1高压，bit2驱动过温，bit3电机过温
  // bit4过流，bit5短路，bit6堵转，bit7输入信号丢失超时
  uint8_t error_state = 0;

  // 获取原装减速比
  float ratio() const;

  // 获取电机ID
  uint8_t motor_id() const;

  // 获取电机型号
  LK_Motors motor_type() const;

  // 获取最大发送电流，单位A
  float max_current() const;

  // 获取控制电流原始值上限
  int16_t max_raw() const;

  // 判断是否收到过有效回复
  bool is_open() const;

  // 判断最近100ms内是否收到有效回复
  bool is_alive(uint32_t now_ms) const;

  // 解析本电机的8字节回复，时间单位ms
  void read(const uint8_t * data, uint32_t stamp_ms);

  // 设置输出端目标力矩，单位N·m
  void cmd_torque(float torque);

  // 生成力矩控制帧（0xA1）
  void write_torque(uint8_t * data) const;

  // 设置输出端目标速度，单位rad/s
  void cmd_velocity(float speed);

  // 生成速度控制帧（0xA2）
  void write_velocity(uint8_t * data) const;

  // 设置电机轴单圈角度，单位rad；方向0顺时针、1逆时针
  void cmd_position(float position, uint8_t direction = 0x00);

  // 生成单圈位置控制帧（0xA5）
  void write_position(uint8_t * data) const;

  // 设置输出端角度增量，单位rad
  void cmd_angle_increment(float angle);

  // 生成增量位置控制帧（0xA7）
  void write_angle_increment(uint8_t * data) const;

  // 设置输出端角度增量和最大速度，单位rad、rad/s
  void cmd_angle_increment_speed(float angle, float max_speed);

  // 生成带限速的增量位置控制帧（0xA8）
  void write_angle_increment_speed(uint8_t * data) const;

  // 生成状态1查询帧（0x9A）
  void write_state1(uint8_t * data) const;

  // 生成状态2查询帧（0x9C）
  void write_state2(uint8_t * data) const;

  // 生成清除错误帧（0x9B）
  void write_clear_error(uint8_t * data) const;

  // 生成电机开启帧（0x88）
  void write_enable(uint8_t * data) const;

  // 生成电机关闭帧（0x80）
  void write_disable(uint8_t * data) const;

  // 生成电机停止帧（0x81）
  void write_stop(uint8_t * data) const;

private:
  // 解析状态1：温度、电压、电流和状态标志
  void read_state1(const uint8_t * data);

  // 解析状态2：温度、转矩电流、速度和编码器
  void read_state2(const uint8_t * data);

  const uint8_t motor_id_;
  const LK_Motors motor_type_;
  const bool multi_circle_;

  float ratio_;
  float torque_const_;
  float current_scale_;
  float max_current_;
  float encoder_range_;
  int16_t max_raw_;

  bool has_read_ = false;
  bool encoder_inited_ = false;
  uint32_t last_read_ms_ = 0;

  int32_t circle_ = 0;
  uint16_t last_encoder_ = 0;

  int16_t cmd_torque_raw_ = 0;
  int32_t cmd_velocity_raw_ = 0;
  uint32_t cmd_position_raw_ = 0;
  uint8_t cmd_direction_ = 0;
  int32_t cmd_angle_increment_raw_ = 0;
  int32_t cmd_angle_increment_speed_raw_ = 0;
  uint16_t cmd_max_speed_raw_ = 0;
};

}  // namespace sp

#endif  // SP_LK_MOTOR_HPP
