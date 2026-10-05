#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// int32 数据域的限幅值, 取 2e9(略小于 INT32_MAX)避免浮点转整数溢出
constexpr float LK_MAX_RAW_I32 = 2.0e9f;

float get_torque_const(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_TORQUE_CONST;
    default:
      return MG4005_TORQUE_CONST;
  }
}

float get_model_ratio(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_P1;
    default:
      return MG4005_P10;
  }
}

float get_max_current(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_MAX_CURRENT;
    default:
      return MG4005_MAX_CURRENT;
  }
}

float get_iq_scale(LK_Motors motor_type)
{
  // 手册 P6: MG 电机 iq 分辨率 (66/4096 A)/LSB; MF 电机 iq 分辨率 (33/4096 A)/LSB
  // MHF 属于 MF 家族(带 H 中空轴), 所以用 MF 的分辨率
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_MF_IQ_SCALE;
    default:
      return LK_MG_IQ_SCALE;
  }
}

int16_t get_max_raw(LK_Motors motor_type)
{
  auto raw = get_max_current(motor_type) / get_iq_scale(motor_type);
  if (raw > 2048.0f) raw = 2048.0f;  // 协议上限
  return static_cast<int16_t>(raw);
}

float get_ecd_range(LK_Motors motor_type)
{
  // 两个型号都按 16bit(65536) 处理:
  //   MHF9025: 实测左拨杆上档转一圈, encoder_raw_max 扫满 65535 ✓
  //   MG4005 : 按原中间件(它就是为 MG4005 写的)的写法 —— encoder/65536、±32768 翻转
  // 手册 P6 那句"14bit 0~16383"是举例说明不同型号位宽, 不代表这两个型号就是 14bit
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_ECD_RANGE_16BIT;
    default:
      return LK_ECD_RANGE_16BIT;
  }
}

LK_Motor::LK_Motor(uint8_t motor_id, LK_Motors motor_type, float ratio, bool multi_circle)
: rx_id(LK_RX_ID_BASE + motor_id),
  tx_id(LK_TX_ID_BASE + motor_id),
  motor_id_(motor_id),
  motor_type_(motor_type),
  ratio_model_(get_model_ratio(motor_type)),
  ratio_added_(ratio),
  ratio_total_(get_model_ratio(motor_type) * ratio),
  torque_const_(get_torque_const(motor_type)),
  iq_scale_(get_iq_scale(motor_type)),
  max_raw_(get_max_raw(motor_type)),
  ecd_range_(get_ecd_range(motor_type)),
  multi_circle_(multi_circle)
{
}

bool LK_Motor::is_open() const { return has_read_; }

bool LK_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

// ---------------------------- 反馈解析 ----------------------------

void LK_Motor::read(const uint8_t * data, uint32_t stamp_ms)
{
  // 能进入本函数的帧都来自本电机的 CAN ID(LK_RX_ID_BASE + motor_id), 因此都算"在线"
  last_read_ms_ = stamp_ms;

  const uint8_t cmd = data[0];

  if (cmd == LK_CMD_STATE1 || cmd == LK_CMD_CLEAR_ERROR) {
    read_state1(data);
  }
  else if (cmd == LK_CMD_STATE2 || (cmd >= LK_CMD_TORQUE && cmd <= LK_CMD_ANGLE_INCREMENT_SPEED)) {
    // 控制指令的反馈帧与0x9C的数据域布局一致
    read_state2(data);
  }
  // 0x80 关闭 / 0x81 停止 / 0x88 使能 的反馈帧不带状态数据, 只用于确认在线
}

void LK_Motor::read_state1(const uint8_t * data)
{
  if (data[0] != LK_CMD_STATE1 && data[0] != LK_CMD_CLEAR_ERROR) return;

  this->temp = static_cast<int8_t>(data[1]);
  this->motor_state = data[6];
  this->error_state = data[7];
}

void LK_Motor::read_state2(const uint8_t * data)
{
  const uint8_t cmd = data[0];
  if (
    cmd != LK_CMD_STATE2 && (cmd < LK_CMD_TORQUE || cmd > LK_CMD_ANGLE_INCREMENT_SPEED)) return;

  // 数据域(减速箱前的电机侧量), 见手册 P6:
  //   data[1]    温度, int8, 1℃/LSB
  //   data[2..3] iq 原始值(int16, 小端); MF 系列 33/4096 A/LSB, MG 系列 66/4096 A/LSB
  //   data[4..5] 转速, int16, 小端, 1dps/LSB
  //   data[6..7] 单圈编码器, uint16, 小端(满量程见 get_ecd_range: 两个型号都按 65536)
  const int16_t iq_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
  const int16_t speed_raw = static_cast<int16_t>((data[5] << 8) | data[4]);
  const uint16_t encoder = static_cast<uint16_t>((data[7] << 8) | data[6]);

  // 首次读取时初始化 last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = encoder;
  }

  // 多圈计数: 过 0 点时 ±满量程 跳变
  const int32_t delta = static_cast<int32_t>(encoder) - static_cast<int32_t>(last_ecd_);
  if (delta > static_cast<int32_t>(ecd_range_ / 2.0f))
    step_--;
  else if (delta < -static_cast<int32_t>(ecd_range_ / 2.0f))
    step_++;
  last_ecd_ = encoder;

  // 电机侧多圈角度 / 单圈角度, 单位: rad
  const float motor_angle_single = encoder / ecd_range_ * 2.0f * SP_PI;
  const float motor_angle =
    motor_angle_single + static_cast<float>(step_) * 2.0f * SP_PI;

  // 更新公有属性(均为减速箱后的输出端量)
  this->temp = static_cast<int8_t>(data[1]);
  this->encoder_raw = encoder;
  if (encoder > encoder_raw_max) this->encoder_raw_max = encoder;
  this->current = iq_raw * iq_scale_;
  this->torque = this->current * torque_const_ * ratio_total_;
  this->speed = speed_raw * SP_PI / 180.0f / ratio_total_;
  this->multicycle_angle = motor_angle / ratio_total_;
  this->angle =
    multi_circle_ ? this->multicycle_angle : limit_angle(motor_angle_single / ratio_total_);
}

// ---------------------------- 指令帧 ----------------------------
// 所有指令帧都按 data[0] 指令字 + data[1..7] 数据 的格式填充完整8字节

void LK_Motor::write_state1(uint8_t * data) const
{
  data[0] = LK_CMD_STATE1;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_state2(uint8_t * data) const
{
  data[0] = LK_CMD_STATE2;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_clear_error(uint8_t * data) const
{
  data[0] = LK_CMD_CLEAR_ERROR;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_open(uint8_t * data) const
{
  data[0] = LK_CMD_TURN_ON;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_turn_on(uint8_t * data) const { write_open(data); }

void LK_Motor::write_turn_off(uint8_t * data) const
{
  data[0] = LK_CMD_TURN_OFF;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_stop(uint8_t * data) const
{
  data[0] = LK_CMD_STOP;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_torque(uint8_t * data) const
{
  // data[4..5] = iqControl(int16, 小端), ±2048 对应 MF ±16.5A / MG ±33A
  data[0] = LK_CMD_TORQUE;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(cmd_raw_ & 0xFF);
  data[5] = static_cast<uint8_t>((cmd_raw_ >> 8) & 0xFF);
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_velocity(uint8_t * data) const
{
  // 手册 P10, 速度闭环控制命令 0xA2:
  //   data[2..3] = iqControl 转矩电流限制(int16, 小端), ±2048 对应 MF ±16.5A / MG ±33A
  //   data[4..7] = speedControl(int32, 小端), 0.01dps/LSB
  // 注意: 这里必须给一个合理的转矩电流限制, 不能留 0, 否则电机可能没有力矩输出
  const auto speed_raw = cmd_velocity_raw_;
  const int16_t iq_limit = max_raw_;

  data[0] = LK_CMD_VELOCITY;
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(iq_limit & 0xFF);
  data[3] = static_cast<uint8_t>((iq_limit >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[5] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((speed_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((speed_raw >> 24) & 0xFF);
}

void LK_Motor::write_position(uint8_t * data) const
{
  // data[1]    = 旋转方向, 0x00 顺时针, 0x01 逆时针
  // data[4..7] = 单圈位置(uint32, 小端), 电机侧 0.01°, 范围 0 ~ 36000
  // cmd_position_ 是输出端"度": 先乘总减速比换成电机侧角度, 再归一化到单圈[0,360)
  float motor_deg = std::fmod(cmd_position_ * ratio_total_, 360.0f);
  if (motor_deg < 0.0f) motor_deg += 360.0f;

  const auto angle_raw = static_cast<uint32_t>(motor_deg * 100.0f);

  data[0] = LK_CMD_POSITION;
  data[1] = cmd_direction_;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write_angle_increment(uint8_t * data) const
{
  // data[4..7] = 角度增量(int32, 小端), 电机侧 0.01°
  // cmd_angle_ 是输出端"度", 乘总减速比得到电机侧角度
  const float raw = cmd_angle_ * ratio_total_ * 100.0f;
  const auto angle_raw = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));

  data[0] = LK_CMD_ANGLE_INCREMENT;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write_angle_increment2(uint8_t * data) const
{
  // data[2..3] = 最大速度(uint16, 小端), 电机侧 1°/s
  // data[4..7] = 角度增量(int32, 小端), 电机侧 0.01°
  const float angle = cmd_angle_ * ratio_total_ * 100.0f;
  const auto angle_raw = static_cast<int32_t>(limit_max(angle, LK_MAX_RAW_I32));

  float speed_dps = cmd_max_speed_ * ratio_total_;
  if (speed_dps < 0.0f) speed_dps = 0.0f;
  if (speed_dps > 65535.0f) speed_dps = 65535.0f;
  const auto speed_raw = static_cast<uint16_t>(speed_dps);

  data[0] = LK_CMD_ANGLE_INCREMENT_SPEED;
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write(uint8_t * data) const
{
  switch (cmd_mode_) {
    case LK_CMD_VELOCITY:
      write_velocity(data);
      break;
    case LK_CMD_POSITION:
      write_position(data);
      break;
    case LK_CMD_ANGLE_INCREMENT:
      write_angle_increment(data);
      break;
    case LK_CMD_ANGLE_INCREMENT_SPEED:
      write_angle_increment2(data);
      break;
    default:
      write_torque(data);
      break;
  }
}

// ---------------------------- 指令缓存 ----------------------------

void LK_Motor::cmd(float torque) { cmd_torque(torque); }

void LK_Motor::cmd_torque(float torque)
{
  // 输出端力矩 -> 电机侧转矩电流 -> iq 原始值(换算系数按系列区分)
  const float current = torque / ratio_total_ / torque_const_;
  float raw = current / iq_scale_;

  if (raw > max_raw_) raw = max_raw_;
  if (raw < -max_raw_) raw = -max_raw_;

  cmd_raw_ = static_cast<int16_t>(raw);
  cmd_mode_ = LK_CMD_TORQUE;
}

void LK_Motor::cmd_velocity(float speed)
{
  // 输出端 rad/s -> 电机侧 0.01°/s
  const float raw = speed * ratio_total_ * 180.0f / SP_PI * 100.0f;
  cmd_velocity_raw_ = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));
  cmd_mode_ = LK_CMD_VELOCITY;
}

void LK_Motor::cmd_position(float position, uint8_t direction)
{
  // position: 输出端 度(单圈), 组帧时会乘减速比并归一化到电机侧的单圈 0~360°
  cmd_position_ = position;
  cmd_direction_ = direction;
  cmd_mode_ = LK_CMD_POSITION;
}

void LK_Motor::cmd_angle_increment(float angle)
{
  // angle: 输出端 度
  cmd_angle_ = angle;
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT;
}

void LK_Motor::cmd_angle_speed(float angle, float max_speed)
{
  // angle: 输出端 度; max_speed: 输出端 度/s
  cmd_angle_ = angle;
  cmd_max_speed_ = max_speed;
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT_SPEED;
}

}  // namespace sp
