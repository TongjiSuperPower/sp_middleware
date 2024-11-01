#ifndef SP__RM_MOTOR_HPP
#define SP__RM_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float M2006_P36 = 36.0f;             // 原装减速箱减速比
constexpr float M3508_P19 = 3591.0f / 187.0f;  // 原装减速箱减速比

constexpr float M2006_P36_MAX_TORQUE = 0.18f * 10;  // N·m
constexpr float M3508_P19_MAX_TORQUE = 0.3f * 20;   // N·m
constexpr float GM6020_MAX_TORQUE = 0.741f * 3;     // N·m

enum class RM_Motors
{
  M2006,    // 无减速箱
  M3508,    // 无减速箱
  GM6020,   // 电流控制
  GM6020_V  // 电压控制
};

class RM_Motor
{
public:
  // motor_id: 电机ID, 取值: 1, 2, ..., 8
  // motor_type: 电机型号， 取值见`RM_Motors`
  // ratio: 减速比
  RM_Motor(uint8_t motor_id, RM_Motors motor_type, float ratio = 1.0f, bool multi_circle = true);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  float angle;          // 只读! 单位: rad
  float speed;          // 只读! 单位: rad/s
  float torque;         // 只读! 单位: N·m
  uint8_t temperature;  // 只读! 单位: 摄氏度

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data) const;

  // 如果非`RM_Motors::GM6020_V`, 单位: N·m
  // 如果是`RM_Motors::GM6020_V`, 单位: V
  void cmd(float value);

private:
  const uint8_t motor_id_;
  const RM_Motors motor_type_;
  const float ratio_;
  const bool multi_circle_;

  bool has_read_;
  uint32_t last_read_ms_;

  int32_t circle_;
  uint16_t last_ecd_;

  int16_t cmd_raw_;
};

}  // namespace sp

#endif  // SP__RM_MOTOR_HPP