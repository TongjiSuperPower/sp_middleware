#ifndef MOTOR__RM_MOTOR_HPP
#define MOTOR__RM_MOTOR_HPP

#include <cstdint>

#include "tools/math_tools/math_tools.hpp"

namespace motor
{
class RM_Motor
{
public:
  RM_Motor(uint8_t motor_id);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data) const;

  float angle() const;
  float speed() const;
  int16_t current_raw() const;

  void cmd_raw(int16_t raw);

protected:
  uint8_t motor_id_;

private:
  bool has_read_;
  uint32_t last_read_ms_;

  int32_t circle_;
  uint16_t angle_ecd_;
  int16_t speed_rpm_;
  int16_t current_raw_;
  int16_t temperate_;

  int16_t cmd_raw_;
};

// -------------------- GM6020 --------------------

constexpr int16_t GM6020_MAX_VOTAGE_RAW = 25000;
constexpr int16_t GM6020_MAX_CURRENT_RAW = 16384;
constexpr float GM6020_RAW_TO_SPEED = (13.33 / 60 * 2 * tools::PI) * 24 / GM6020_MAX_VOTAGE_RAW;
constexpr float GM6020_RAW_TO_TORQUE = 0.741 * 3 / GM6020_MAX_CURRENT_RAW;

class GM6020 : public RM_Motor
{
public:
  GM6020(uint8_t motor_id, bool voltage_ctrl = true);
  uint16_t rx_id() const;
  uint16_t tx_id() const;
  float torque() const;
  void cmd(float speed_or_torque);

private:
  bool voltage_ctrl_;
};

// -------------------- M2006 --------------------

constexpr int16_t M2006_MAX_CURRENT_RAW = 10000;
constexpr float M2006_P36 = 36;
constexpr float M2006_RAW_TO_TORQUE = 0.18 / M2006_P36 * 10 / M2006_MAX_CURRENT_RAW;

class M2006 : public RM_Motor
{
public:
  M2006(uint8_t motor_id);

  uint16_t rx_id() const;
  uint16_t tx_id() const;

  float angle() const;
  float speed() const;
  float torque() const;

  void cmd(float torque);
};

// -------------------- M3508 --------------------

constexpr int16_t M3508_MAX_CURRENT_RAW = 16384;
constexpr float M3508_P19 = 3591.0 / 187.0;
constexpr float M3508_RAW_TO_TORQUE = 0.3 / M3508_P19 * 20 / M3508_MAX_CURRENT_RAW;

class M3508 : public RM_Motor
{
public:
  M3508(uint8_t motor_id, float ratio = M3508_P19);

  uint16_t rx_id() const;
  uint16_t tx_id() const;

  float angle() const;
  float speed() const;
  float torque() const;

  void cmd(float torque);

private:
  float ratio_;
};

}  // namespace motor

#endif  // MOTOR__RM_MOTOR_HPP