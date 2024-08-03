#ifndef IO__DBUS_HPP
#define IO__DBUS_HPP

#include "usart.h"

namespace io
{
constexpr size_t DBUS_BUFF_SIZE = 18;

enum class DBusSwitchMode
{
  DOWN,
  MID,
  UP
};

class DBus
{
public:
  DBus(UART_HandleTypeDef * huart, bool use_dma = true);

  float stick_rh;  // [-1, 1] right horizontal
  float stick_rv;  // [-1, 1] right vertical
  float stick_lh;  // [-1, 1] left horizontal
  float stick_lv;  // [-1, 1] left vertical

  DBusSwitchMode switch_r;
  DBusSwitchMode switch_l;

  void start();
  void update(uint32_t stamp_ms);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  bool has_read_;
  uint32_t last_read_ms_;

  uint8_t buff_[DBUS_BUFF_SIZE];

  void request();
};

}  // namespace io

#endif  // IO__DBUS_HPP