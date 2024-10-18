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

struct DBusMouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
};

struct DBusKeysData
{
  bool w;
  bool s;
  bool a;
  bool d;
  bool shift;
  bool ctrl;
  bool q;
  bool e;
  bool r;
  bool f;
  bool g;
  bool z;
  bool x;
  bool c;
  bool v;
  bool b;
};

class DBus
{
public:
  DBus(UART_HandleTypeDef * huart, bool use_dma = true);

  float stick_rh;  // 只读! 右侧水平, 取值范围: [-1, 1], 右正左负
  float stick_rv;  // 只读! 右侧垂直, 取值范围: [-1, 1], 上正下负
  float stick_lh;  // 只读! 左侧水平, 取值范围: [-1, 1], 右正左负
  float stick_lv;  // 只读! 左侧垂直, 取值范围: [-1, 1], 上正下负
  float stick_lu;  // 只读! 左上拨轮, 取值范围: [-1, 1], 左正右负

  DBusSwitchMode switch_r;  // 只读!
  DBusSwitchMode switch_l;  // 只读!

  DBusMouseData mouse;  // 只读!
  DBusKeysData keys;    // 只读!

  void request();
  void update(uint32_t stamp_ms);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  bool has_read_;
  uint32_t last_read_ms_;

  uint8_t buff_[DBUS_BUFF_SIZE];
};

}  // namespace io

#endif  // IO__DBUS_HPP