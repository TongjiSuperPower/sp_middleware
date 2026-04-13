#ifndef SP__DBUS_HPP
#define SP__DBUS_HPP

#include <cstdint>
#include "usart.h"

namespace sp
{
constexpr size_t DBUS_BUFF_SIZE = 18;
constexpr uint16_t DBUS_KEY_W_MASK     = 0x0001;
constexpr uint16_t DBUS_KEY_S_MASK     = 0x0002;
constexpr uint16_t DBUS_KEY_A_MASK     = 0x0004;
constexpr uint16_t DBUS_KEY_D_MASK     = 0x0008;
constexpr uint16_t DBUS_KEY_SHIFT_MASK = 0x0010;
constexpr uint16_t DBUS_KEY_CTRL_MASK  = 0x0020;
constexpr uint16_t DBUS_KEY_Q_MASK     = 0x0040;
constexpr uint16_t DBUS_KEY_E_MASK     = 0x0080;
constexpr uint16_t DBUS_KEY_R_MASK     = 0x0100;
constexpr uint16_t DBUS_KEY_F_MASK     = 0x0200;
constexpr uint16_t DBUS_KEY_G_MASK     = 0x0400;
constexpr uint16_t DBUS_KEY_Z_MASK     = 0x0800;
constexpr uint16_t DBUS_KEY_X_MASK     = 0x1000;
constexpr uint16_t DBUS_KEY_C_MASK     = 0x2000;
constexpr uint16_t DBUS_KEY_V_MASK     = 0x4000;
constexpr uint16_t DBUS_KEY_B_MASK     = 0x8000;

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

  float ch_rh;  // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_rv;  // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lh;  // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_lv;  // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lu;  // 只读! 左上方拨轮, 取值范围: [-1, 1], 左正右负

  DBusSwitchMode sw_r;  // 只读! 右三位开关
  DBusSwitchMode sw_l;  // 只读! 左三位开关

  DBusMouseData mouse;  // 只读!
  DBusKeysData keys;    // 只读!
  uint16_t keyboard_value;  // 只读! 键盘16位原始值

  void request();
  void update(uint16_t size, uint32_t stamp_ms);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  bool has_read_;
  uint32_t last_read_ms_;

  uint8_t buff_[DBUS_BUFF_SIZE];
};

}  // namespace sp

#endif  // SP__DBUS_HPP