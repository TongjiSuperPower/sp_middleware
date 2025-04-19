#ifndef SP__VT03_HPP
#define SP__VT03_HPP

#include <array>

#include "usart.h"

namespace sp
{

struct __attribute__((packed)) VT03RemoteData
{
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left : 2;
  uint8_t mouse_right : 2;
  uint8_t mouse_middle : 2;
  uint16_t key;
  uint16_t crc16;
};

enum class VT03Mode
{
  C,
  N,
  S
};

struct VT03MouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
  bool middle;
};

struct VT03KeysData
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

class VT03
{
public:
  VT03(UART_HandleTypeDef * huart, bool use_dma = true);
  UART_HandleTypeDef * huart;

  float ch_rh;   // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_rv;   // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lh;   // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_lv;   // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lu;   // 只读! 左上方拨轮, 取值范围: [-1, 1], 左正右负
  bool fn_l;     // 只读! 左自定义按键
  bool fn_r;     // 只读! 右自定义按键
  bool pause;    // 只读! 暂停按键
  bool trigger;  // 只读! 扳机按键

  VT03Mode mode;        // 只读! 档位切换开关
  VT03MouseData mouse;  // 只读! 鼠标数据
  VT03KeysData keys;    // 只读! 键盘数据

  void request();
  void update(uint16_t size);

private:
  const bool use_dma_;

  VT03RemoteData remote_data_;
  std::array<uint8_t, sizeof(VT03RemoteData)> buff_;  // TODO 0x0302 size

  void update_remote();
};

}  // namespace sp

#endif  // SP__VT03_HPP