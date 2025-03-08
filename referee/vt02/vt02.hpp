#ifndef SP__VT02_HPP
#define SP__VT02_HPP

#include "usart.h"

namespace sp
{
constexpr size_t VT02_BUFF_SIZE = 50;

struct __attribute__((packed)) RefereeCustomData
{
  float yaw;
  float roll;
  float pitch;
  float roll2;
  float x;
  float y;
  float z;
  bool button;
  uint8_t reserved;  //保留位
};

struct RefereeMouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
};

struct RefereeKeysData
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

class VT02
{
public:
  VT02(UART_HandleTypeDef * huart, bool use_dma = true);

  RefereeCustomData custom;  // 只读!
  RefereeMouseData mouse;    // 只读!
  RefereeKeysData keys;      // 只读!

  void request();
  void update();

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  uint8_t buff_[VT02_BUFF_SIZE];

  void update_mouse_and_keys();
};

}  // namespace sp

#endif  // SP__VT02_HPP