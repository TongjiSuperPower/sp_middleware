#ifndef SP__PM02_HPP
#define SP__PM02_HPP

#include "referee/referee_protocol/referee_protocol.hpp"
#include "usart.h"

namespace sp
{
// 数据段最长为127
// ref: RoboMaster 裁判系统串口协议附录 V1.6.3（20240527）.pdf
constexpr size_t PM02_BUFF_SIZE = referee::HEAD_LEN + referee::CMD_ID_LEN + 127 + referee::TAIL_LEN;

class PM02
{
public:
  PM02(UART_HandleTypeDef * huart, bool use_dma = true);

  referee::GameStatus game_status;     // 只读!
  referee::GameResult game_result;     // 只读!
  referee::GameRobotHP game_robot_hp;  // 只读!

  referee::RobotStatus robot_status;  // 只读!
  referee::PowerHeatData power_heat;  // 只读!
  referee::ShootData shoot;           // 只读!

  void request();
  void update();

  // TODO UI

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  uint8_t buff_[PM02_BUFF_SIZE];
};

}  // namespace sp

#endif  // SP__PM02_HPP