#ifndef SP__PM02_HPP
#define SP__PM02_HPP

#include "referee/referee_protocol/referee_protocol.hpp"
#include "usart.h"

namespace sp
{
// ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/referee/rm_referee.c#L21
constexpr size_t PM02_BUFF_SIZE = 255;

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
  void update(uint16_t size);

  // TODO UI

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  uint8_t buff_[PM02_BUFF_SIZE];
  void update(uint8_t * frame_start, uint16_t size);
};

}  // namespace sp

#endif  // SP__PM02_HPP