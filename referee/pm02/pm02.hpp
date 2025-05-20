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

  UART_HandleTypeDef * huart;  // 只读!

  referee::GameStatus game_status;     // 只读! 0x0001 比赛状态数据
  referee::GameResult game_result;     // 只读! 0x0002 比赛结果数据
  referee::GameRobotHP game_robot_hp;  // 只读! 0x0003 比赛机器人血量数据

  referee::EventData event;                 // 只读! 0x0101 机器人事件数据
  referee::RefereeWarning referee_warning;  // 只读! 0x0104 裁判警告数据
  referee::DartInfo dart_info;              // 只读! 0x0105 飞镖发射相关数据

  referee::RobotStatus robot_status;                  // 只读! 0x0201 机器人状态数据
  referee::PowerHeatData power_heat;                  // 只读! 0x0202 实时底盘缓冲能量和射击热量数据
  referee::RobotPos robot_pos;                        // 只读! 0x0203 机器人位置数据
  referee::Buff buff;                                 // 只读! 0x0204 机器人增益数据
                                                      // 没有0x0205
  referee::HurtData hurt;                             // 只读! 0x0206 伤害状态数据
  referee::ShootData shoot;                           // 只读! 0x0207 实时射击数据
  referee::ProjectileAllowance projectile_allowance;  // 只读! 0x0208 允许发弹量
  referee::RFID_Status rfid_status;                   // 只读! 0x0209 机器人RFID模块状态
  referee::DartClientCmd dart_client_cmd;             // 只读! 0x020A 飞镖选手端指令数据
  referee::GroundRobotPosition ground_robot_pos;      // 只读! 0x020B 地面机器人位置数据
  referee::RadarMarkData radar_mark;                  // 只读! 0x020C 雷达标记进度数据
  referee::SentryInfo sentry_info;                    // 只读! 0x020D 哨兵自主决策信息同步
  referee::RadarInfo radar_info;                      // 只读! 0x020E 雷达自主决策信息同步

  // 0x0301 机器人交互数据 TODO
  // 0x0302 自定义控制器与机器人交互数据 图传链路
  referee::MapCommand map_command;  // 只读! 0x0303 选手端小地图交互数据
  // 0x0304 键鼠遥控数据 图传链路
  // 0x0305 选手端小地图接收雷达数据 选手端接收
  // 0x0306 自定义控制器与选手端交互数据 选手端接收
  // 0x0307 选手端小地图接收哨兵数据 选手端接收
  // 0x0308 选手端小地图接收机器人数据 选手端接收
  // 0x0309 自定义控制器接收机器人数据 图传链路

  void request();
  void update(uint16_t size);
  void send(const uint8_t * data, size_t size);

  // TODO UI

private:
  const bool use_dma_;

  uint8_t buff_[PM02_BUFF_SIZE];
  void update(uint8_t * frame_start, uint16_t size);
};

}  // namespace sp

#endif  // SP__PM02_HPP