#ifndef SP__REFEREE_PROTOCOL_HPP
#define SP__REFEREE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

namespace sp::referee
{
constexpr uint8_t SOF = 0xA5;

constexpr size_t HEAD_LEN = 5;
constexpr size_t CMD_ID_LEN = 2;
constexpr size_t TAIL_LEN = 2;
constexpr size_t DATA_START = HEAD_LEN + CMD_ID_LEN;

enum class CMD_ID : uint16_t
{
  GAME_STATUS = 0x0001,    // 比赛状态数据
  GAME_RESULT = 0x0002,    // 比赛结果数据
  GAME_ROBOT_HP = 0x0003,  // 比赛机器人血量数据

  EVENT_DATA = 0x0101,       // 场地事件数据
  REFEREE_WARNING = 0x0104,  // 裁判警告数据

  ROBOT_STATUS = 0x0201,           // 机器人性能体系数据
  POWER_HEAT_DATA = 0x0202,        // 实时底盘功率和枪口热量数据
  ROBOT_POS = 0x0203,              // 机器人位置数据
  BUFF = 0x0204,                   // 机器人增益数据
  AIR_SUPPORT_DATA = 0x0205,       // 空中支援时间数据
  HURT_DATA = 0x0206,              // 伤害状态数据
  SHOOT_DATA = 0x0207,             // 实时射击数据
  PROJECTILE_ALLOWANCE = 0x0208,   // 允许发弹量
  RFID_STATUS = 0x0209,            // 机器人RFID模块状态
  DART_CLIENT_CMD = 0x020A,        // 飞镖选手端指令数据
  GROUND_ROBOT_POSITION = 0x020B,  // 地面机器人位置数据
  RADAR_MARK_DATA = 0x020C,        // 雷达标记进度数据
  SENTRY_INFO = 0x020D,            // 哨兵自主决策信息同步
  RADAR_INFO = 0x020F,             // 雷达自主决策信息同步

  STUDENT_INTERACTIVE = 0x0301,  // 机器人交互数据
  CUSTOM_ROBOT_DATA = 0x0302,    // 自定义控制器与机器人交互数据
  REMOTE_CONTROL = 0x0304        // 键鼠遥控数据
};

#pragma pack(1)

struct GameStatus
{
  // bit 0-3: 比赛类型
  //       1: RoboMaster机甲大师超级对抗赛
  //       2: RoboMaster机甲大师高校单项赛
  //       3: ICRA RoboMaster高校人工智能挑战赛
  //       4: RoboMaster机甲大师高校联盟赛3V3对抗
  //       5: RoboMaster机甲大师高校联盟赛步兵对抗
  // bit 4-7: 当前比赛阶段
  //       0: 未开始比赛
  //       1: 准备阶段
  //       2: 十五秒裁判系统自检阶段
  //       3: 五秒倒计时
  //       4: 比赛中
  //       5: 比赛结算中
  uint8_t game_type_progress;
  uint16_t stage_remain_time;  // 单位: s
  uint64_t sync_timestamp;  // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
};

struct GameResult
{
  uint8_t winner;  // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
};

struct GameRobotHP
{
  uint16_t red_1_robot_hp;
  uint16_t red_2_robot_hp;
  uint16_t red_3_robot_hp;
  uint16_t red_4_robot_hp;
  uint16_t red_5_robot_hp;
  uint16_t red_7_robot_hp;
  uint16_t red_outpost_hp;
  uint16_t red_base_hp;
  uint16_t blue_1_robot_hp;
  uint16_t blue_2_robot_hp;
  uint16_t blue_3_robot_hp;
  uint16_t blue_4_robot_hp;
  uint16_t blue_5_robot_hp;
  uint16_t blue_7_robot_hp;
  uint16_t blue_outpost_hp;
  uint16_t blue_base_hp;
};

struct RobotStatus
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_hp;
  uint16_t maximum_hp;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  // bit0: gimbal口输出: 0为无输出, 1为24V输出
  // bit1: chassis口输出: 0为无输出, 1为24V输出
  // bit2: shooter口输出: 0为无输出, 1为24V输出
  uint8_t power_management_output;
};

struct PowerHeatData
{
  uint16_t chassis_voltage;  // 单位: mV
  uint16_t chassis_current;  // 单位: mA
  float chassis_power;       // 单位: W
  uint16_t buffer_energy;    // 单位: J
  uint16_t shooter_17mm_1_barrel_heat;
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
};

struct ShootData
{
  uint8_t bullet_type;  // 1: 17mm弹丸, 2: 42mm弹丸
  uint8_t shooter_number;  // 1: 第1个17mm发射机构, 2: 第2个17mm发射机构, 3: 42mm发射机构
  uint8_t launching_frequency;  // 单位: Hz
  float initial_speed;          // 单位: m/s
};

#pragma pack()

}  // namespace sp::referee

#endif  // SP__REFEREE_PROTOCOL_HPP