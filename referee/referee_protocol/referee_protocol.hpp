#ifndef SP__REFEREE_PROTOCOL_HPP
#define SP__REFEREE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

namespace sp::referee::robot_id
{
constexpr uint8_t RED_HERO = 1;
constexpr uint8_t RED_ENGINEER = 2;
constexpr uint8_t RED_STANDARD_3 = 3;
constexpr uint8_t RED_STANDARD_4 = 4;
constexpr uint8_t RED_STANDARD_5 = 5;
constexpr uint8_t RED_AERIAL = 6;
constexpr uint8_t RED_SENTRY = 7;
constexpr uint8_t RED_RADAR = 9;
constexpr uint8_t RED_OUTPOST = 10;
constexpr uint8_t RED_BASE = 11;
constexpr uint8_t BLUE_HERO = 101;
constexpr uint8_t BLUE_ENGINEER = 102;
constexpr uint8_t BLUE_STANDARD_3 = 103;
constexpr uint8_t BLUE_STANDARD_4 = 104;
constexpr uint8_t BLUE_STANDARD_5 = 105;
constexpr uint8_t BLUE_AERIAL = 106;
constexpr uint8_t BLUE_SENTRY = 107;
constexpr uint8_t BLUE_RADAR = 109;
constexpr uint8_t BLUE_OUTPOST = 110;
constexpr uint8_t BLUE_BASE = 111;
}  // namespace sp::referee::robot_id

namespace sp::referee::client_id
{
constexpr uint16_t RED_HERO_CLIENT = 0x0101;
constexpr uint16_t RED_ENGINEER_CLIENT = 0x0102;
constexpr uint16_t RED_STANDARD_3_CLIENT = 0x0103;
constexpr uint16_t RED_STANDARD_4_CLIENT = 0x0104;
constexpr uint16_t RED_STANDARD_5_CLIENT = 0x0105;
constexpr uint16_t RED_AERIAL_CLIENT = 0x0106;
constexpr uint16_t BLUE_HERO_CLIENT = 0x0165;
constexpr uint16_t BLUE_ENGINEER_CLIENT = 0x0166;
constexpr uint16_t BLUE_STANDARD_3_CLIENT = 0x0167;
constexpr uint16_t BLUE_STANDARD_4_CLIENT = 0x0168;
constexpr uint16_t BLUE_STANDARD_5_CLIENT = 0x0169;
constexpr uint16_t BLUE_AERIAL_CLIENT = 0x016A;
constexpr uint16_t REFEREE_SERVER = 0x8080;
}  // namespace sp::referee::client_id

namespace sp::referee::cmd_id
{
constexpr uint16_t GAME_STATUS = 0x0001;    // 比赛状态数据
constexpr uint16_t GAME_RESULT = 0x0002;    // 比赛结果数据
constexpr uint16_t GAME_ROBOT_HP = 0x0003;  // 比赛机器人血量数据

constexpr uint16_t EVENT_DATA = 0x0101;       // 场地事件数据
constexpr uint16_t REFEREE_WARNING = 0x0104;  // 裁判警告数据
constexpr uint16_t DART_INFO = 0x0105;        // 飞镖发射相关数据

constexpr uint16_t ROBOT_STATUS = 0x0201;           // 机器人性能体系数据
constexpr uint16_t POWER_HEAT_DATA = 0x0202;        // 实时底盘缓冲能量和射击热量数据
constexpr uint16_t ROBOT_POS = 0x0203;              // 机器人位置数据
constexpr uint16_t BUFF = 0x0204;                   // 机器人增益数据
                                                    // 没有0x0205
constexpr uint16_t HURT_DATA = 0x0206;              // 伤害状态数据
constexpr uint16_t SHOOT_DATA = 0x0207;             // 实时射击数据
constexpr uint16_t PROJECTILE_ALLOWANCE = 0x0208;   // 允许发弹量
constexpr uint16_t RFID_STATUS = 0x0209;            // 机器人RFID模块状态
constexpr uint16_t DART_CLIENT_CMD = 0x020A;        // 飞镖选手端指令数据
constexpr uint16_t GROUND_ROBOT_POSITION = 0x020B;  // 地面机器人位置数据
constexpr uint16_t RADAR_MARK_DATA = 0x020C;        // 雷达标记进度数据
constexpr uint16_t SENTRY_INFO = 0x020D;            // 哨兵自主决策信息同步
constexpr uint16_t RADAR_INFO = 0x020E;             // 雷达自主决策信息同步

constexpr uint16_t ROBOT_INTERACTION_DATA = 0x0301;  // 机器人交互数据
constexpr uint16_t CUSTOM_ROBOT_DATA = 0x0302;       // 自定义控制器与机器人交互数据 图传链路
constexpr uint16_t MAP_COMMAND = 0x0303;             // 选手端小地图交互数据
constexpr uint16_t REMOTE_CONTROL = 0x0304;          // 键鼠遥控数据 图传链路
constexpr uint16_t MAP_ROBOT_DATA = 0x0305;          // 选手端小地图接收雷达数据
constexpr uint16_t CUSTOM_CLIENT_DATA = 0x0306;      // 自定义控制器与选手端交互数据
constexpr uint16_t MAP_DATA = 0x0307;                // 选手端小地图接收哨兵数据
constexpr uint16_t CUSTOM_INFO = 0x0308;             // 选手端小地图接收机器人数据
constexpr uint16_t ROBOT_CUSTOM_DATA = 0x0309;       // 自定义控制器接收机器人数据 图传链路
}  // namespace sp::referee::cmd_id

namespace sp::referee::data_cmd_id
{
// 0x0200~0x02FF 机器人之间通信 TODO
constexpr uint16_t INTERACTION_LAYER_DELETE = 0x0100;     // 选手端删除图层
constexpr uint16_t INTERACTION_FIGURE = 0x0101;           // 选手端绘制一个图形
constexpr uint16_t INTERACTION_FIGURE_2 = 0x0102;         // 选手端绘制两个图形
constexpr uint16_t INTERACTION_FIGURE_5 = 0x0103;         // 选手端绘制五个图形
constexpr uint16_t INTERACTION_FIGURE_7 = 0x0104;         // 选手端绘制七个图形
constexpr uint16_t EXT_CLIENT_CUSTOM_CHARACTER = 0x0110;  // 选手端绘制字符图形
constexpr uint16_t SENTRY_CMD = 0x0120;                   // 哨兵自主决策指令 TODO
constexpr uint16_t RADAR_CMD = 0x0121;                    // 雷达自主决策指令 TODO
}  // namespace sp::referee::data_cmd_id

namespace sp::referee
{
constexpr uint8_t SOF = 0xA5;

struct __attribute__((packed)) FrameHeader
{
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
};

constexpr size_t HEAD_LEN = sizeof(FrameHeader);
constexpr size_t CMD_ID_LEN = 2;
constexpr size_t TAIL_LEN = 2;
constexpr size_t DATA_START = HEAD_LEN + CMD_ID_LEN;

// 0x0001 比赛状态数据
struct __attribute__((packed)) GameStatus
{
  uint8_t game_type : 4;
  // bit 0-3: 比赛类型
  //       1: RoboMaster机甲大师超级对抗赛
  //       2: RoboMaster机甲大师高校单项赛
  //       3: ICRA RoboMaster高校人工智能挑战赛
  //       4: RoboMaster机甲大师高校联盟赛3V3对抗
  //       5: RoboMaster机甲大师高校联盟赛步兵对抗
  uint8_t game_progress : 4;
  // bit 4-7: 当前比赛阶段
  //       0: 未开始比赛
  //       1: 准备阶段
  //       2: 十五秒裁判系统自检阶段
  //       3: 五秒倒计时
  //       4: 比赛中
  //       5: 比赛结算中
  uint16_t stage_remain_time;  // 单位: s
  uint64_t sync_timestamp;     // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
};

// 0x0002 比赛结果数据
struct __attribute__((packed)) GameResult
{
  uint8_t winner;  // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
};

// 0x0003 比赛机器人血量数据
struct __attribute__((packed)) GameRobotHP
{
  uint16_t red_1_robot_hp;  // 红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
  uint16_t red_2_robot_hp;  // 红 2 工程机器人血量
  uint16_t red_3_robot_hp;  // 红 3 步兵机器人血量
  uint16_t red_4_robot_hp;  // 红 4 步兵机器人血量
  uint16_t reserved1;       // 保留
  uint16_t red_7_robot_hp;  // 红 7 哨兵机器人血量
  uint16_t red_outpost_hp;  // 红方前哨站血量
  uint16_t red_base_hp;     // 红方基地血量

  uint16_t blue_1_robot_hp;  // 蓝 1 英雄机器人血量
  uint16_t blue_2_robot_hp;  // 蓝 2 工程机器人血量
  uint16_t blue_3_robot_hp;  // 蓝 3 步兵机器人血量
  uint16_t blue_4_robot_hp;  // 蓝 4 步兵机器人血量
  uint16_t reserved2;        // 保留
  uint16_t blue_7_robot_hp;  // 蓝 7 哨兵机器人血量
  uint16_t blue_outpost_hp;  // 蓝方前哨站血量
  uint16_t blue_base_hp;     // 蓝方基地血量
};

// 0x0101 机器人事件数据
struct __attribute__((packed)) EventData
{
  // 0:未占领/未激活
  // 1:已占领/已激活
  uint32_t supply_status : 3;  // bit 0-2
  // bit 0：己方与兑换区不重叠的补给区占领状态，1 为已占领
  // bit 1：己方与兑换区重叠的补给区占领状态，1 为已占领
  // bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
  uint32_t energy_status : 2;  // bit 3-4
  // bit 3：己方小能量机关的激活状态，1 为已激活
  // bit 4：己方大能量机关的激活状态，1 为已激活
  uint32_t central_highground_status : 2;  // bit 5-6
  // bit 5-6：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
  uint32_t trapezoidal_highground_status : 2;  // bit 7-8
  // bit 7-8：己方梯形高地的占领状态，1 为已占领
  uint32_t last_hit_time : 9;  // bit 9-17
  // bit  9-17：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0）
  uint32_t last_hit_target : 3;  // bit 18-20
  // bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0
  // 1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标
  uint32_t center_boost_status : 2;  // bit 21-22
  // 中心增益点的占领状态，0 为未被占领，1 为被己方占领，2为被对方占领，3 为被双方占领。（仅 RMUL 适用）
  uint32_t reserved : 8;  // bit 23-31
  // 保留
};

// 0x0104 裁判警告数据
struct __attribute__((packed)) RefereeWarning
{
  uint8_t level;
  uint8_t offefending_robot_id;
  uint8_t count;
};

// 0x0105 飞镖发射相关数据
struct __attribute__((packed)) DartInfo
{
  uint8_t dart_remaining_time;
  uint16_t dart_info;
};

// 0x0201 机器人状态数据
struct __attribute__((packed)) RobotStatus
{
  uint8_t robot_id;                             // 机器人ID
  uint8_t robot_level;                          // 机器人等级
  uint16_t current_hp;                          // 当前血量
  uint16_t maximum_hp;                          // 最大血量
  uint16_t shooter_barrel_cooling_value;        // 发射机构冷却速率
  uint16_t shooter_barrel_heat_limit;           // 发射机构热量上限
  uint16_t chassis_power_limit;                 // 底盘功率上限
  uint8_t power_management_gimbal_output : 1;   // gimbal口输出: 0为无输出, 1为24V输出
  uint8_t power_management_chassis_output : 1;  // chassis口输出: 0为无输出, 1为24V输出
  uint8_t power_management_shooter_output : 1;  // shooter口输出: 0为无输出, 1为24V输出
};

// 0x0202 实时底盘缓冲能量和射击热量数据
struct __attribute__((packed)) PowerHeatData
{
  uint16_t reserved_1;
  uint16_t reserved_2;
  float reserved_3;
  uint16_t buffer_energy;               // 缓冲能量（单位：J）
  uint16_t shooter_17mm_1_barrel_heat;  // 第 1 个 17mm 发射机构的射击热量
  uint16_t shooter_17mm_2_barrel_heat;  // 第 2 个 17mm 发射机构的射击热量
  uint16_t shooter_42mm_barrel_heat;    // 42mm 发射机构的射击热量
};

// 0x0203 机器人位置数据
struct __attribute__((packed)) RobotPos
{
  float x;      // 本机器人位置 x 坐标，单位：m
  float y;      // 本机器人位置 y 坐标，单位：m
  float angle;  // 本机器人测速模块的朝向，单位：度。正北为 0 度
};

// 0x0204 机器人增益数据
struct __attribute__((packed)) Buff
{
  uint8_t recovery_y_buff;  // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
  uint8_t
    cooling_buff;  // 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却;堡垒增益点提供的固定热量冷却增益暂不适用）
  uint8_t defence_buff;        // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
  uint8_t vulnerability_buff;  // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
  uint16_t attack_buff;        // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
  uint8_t remaining_energy;
  // bit  0-4：机器人剩余能量值反馈，
  // 以 16 进制标识机器人剩余能量值比例，
  // 仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。
  // bit 0：在剩余能量≥50%时为 1，其余情况为 0
  // bit 1：在剩余能量≥30%时为 1，其余情况为 0
  // bit 2：在剩余能量≥15%时为 1，其余情况为 0
  // bit 3：在剩余能量≥5%时为 1，其余情况为 0
  // bit 4：在剩余能量≥1%时为 1，其余情况为 0
};

// 0x0206 伤害状态数据
struct __attribute__((packed)) HurtData
{
  uint8_t armor_id : 4;
  // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
  // 该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；
  // 当其他原因导致扣血时，该数值为 0
  uint8_t HP_deduction_reason : 4;
  // bit 4-7：血量变化类型
  // 0：装甲模块被弹丸攻击导致扣血
  // 1：裁判系统重要模块离线导致扣血
  // 2：射击初速度超限导致扣血
  // 3：射击热量超限导致扣血
  // 4：底盘功率超限导致扣血
  // 5：装甲模块受到撞击导致扣血
};

// 0x0207 实时射击数据
struct __attribute__((packed)) ShootData
{
  uint8_t bullet_type;          // 1: 17mm弹丸, 2: 42mm弹丸
  uint8_t shooter_number;       // 1: 第1个17mm发射机构, 2: 第2个17mm发射机构, 3: 42mm发射机构
  uint8_t launching_frequency;  // 单位: Hz
  float initial_speed;          // 单位: m/s
};

// 0x0208 允许发弹量
struct __attribute__((packed)) ProjectileAllowance
{
  uint16_t projectile_allowance_17mm;  // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;  // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;        // 剩余金币数量
};

// 0x0209 机器人RFID模块状态
struct __attribute__((packed)) RFID_Status
{
  uint32_t friendly_base : 1;             // bit 0：己方基地增益点
  uint32_t friendly_central : 1;          // bit 1：己方中央高地增益点
  uint32_t enemy_central : 1;             // bit 2：对方中央高地增益点
  uint32_t friendly_trapezoid : 1;        // bit 3：己方梯形高地增益点
  uint32_t enemy_trapezoid : 1;           // bit 4：对方梯形高地增益点
  uint32_t friendly_flying_slope_f : 1;   // bit 5：己方地形跨越增益点（飞坡前，靠近己方一侧）
  uint32_t friendly_flying_slope_b : 1;   // bit 6：己方地形跨越增益点（飞坡后，靠近己方一侧）
  uint32_t enemy_flying_slope_f : 1;      // bit 7：对方地形跨越增益点（飞坡前，靠近对方一侧）
  uint32_t enemy_flying_slope_b : 1;      // bit 8：对方地形跨越增益点（飞坡后，靠近对方一侧）
  uint32_t friendly_ch_down : 1;          // bit 9：己方地形跨越增益点（中央高地下方）
  uint32_t friendly_ch_up : 1;            // bit 10：己方地形跨越增益点（中央高地上方）
  uint32_t enemy_ch_down : 1;             // bit 11：对方地形跨越增益点（中央高地下方）
  uint32_t enemy_ch_up : 1;               // bit 12：对方地形跨越增益点（中央高地上方）
  uint32_t friendly_road_down : 1;        // bit 13：己方地形跨越增益点（公路下方）
  uint32_t friendly_road_up : 1;          // bit 14：己方地形跨越增益点（公路上方）
  uint32_t enemy_road_down : 1;           // bit 15：对方地形跨越增益点（公路下方）
  uint32_t enemy_road_up : 1;             // bit 16：对方地形跨越增益点（公路上方）
  uint32_t friendly_fort : 1;             // bit 17：己方堡垒增益点
  uint32_t friendly_outpost : 1;          // bit 18：己方前哨站增益点
  uint32_t friendly_supply_no_trade : 1;  // bit 19：己方与兑换区不重叠的补给区 / RMUL 补给区
  uint32_t friendly_supply_trade : 1;     // bit 20：己方与兑换区重叠的补给区
  uint32_t friendly_big_resource : 1;     // bit 21：己方大资源岛增益点
  uint32_t enemy_big_resource : 1;        // bit 22：对方大资源岛增益点
  uint32_t center_point : 1;              // bit 23：中心增益点（仅 RMUL 适用）
  uint32_t reserved : 8;                  // bit 24-31：保留
};

// 0x020A 飞镖选手端指令数据
struct __attribute__((packed)) DartClientCmd
{
  uint8_t
    dart_launch_opening_status;  // 当前飞镖发射站的状态： 1：关闭 2：正在开启或者关闭中 0：已经开启
  uint8_t reserved;
  uint16_t target_change_time;  // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
  uint16_t
    latest_launch_cmd_time;  // 发送最新飞镖发射指令的比赛剩余时间，单位：秒，无/未发射，默认为 0。
};

// 0x020B 地面机器人位置数据
struct __attribute__((packed)) GroundRobotPosition
{
  float hero_x;        // 己方英雄机器人位置 x 轴坐标，单位：m
  float hero_y;        // 己方英雄机器人位置 y 轴坐标，单位：m
  float engineer_x;    // 己方工程机器人位置 x 轴坐标，单位：m
  float engineer_y;    // 己方工程机器人位置 y 轴坐标，单位：m
  float standard_3_x;  // 己方 3 号步兵机器人位置 x 轴坐标，单位：m
  float standard_3_y;  // 己方 3 号步兵机器人位置 y 轴坐标，单位：m
  float standard_4_x;  // 己方 4 号步兵机器人位置 x 轴坐标，单位：m
  float standard_4_y;  // 己方 4 号步兵机器人位置 y 轴坐标，单位：m
  float reserved1;     // 保留
  float reserved2;     // 保留
};

// 0x020C 雷达标记进度数据
struct __attribute__((packed)) RadarMarkData
{
  // 在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。
  uint8_t enemy_hero_1_vulnerable : 1;      // bit 0: 对方 1号英雄机器人易伤情况
  uint8_t enemy_engineer_2_vulnerable : 1;  // bit 1: 对方 2号工程机器人易伤情况
  uint8_t enemy_infantry_3_vulnerable : 1;  // bit 2: 对方 3号步兵机器人易伤情况
  uint8_t enemy_infantry_4_vulnerable : 1;  // bit 3: 对方 4号步兵机器人易伤情况
  uint8_t enemy_sentry_vulnerable : 1;      // bit 4: 对方哨兵机器人易伤情况
  uint8_t reserved : 3;                     // bit 5-7: 保留
};

// 0x020D 哨兵自主决策信息同步
struct __attribute__((packed)) SentryInfo
{
  uint32_t allowed_fire_amount : 11;  // bits 0-10  : 除远程兑换外，哨兵机器人成功兑换的允许发弹量
  uint32_t remote_exchange_fire_count : 4;    // bits 11-14 : 哨兵机器人成功远程兑换允许发弹量的次数
  uint32_t remote_exchange_health_count : 4;  // bits 15-18 : 哨兵机器人成功远程兑换血量的次数
  uint32_t can_confirm_free_resurrect : 1;    // bit 19     : 哨兵机器人当前是否可以确认免费复活
  uint32_t can_exchange_immediate_resurrect : 1;  // bit 20     : 哨兵机器人当前是否可以兑换立即复活
  uint32_t
    immediate_resurrect_cost : 10;  // bits 21-30 : 哨兵机器人当前若兑换立即复活需要花费的金币数
  uint32_t reserved1 : 1;           // bit 31     : 保留
  uint16_t is_out_of_battle : 1;    // bit 0      : 哨兵当前是否处于脱战状态
  uint16_t team_17mm_remaining_exchanges : 11;  // bits 1-11  : 队伍 17mm 允许发弹量的剩余可兑换数
  uint16_t reserved2 : 4;                       // bits 12-15 : 保留
};

// 0x020E 雷达自主决策信息同步
struct __attribute__((packed)) RadarInfo
{
  uint8_t
    double_vul_trigger_chance : 2;  // bits 0-1 : 雷达拥有触发双倍易伤的机会（0~2），开局为 0，最大可达 2
  uint8_t
    opponent_in_double_vulnerability : 1;  // bit 2 : 对方是否正在被触发双倍易伤 0：对方未被触发双倍易伤 1：对方正在被触发双倍易伤
  uint8_t reserved : 5;  // bits 3-7
};

// 0x0301 机器人交互数据
struct __attribute__((packed)) RobotInteractionData
{
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[112];  // 最大为112
};

// 0x0301 0x0100 选手端删除图层
struct __attribute__((packed)) InteractionLayerDelete
{
  uint8_t delete_type;  // 0：空操作 1：删除图层 2：删除所有
  uint8_t layer;        // 图层数：0~9
};

// 0x0301 0x0101 选手端绘制一个图形
struct __attribute__((packed)) InteractionFigure
{
  uint8_t figure_name[3];  // 在图形删除、修改等操作中，作为索引

  // bit 0-2：图形操作 0：空操作 1：增加 2：修改 3：删除
  uint32_t operate_type : 3;
  // bit 3-5：图形类型 0：直线 1：矩形 2：正圆 3：椭圆 4：圆弧 5：浮点数 6：整型数 7：字符
  uint32_t figure_type : 3;
  // bit 6-9：图层数（0~9）
  uint32_t layer : 4;
  // bit 10-13：颜色 0：红/蓝（己方颜色） 1：黄色 2：绿色 3：橙色 4：紫红色 5：粉色 6：青色 7：黑色 8：白色
  uint32_t color : 4;
  // bit 14-22：根据绘制的图形不同，含义不同
  uint32_t details_a : 9;
  // bit 23-31：根据绘制的图形不同，含义不同
  uint32_t details_b : 9;

  // bit 0-9：线宽，建议字体大小与线宽比例为 10：1
  uint32_t width : 10;
  // bit 10-20：起点/圆心 x 坐标
  uint32_t start_x : 11;
  // bit 21-31：起点/圆心 y 坐标
  uint32_t start_y : 11;

  // bit 0-9：根据绘制的图形不同，含义不同
  uint32_t details_c : 10;
  // bit 10-20：根据绘制的图形不同，含义不同
  uint32_t details_d : 11;
  // bit 21-31：根据绘制的图形不同，含义不同
  uint32_t details_e : 11;
};

// 0x0301 0x0102 选手端绘制两个图形
struct __attribute__((packed)) InteractionFigure2
{
  InteractionFigure interaction_figures[2];
};

// 0x0301 0x0103 选手端绘制五个图形
struct __attribute__((packed)) InteractionFigure5
{
  InteractionFigure interaction_figures[5];
};

// 0x0301 0x0104 选手端绘制七个图形
struct __attribute__((packed)) InteractionFigure7
{
  InteractionFigure interaction_figures[7];
};

// 0x0301 0x0110 选手端绘制字符图形
struct __attribute__((packed)) ExtClientCustomCharacter
{
  InteractionFigure interaction_figure;
  uint8_t data[30];
};

// 0x0301 0x0120 哨兵自主决策指令 TODO
struct __attribute__((packed)) SentryCmd
{
  uint32_t resurrect : 1;             // bit 0：是否确认复活 0：否 1：是
  uint32_t immediate_resurrect : 1;   // bit 1：是否消耗金币兑换立即复活  0：否 1：是
  uint32_t exchange_17mm_value : 11;  // bit 2-12：发弹量兑换值（递增有效）
  uint32_t
    remote_exchange_17mm_count : 4;  // bit 13-16：远程兑换发弹量次数，开局为0，单调递增，每次加1
  uint32_t
    remote_exchange_blood_count : 4;  // bit 17-20：远程兑换血量次数，开局为0，单调递增，每次加1
  uint32_t reserved : 11;             // bit 21-31：保留
};

// 0x0301 0x0121 雷达自主决策指令 TODO
struct __attribute__((packed)) RadarCmd
{
  uint8_t radar_cmd;  // 触发双倍易伤次数， 开局为0，单调递增，每次加1，最大为2
};

// 0x0302 自定义控制器与机器人交互数据 图传链路
struct __attribute__((packed)) CustomRobotData
{
  uint8_t data[30];
};

// 0x0303 选手端小地图交互数据
struct __attribute__((packed)) MapCommand
{
  float target_position_x;  // 目标位置 x 轴坐标，单位：m
  float target_position_y;  // 目标位置 y 轴坐标，单位：m
  uint8_t cmd_keyboard;     // 云台手按下的键盘按键通用键值
  uint8_t target_robot_id;  // 对方机器人 ID
  uint16_t cmd_source;      // 信息来源 ID
};

// 0x0304 键鼠遥控数据 图传链路
struct __attribute__((packed)) RemoteControl
{
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;   // 0: 未按下, 1: 按下
  int8_t right_button_down;  // 0: 未按下, 1: 按下
  uint16_t keyboard_value;   // 每个bit对应一个按键, 详见《RoboMaster裁判系统串口协议附录.pdf》
  uint16_t reserved;
};

// 0x0305 选手端小地图接收雷达数据
struct __attribute__((packed)) MapRobotData
{
  uint16_t hero_position_x;        // 英雄机器人 x 位置坐标，单位：cm
  uint16_t hero_position_y;        // 英雄机器人 y 位置坐标，单位：cm
  uint16_t engineer_position_x;    // 工程机器人 x 位置坐标，单位：cm
  uint16_t engineer_position_y;    // 工程机器人 y 位置坐标，单位：cm
  uint16_t infantry_3_position_x;  // 3号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_3_position_y;  // 3号步兵机器人 y 位置坐标，单位：cm
  uint16_t infantry_4_position_x;  // 4号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_4_position_y;  // 4号步兵机器人 y 位置坐标，单位：cm
  uint16_t infantry_5_position_x;  // 5号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_5_position_y;  // 5号步兵机器人 y 位置坐标，单位：cm
  uint16_t sentry_position_x;      // 哨兵机器人 x 位置坐标，单位：cm
  uint16_t sentry_position_y;      // 哨兵机器人 y 位置坐标，单位：cm
};

// 0x0306 自定义控制器与选手端交互数据
struct __attribute__((packed)) CustomClientData
{
  uint16_t key_value_1 : 8;  // bit 0-7：按键 1 键值
  uint16_t key_value_2 : 8;  // bit 8-15：按键 2 键值
  uint16_t x_position : 12;  // bit 0-11：鼠标 X 轴像素位置
  uint16_t mouse_left : 4;   // bit 12-15：鼠标左键状态
  uint16_t y_position : 12;  // bit 0-11：鼠标 Y 轴像素位置
  uint16_t mouse_right : 4;  // bit 12-15：鼠标右键状态
  uint16_t reserved;
};

// 0x0307 选手端小地图接收哨兵数据
struct __attribute__((packed)) MapData
{
  uint8_t intention;          // 1：到目标点攻击 2：到目标点防守 3：移动到目标点
  uint16_t start_position_x;  // 路径起点 x 轴坐标，单位：dm
  uint16_t start_position_y;  // 路径起点 y 轴坐标，单位：dm
  int8_t delta_x[49];         // 路径点 x 轴增量数组，单位：dm
  int8_t delta_y[49];         // 路径点 y 轴增量数组，单位：dm
  uint16_t sender_id;         // 发送方 ID
};

// 0x0308 选手端小地图接收机器人数据
struct __attribute__((packed)) CustomInfo
{
  uint16_t sender_id;     // 发送者的 ID
  uint16_t receiver_id;   // 接收者的 ID
  uint8_t user_data[30];  // 字符
};

// 0x0309 自定义控制器接收机器人数据 图传链路
struct __attribute__((packed)) RobotCustomData
{
  uint8_t data[30];
};

}  // namespace sp::referee

#endif  // SP__REFEREE_PROTOCOL_HPP