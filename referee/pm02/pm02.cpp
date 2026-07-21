#include "pm02.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace
{
constexpr size_t INTERACTION_HEADER_LEN = 6;

template <typename T>
void copy_fixed(T & dst, const uint8_t * data, size_t size)
{
  // 固定长度协议帧必须和结构体大小一致，避免异常长度导致 memcpy 越界。
  if (size != sizeof(T)) return;
  std::memcpy(&dst, data, sizeof(T));
}

uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

bool sentry_to_radar_ids(uint16_t sentry_id, uint16_t & radar_id)
{
  if (sentry_id == sp::referee::robot_id::RED_SENTRY) {
    radar_id = sp::referee::robot_id::RED_RADAR;
    return true;
  }
  if (sentry_id == sp::referee::robot_id::BLUE_SENTRY) {
    radar_id = sp::referee::robot_id::BLUE_RADAR;
    return true;
  }
  return false;
}

}  // namespace

namespace sp
{
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma)
{
  uint8_t * start = reinterpret_cast<uint8_t *>(&this->game_status);
  uint8_t * end = reinterpret_cast<uint8_t *>(&this->custom_info) + sizeof(this->custom_info);
  if (end > start) {
    std::memset(start, 0, static_cast<size_t>(end - start));
  }

  std::memset(this->buff_, 0, sizeof(this->buff_));
  for (auto & mb : this->multi_buff_) {
    std::memset(mb.data(), 0, mb.size());
  }
}

void PM02::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_, PM02_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_, PM02_BUFF_SIZE);
  }
}

void PM02::update(uint16_t size) { update(buff_, size); }

void PM02::update(uint8_t * frame_start, uint16_t size)
{
  if (size < referee::HEAD_LEN) return;
  if (frame_start[0] != referee::SOF) return;
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  if (frame_len > PM02_BUFF_SIZE) return;
  if (size < frame_len) return;
  if (!check_crc16(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];
  const uint8_t * data = frame_start + referee::DATA_START;

  switch (cmd_id) {
    // 0x0001 比赛状态数据
    case referee::cmd_id::GAME_STATUS:
      copy_fixed(this->game_status, data, data_len);
      break;
    // 0x0002 比赛结果数据
    case referee::cmd_id::GAME_RESULT:
      copy_fixed(this->game_result, data, data_len);
      break;
    // 0x0003 比赛机器人血量数据
    case referee::cmd_id::GAME_ROBOT_HP:
      copy_fixed(this->game_robot_hp, data, data_len);
      break;
    // 0x0101 场地事件数据
    case referee::cmd_id::EVENT_DATA:
      copy_fixed(this->event, data, data_len);
      break;
    // 0x0104 裁判警告数据
    case referee::cmd_id::REFEREE_WARNING:
      copy_fixed(this->referee_warning, data, data_len);
      break;
    // 0x0105 飞镖发射相关数据
    case referee::cmd_id::DART_INFO:
      copy_fixed(this->dart_info, data, data_len);
      break;
    // 0x0201 机器人状态数据
    case referee::cmd_id::ROBOT_STATUS:
      copy_fixed(this->robot_status, data, data_len);
      break;
    // 0x0202 实时底盘缓冲能量和射击热量数据
    case referee::cmd_id::POWER_HEAT_DATA:
      copy_fixed(this->power_heat, data, data_len);
      break;
    // 0x0203 机器人位置数据
    case referee::cmd_id::ROBOT_POS:
      copy_fixed(this->robot_pos, data, data_len);
      break;
    // 0x0204 机器人增益数据
    case referee::cmd_id::BUFF:
      copy_fixed(this->buff, data, data_len);
      break;
    // 没有 0x0205
    // 0x0206 伤害状态数据
    case referee::cmd_id::HURT_DATA:
      copy_fixed(this->hurt, data, data_len);
      break;
    // 0x0207 实时射击数据
    case referee::cmd_id::SHOOT_DATA:
      copy_fixed(this->shoot, data, data_len);
      break;
    // 0x0208 允许发弹量数据
    case referee::cmd_id::PROJECTILE_ALLOWANCE:
      copy_fixed(this->projectile_allowance, data, data_len);
      break;
    // 0x0209 机器人 RFID 模块状态
    case referee::cmd_id::RFID_STATUS:
      copy_fixed(this->rfid_status, data, data_len);
      break;
    // 0x020A 飞镖选手端指令数据
    case referee::cmd_id::DART_CLIENT_CMD:
      copy_fixed(this->dart_client_cmd, data, data_len);
      break;
    // 0x020B 地面机器人位置数据
    case referee::cmd_id::GROUND_ROBOT_POSITION:
      copy_fixed(this->ground_robot_pos, data, data_len);
      break;
    // 0x020C 雷达标记进度数据
    case referee::cmd_id::RADAR_MARK_DATA:
      copy_fixed(this->radar_mark, data, data_len);
      break;
    // 0x020D 哨兵自主决策信息同步
    case referee::cmd_id::SENTRY_INFO:
      copy_fixed(this->sentry_info, data, data_len);
      break;
    // 0x020E 雷达自主决策信息同步
    case referee::cmd_id::RADAR_INFO:
      copy_fixed(this->radar_info, data, data_len);
      break;
    // 0x0A05 雷达站增益点状态数据（雷达侧可据此生成无敌掩码）
    case referee::cmd_id::RADAR_BUFF_STATUS:
      if (data_len == sizeof(this->radar_buff_status)) {
        copy_fixed(this->radar_buff_status, data, data_len);
        this->radar_buff_status_valid = referee::radar_buff_status_valid(this->radar_buff_status);
        if (this->radar_buff_status_valid) {
          this->radar_buff_status_last_update_ms = HAL_GetTick();
        }
      }
      else {
        this->radar_buff_status_valid = false;
      }
      break;
    // 0x0301 机器人交互数据
    case referee::cmd_id::ROBOT_INTERACTION_DATA:
      if (data_len >= INTERACTION_HEADER_LEN) {
        const uint16_t data_cmd_id = read_u16_le(data);
        const uint16_t sender_id = read_u16_le(data + 2);
        const uint16_t receiver_id = read_u16_le(data + 4);
        uint16_t expected_radar_id = 0;

        const bool valid_route =
          sentry_to_radar_ids(this->robot_status.robot_id, expected_radar_id) &&
          sender_id == expected_radar_id && receiver_id == this->robot_status.robot_id;

        if (data_cmd_id == referee::data_cmd_id::RADAR_SENTRY_BUFF_CMD && valid_route) {
          // 雷达当前发送格式：6 字节交互头 + 完整 41 字节 0x0A05 数据。
          if (data_len == INTERACTION_HEADER_LEN + sizeof(referee::RadarBuffStatus)) {
            referee::RadarBuffStatus received_buff{};
            std::memcpy(&received_buff, data + INTERACTION_HEADER_LEN, sizeof(received_buff));

            if (referee::radar_buff_status_valid(received_buff)) {
              this->radar_buff_status = received_buff;
              this->radar_buff_status_valid = true;
              this->radar_buff_status_last_update_ms = HAL_GetTick();
            }
          }
        }
        else if (
          data_cmd_id == referee::data_cmd_id::RADAR_SENTRY_POSITION_CMD && valid_route &&
          data_len == INTERACTION_HEADER_LEN + sizeof(referee::RadarSentryPosition)) {
          referee::RadarSentryPosition received_position{};
          std::memcpy(
            &received_position, data + INTERACTION_HEADER_LEN, sizeof(received_position));
          this->enemy_robot_position = received_position;
          this->enemy_robot_position_valid = true;
          this->enemy_robot_position_last_update_ms = HAL_GetTick();
        }
      }
      break;
    // 0x0303 选手端小地图交互数据
    case referee::cmd_id::MAP_COMMAND:
      copy_fixed(this->map_command, data, data_len);
      break;
    // 0x0305 选手端小地图接收雷达数据
    case referee::cmd_id::MAP_ROBOT_DATA:
      copy_fixed(this->map_robot_data, data, data_len);
      break;
    // 0x0306 自定义控制器与选手端交互数据
    // case referee::cmd_id::CUSTOM_CLIENT_DATA:
    //   copy_fixed(this->custom_client_data, data, data_len);
    //   break;
    // 0x0307 选手端小地图接收哨兵数据
    case referee::cmd_id::MAP_DATA:
      copy_fixed(this->map_data, data, data_len);
      break;
    // 0x0308 选手端小地图接收机器人数据
    case referee::cmd_id::CUSTOM_INFO:
      copy_fixed(this->custom_info, data, data_len);
      break;
    default:
      break;
  }

  // 递归解析，因为缓冲区中可能包含多帧裁判系统的数据。
  update(frame_start + frame_len, size - frame_len);
}

void PM02::send(const uint8_t * data, size_t size)
{
  if (use_dma_) {
    HAL_UART_Transmit_DMA(this->huart, data, size);
  }
  else {
    HAL_UART_Transmit(this->huart, data, size, 0xFF);
  }
}

bool PM02::radar_buff_status_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_buff_status_valid &&
         static_cast<uint32_t>(now_ms - this->radar_buff_status_last_update_ms) <= timeout_ms;
}

bool PM02::enemy_robot_position_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->enemy_robot_position_valid &&
         static_cast<uint32_t>(now_ms - this->enemy_robot_position_last_update_ms) <= timeout_ms;
}

}  // namespace sp
