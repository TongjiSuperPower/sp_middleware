#include "pm02.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace sp
{
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

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

  if (size < frame_len) return;
  if (!check_crc16(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];

  switch (cmd_id) {
    // 0x0001 比赛状态数据
    case referee::cmd_id::GAME_STATUS:
      std::memcpy(&(this->game_status), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0002 比赛结果数据
    case referee::cmd_id::GAME_RESULT:
      std::memcpy(&(this->game_result), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0003 比赛机器人血量数据
    case referee::cmd_id::GAME_ROBOT_HP:
      std::memcpy(&(this->game_robot_hp), frame_start + referee::DATA_START, data_len);
      break;

    // 0x0101 机器人事件数据
    case referee::cmd_id::EVENT_DATA:
      std::memcpy(&(this->event), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0104 裁判警告数据
    case referee::cmd_id::REFEREE_WARNING:
      std::memcpy(&(this->referee_warning), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0105 飞镖发射相关数据
    case referee::cmd_id::DART_INFO:
      std::memcpy(&(this->dart_info), frame_start + referee::DATA_START, data_len);
      break;

    // 0x0201 机器人性能体系数据
    case referee::cmd_id::ROBOT_STATUS:
      std::memcpy(&(this->robot_status), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0202 实时底盘缓冲能量和射击热量数据
    case referee::cmd_id::POWER_HEAT_DATA:
      std::memcpy(&(this->power_heat), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0203 机器人位置数据
    case referee::cmd_id::ROBOT_POS:
      std::memcpy(&(this->robot_pos), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0204 机器人增益数据
    case referee::cmd_id::BUFF:
      std::memcpy(&(this->buff), frame_start + referee::DATA_START, data_len);
      break;
    // 没有0x0205
    // 0x0206 伤害状态数据
    case referee::cmd_id::HURT_DATA:
      std::memcpy(&(this->hurt), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0207 实时射击数据
    case referee::cmd_id::SHOOT_DATA:
      std::memcpy(&(this->shoot), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0208 允许发弹量
    case referee::cmd_id::PROJECTILE_ALLOWANCE:
      std::memcpy(&(this->projectile_allowance), frame_start + referee::DATA_START, data_len);
      break;
    // 0x0209 机器人RFID模块状态
    case referee::cmd_id::RFID_STATUS:
      std::memcpy(&(this->rfid_status), frame_start + referee::DATA_START, data_len);
      break;
    // 0x020A 飞镖选手端指令数据
    case referee::cmd_id::DART_CLIENT_CMD:
      std::memcpy(&(this->dart_client_cmd), frame_start + referee::DATA_START, data_len);
      break;
    // 0x020B 地面机器人位置数据
    case referee::cmd_id::GROUND_ROBOT_POSITION:
      std::memcpy(&(this->ground_robot_pos), frame_start + referee::DATA_START, data_len);
      break;
    // 0x020C 雷达标记进度数据
    case referee::cmd_id::RADAR_MARK_DATA:
      std::memcpy(&(this->radar_mark), frame_start + referee::DATA_START, data_len);
      break;
    // 0x020D 哨兵自主决策信息同步
    case referee::cmd_id::SENTRY_INFO:
      std::memcpy(&(this->sentry_info), frame_start + referee::DATA_START, data_len);
      break;
    // 0x020E 雷达自主决策信息同步
    case referee::cmd_id::RADAR_INFO:
      std::memcpy(&(this->radar_info), frame_start + referee::DATA_START, data_len);
      break;

    // 0x0303 选手端小地图交互数据
    case referee::cmd_id::MAP_COMMAND:
      std::memcpy(&(this->map_command), frame_start + referee::DATA_START, data_len);
      break;

    default:
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
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

}  // namespace sp
