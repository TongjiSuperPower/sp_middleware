#include "pm02.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace sp
{
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart_(huart), use_dma_(use_dma) {}

void PM02::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, PM02_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, PM02_BUFF_SIZE);
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

  auto cmd_id = referee::CMD_ID((frame_start[6] << 8) | frame_start[5]);

  switch (cmd_id) {
    case referee::CMD_ID::GAME_STATUS:
      std::memcpy(&(this->game_status), frame_start + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::GAME_RESULT:
      std::memcpy(&(this->game_result), frame_start + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::GAME_ROBOT_HP:
      std::memcpy(&(this->game_robot_hp), frame_start + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::ROBOT_STATUS:
      std::memcpy(&(this->robot_status), frame_start + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::POWER_HEAT_DATA:
      std::memcpy(&(this->power_heat), frame_start + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::SHOOT_DATA:
      std::memcpy(&(this->shoot), frame_start + referee::DATA_START, data_len);
      break;
    // TODO 其它数据请参考官网最新版《RoboMaster裁判系统串口协议附录.pdf》
    default:
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  update(frame_start + frame_len, size - frame_len);
}

}  // namespace sp
