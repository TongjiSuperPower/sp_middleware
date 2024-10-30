#include "pm02.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace sp
{
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart_(huart), use_dma_(use_dma) {}

void PM02::request()
{
  if (use_dma_)
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, PM02_BUFF_SIZE);  // dismiss return
  else
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, PM02_BUFF_SIZE);  // dismiss return
}

void PM02::update()
{
  if (buff_[0] != referee::SOF) return;
  if (!check_crc8(buff_, referee::HEAD_LEN)) return;

  size_t data_len = (buff_[2] << 8) | buff_[1];
  size_t len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;
  if (!check_crc16(buff_, len)) return;

  auto cmd_id = referee::CMD_ID((buff_[6] << 8) | buff_[5]);

  switch (cmd_id) {
    case referee::CMD_ID::GAME_STATUS:
      std::memcpy(&(this->game_status), buff_ + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::GAME_RESULT:
      std::memcpy(&(this->game_result), buff_ + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::GAME_ROBOT_HP:
      std::memcpy(&(this->game_robot_hp), buff_ + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::ROBOT_STATUS:
      std::memcpy(&(this->robot_status), buff_ + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::POWER_HEAT_DATA:
      std::memcpy(&(this->power_heat), buff_ + referee::DATA_START, data_len);
      break;
    case referee::CMD_ID::SHOOT_DATA:
      std::memcpy(&(this->shoot), buff_ + referee::DATA_START, data_len);
      break;
    // TODO 其它数据请参考官网最新版《RoboMaster裁判系统串口协议附录.pdf》
    default:
      break;
  }
}

}  // namespace sp
