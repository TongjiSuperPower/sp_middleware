#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
VT03::VT03(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

bool VT03::is_open() const { return has_read_; }

bool VT03::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

bool VT03::custom_2_robot_is_open() const { return custom_2_robot_has_read_; }

bool VT03::custom_2_robotis_alive(uint32_t now_ms) const
{
  return custom_2_robot_is_open() && (now_ms - custom_2_robot_last_read_ms_ < 100);
}

void VT03::request()
{
  if (use_dma_) {
    this->huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    this->huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    this->huart->RxXferSize = DMA_NDTR_SIZE;

    SET_BIT(this->huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);

    HAL_DMAEx_MultiBufferStart(
      this->huart->hdmarx, (uint32_t)&this->huart->Instance->RDR, (uint32_t)multi_buff_[0].data(),
      (uint32_t)multi_buff_[1].data(), DMA_NDTR_SIZE);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
  }
}

void VT03::update(uint16_t size, uint32_t stamp_ms)
{
  if (use_dma_) {
    DMA_Stream_TypeDef * dma_stream = (DMA_Stream_TypeDef *)this->huart->hdmarx->Instance;

    if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
      __HAL_DMA_DISABLE(this->huart->hdmarx);  // 暂停 DMA 传输
      dma_stream->CR |= DMA_SxCR_CT;           // 置 1 CT位，将下一次写入目标切换至 Memory 1
      __HAL_DMA_SET_COUNTER(this->huart->hdmarx, DMA_NDTR_SIZE);  // 重置计数器为双倍长度

      // 提取 Memory 0 的数据进行解析
      update(multi_buff_[0].data(), size, stamp_ms);
    }
    else {
      // 此时 CT = 1，数据刚刚被写入 Memory 1
      __HAL_DMA_DISABLE(this->huart->hdmarx);  // 暂停 DMA 传输
      dma_stream->CR &= ~(DMA_SxCR_CT);        // 清零 CT位，将下一次写入目标切换至 Memory 0
      __HAL_DMA_SET_COUNTER(this->huart->hdmarx, DMA_NDTR_SIZE);  // 重置计数器为双倍长度

      // 提取 Memory 1 的数据进行解析
      update(multi_buff_[1].data(), size, stamp_ms);
    }

    // 重新开启 DMA，等待下一帧数据流入
    this->huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    __HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);
    SET_BIT(this->huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_ENABLE(this->huart->hdmarx);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
    update(buff_.data(), size, stamp_ms);
  }
}

void VT03::update(uint8_t * frame_start, uint16_t size, uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    size_t frame_len = sizeof(VT03RemoteData);

    if (size < frame_len) return;
    if (!check_crc16(frame_start, frame_len)) return;

    update_remote(reinterpret_cast<VT03RemoteData *>(frame_start));

    // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
    update(frame_start + frame_len, size - frame_len, stamp_ms);
    return;
  }

  if (size < referee::HEAD_LEN) return;
  if (frame_start[0] != referee::SOF) return;
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  if (size < frame_len) return;
  if (!check_crc16(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];

  switch (cmd_id) {
    // 0x0302 自定义控制器与机器人交互数据
    case referee::cmd_id::CUSTOM_ROBOT_DATA:
      std::copy(
        frame_start + referee::DATA_START, frame_start + referee::DATA_START + data_len,
        reinterpret_cast<uint8_t *>(&this->custom));
      custom_2_robot_last_read_ms_ = stamp_ms;
      custom_2_robot_has_read_ = true;
      break;

    // 0x0309 自定义控制器与机器人交互数据
    case referee::cmd_id::ROBOT_CUSTOM_DATA:
      std::copy(
        frame_start + referee::DATA_START, frame_start + referee::DATA_START + data_len,
        reinterpret_cast<uint8_t *>(&this->robot));
      break;

    //0x0311 自定义客户端发送给机器人的自定义指令
    case referee::cmd_id::CLIENT_ROBOT_DATA:
      std::fill(this->custom_client.data.begin(), this->custom_client.data.end(), 0);

      if (data_len > this->custom_client.data.size()) {
        this->custom_client.size = 0;
        this->custom_client.is_valid = false;
        break;
      }

      this->custom_client.size = static_cast<uint8_t>(data_len);
      this->custom_client.is_valid = true;
      std::memcpy(this->custom_client.data.data(), frame_start + referee::DATA_START, data_len);
      break;

    default:
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  update(frame_start + frame_len, size - frame_len, stamp_ms);
}

void VT03::update_remote(const VT03RemoteData * data)
{
  this->ch_rh = (data->ch_0 - 1024) / 660.0f;
  this->ch_rv = (data->ch_1 - 1024) / 660.0f;
  this->ch_lv = (data->ch_2 - 1024) / 660.0f;
  this->ch_lh = (data->ch_3 - 1024) / 660.0f;
  this->wheel = (data->wheel - 1024) / 660.0f;

  this->fn_l = data->fn_1;
  this->fn_r = data->fn_2;
  this->pause = data->pause;
  this->trigger = data->trigger;

  this->mouse.vx = data->mouse_x / 32768.0f;
  this->mouse.vy = data->mouse_y / 32768.0f;
  this->mouse.vs = data->mouse_z / 32768.0f;
  this->mouse.left = data->mouse_left;
  this->mouse.middle = data->mouse_middle;
  this->mouse.right = data->mouse_right;

  this->keyboard_value = data->keys;

  this->keys.w = ((this->keyboard_value & VT03_KEY_W_MASK) != 0);
  this->keys.s = ((this->keyboard_value & VT03_KEY_S_MASK) != 0);
  this->keys.a = ((this->keyboard_value & VT03_KEY_A_MASK) != 0);
  this->keys.d = ((this->keyboard_value & VT03_KEY_D_MASK) != 0);
  this->keys.shift = ((this->keyboard_value & VT03_KEY_SHIFT_MASK) != 0);
  this->keys.ctrl = ((this->keyboard_value & VT03_KEY_CTRL_MASK) != 0);
  this->keys.q = ((this->keyboard_value & VT03_KEY_Q_MASK) != 0);
  this->keys.e = ((this->keyboard_value & VT03_KEY_E_MASK) != 0);
  this->keys.r = ((this->keyboard_value & VT03_KEY_R_MASK) != 0);
  this->keys.f = ((this->keyboard_value & VT03_KEY_F_MASK) != 0);
  this->keys.g = ((this->keyboard_value & VT03_KEY_G_MASK) != 0);
  this->keys.z = ((this->keyboard_value & VT03_KEY_Z_MASK) != 0);
  this->keys.x = ((this->keyboard_value & VT03_KEY_X_MASK) != 0);
  this->keys.c = ((this->keyboard_value & VT03_KEY_C_MASK) != 0);
  this->keys.v = ((this->keyboard_value & VT03_KEY_V_MASK) != 0);
  this->keys.b = ((this->keyboard_value & VT03_KEY_B_MASK) != 0);

  this->mode = (data->mode_sw == 2)   ? VT03Mode::S
               : (data->mode_sw == 1) ? VT03Mode::N
                                      : VT03Mode::C;
}

void VT03::send_custom_client_data(const CustomByteBlock & custom_data)
{
  // 静态数组避免频繁在栈上分配内存，320字节通常足够裁判系统单包使用
  static uint8_t tx_buff[320];

  constexpr size_t data_len = sizeof(CustomByteBlock);
  constexpr size_t frame_len =
    referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // 1. 安全防线：防止缓冲区溢出
  if (frame_len > sizeof(tx_buff)) return;

  // 2. 组装帧头 (Frame Header)
  tx_buff[0] = referee::SOF;
  tx_buff[1] = data_len & 0xFF;
  tx_buff[2] = (data_len >> 8) & 0xFF;
  tx_buff[3] = this->seq_++;
  sp::append_crc8(tx_buff, referee::HEAD_LEN);  // 一步追加 CRC8

  // 3. 组装命令码 (Cmd ID = 0x0310)
  tx_buff[referee::HEAD_LEN] = 0x10;
  tx_buff[referee::HEAD_LEN + 1] = 0x03;

  // 4. 拷贝数据段 (Data)
  std::memcpy(tx_buff + referee::DATA_START, &custom_data, data_len);

  // 5. 组装帧尾 (Frame Tail / CRC16)
  sp::append_crc16(tx_buff, frame_len);  // 优雅地一步追加 CRC16

  // 6. 物理层发送
  if (use_dma_) {
    HAL_UART_Transmit_DMA(this->huart, tx_buff, frame_len);
  }
  else {
    HAL_UART_Transmit_IT(this->huart, tx_buff, frame_len);
  }
}

}  // namespace sp
