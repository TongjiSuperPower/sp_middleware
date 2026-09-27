#include "laser.hpp"

namespace sp
{
LaserGY56::LaserGY56(UART_HandleTypeDef * huart)
: huart_(huart),
  rx_buff_{0},
  frame_{0},
  frame_size_(0),
  distance_cm_(0),
  last_frame_tick_(0),
  mode_(LaserMode::UNKNOWN),
  has_reliable_distance_(false)
{
}

uint16_t LaserGY56::get_distance() const { return distance_cm_; }

bool LaserGY56::is_valid(uint32_t timeout_ms) const
{
  if (!has_reliable_distance_) {
    return false;
  }

  return HAL_GetTick() - last_frame_tick_ <= timeout_ms;
}

LaserMode LaserGY56::get_mode() const { return mode_; }

bool LaserGY56::send_mode(LaserMode mode)
{
  switch (mode) {
    case LaserMode::SHORT:
      return send_command(0x51);
    case LaserMode::MEDIUM:
      return send_command(0x52);
    case LaserMode::LONG:
      return send_command(0x53);
    default:
      return false;
  }
}

bool LaserGY56::set_baud_rate_9600() { return send_command(0xAE); }

bool LaserGY56::set_baud_rate_115200() { return send_command(0xAF); }

bool LaserGY56::save_config() { return send_command(0x25); }

bool LaserGY56::start_receive()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return false;
  }
  if (huart_->RxState == HAL_UART_STATE_BUSY_RX) {
    return true;
  }
  if (huart_->RxState != HAL_UART_STATE_READY) {
    return false;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, RX_BUFFER_SIZE) != HAL_OK) {
    return false;
  }

  // 仅在空闲或缓冲区满时处理数据, 避免半满回调重复解析
  __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  return true;
}

void LaserGY56::receive_complete(uint16_t size)
{
  if (size > RX_BUFFER_SIZE) {
    size = RX_BUFFER_SIZE;
  }

  for (uint16_t i = 0; i < size; i++) {
    update(rx_buff_[i]);
  }
  start_receive();
}

void LaserGY56::update(uint8_t byte)
{
  // 维护8字节窗口, 使帧头错位或分段接收时仍能重新找到完整数据帧
  if (frame_size_ < FRAME_SIZE) {
    frame_[frame_size_] = byte;
    frame_size_++;
  }
  else {
    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
      frame_[i] = frame_[i + 1];
    }
    frame_[FRAME_SIZE - 1] = byte;
  }

  if (frame_size_ < FRAME_SIZE || !frame_valid()) {
    return;
  }

  parse_frame();
  frame_size_ = 0;
}

bool LaserGY56::send_command(uint8_t command)
{
  if (huart_ == nullptr) {
    return false;
  }

  uint8_t data[3] = {0xA5, command, static_cast<uint8_t>(0xA5 + command)};
  return HAL_UART_Transmit(huart_, data, sizeof(data), 10) == HAL_OK;
}

bool LaserGY56::frame_valid() const
{
  // 检查帧头、数据类型和逐字节累加校验
  if (frame_[0] != 0x5A || frame_[1] != 0x5A || frame_[2] != 0x15 || frame_[3] != 0x03) {
    return false;
  }

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
    checksum = static_cast<uint8_t>(checksum + frame_[i]);
  }
  return checksum == frame_[FRAME_SIZE - 1];
}

void LaserGY56::parse_frame()
{
  // 状态不为0时不更新距离、模式和有效时间
  uint8_t status = static_cast<uint8_t>((frame_[6] >> 4) & 0x0F);
  if (status != 0) {
    return;
  }

  // 实机8字节协议的距离单位为cm, 直接保存帧中的数值
  distance_cm_ = static_cast<uint16_t>((static_cast<uint16_t>(frame_[4]) << 8) | frame_[5]);
  mode_ = static_cast<LaserMode>(frame_[6] & 0x03);
  last_frame_tick_ = HAL_GetTick();
  has_reliable_distance_ = true;
}

}  // namespace sp
