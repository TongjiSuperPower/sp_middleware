#include "vt03.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace sp
{
namespace
{
bool decode_varint(const uint8_t * data, size_t size, size_t & offset, uint64_t & value)
{
  value = 0;

  for (uint8_t shift = 0; shift < 70; shift += 7) {
    if (offset >= size) return false;

    const uint8_t byte = data[offset++];
    value |= static_cast<uint64_t>(byte & 0x7F) << shift;

    if ((byte & 0x80) == 0) return true;
  }

  return false;
}

bool skip_protobuf_field(
  const uint8_t * data, size_t size, size_t & offset, uint8_t wire_type)
{
  uint64_t field_size = 0;

  switch (wire_type) {
    case 0:
      return decode_varint(data, size, offset, field_size);

    case 1:
      if (size - offset < 8) return false;
      offset += 8;
      return true;

    case 2:
      if (!decode_varint(data, size, offset, field_size)) return false;
      if (size - offset < field_size) return false;
      offset += static_cast<size_t>(field_size);
      return true;

    case 5:
      if (size - offset < 4) return false;
      offset += 4;
      return true;

    default:
      return false;
  }
}
}  // namespace

VT03::VT03(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

bool VT03::is_open() const { return has_read_; }

bool VT03::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

void VT03::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_.data(), buff_.size());
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
  }
}

void VT03::update(uint16_t size, uint32_t stamp_ms) { update(buff_.data(), size, stamp_ms); }

void VT03::update(uint8_t * frame_start, uint16_t size, uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  constexpr size_t remote_frame_len = sizeof(VT03RemoteData);

  size_t offset = 0;
  while (offset < size) {
    uint8_t * cursor = frame_start + offset;
    size_t remaining = size - offset;

    if (remaining >= remote_frame_len && cursor[0] == 0xA9 && cursor[1] == 0x53) {
      if (check_crc16(cursor, remote_frame_len)) {
        update_remote(reinterpret_cast<const VT03RemoteData *>(cursor));
        offset += remote_frame_len;
        continue;
      }
    }

    if (cursor[0] != referee::SOF) {
      ++offset;
      continue;
    }

    if (remaining < referee::HEAD_LEN) break;
    if (!check_crc8(cursor, referee::HEAD_LEN)) {
      ++offset;
      continue;
    }

    size_t data_len = (cursor[2] << 8) | cursor[1];
    size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

    // 帧长异常时按字节滑动重同步，尽量保住后续合法帧。
    if (frame_len > remaining) {
      ++offset;
      continue;
    }

    if (!check_crc16(cursor, frame_len)) {
      ++offset;
      continue;
    }

    uint16_t cmd_id = (cursor[6] << 8) | cursor[5];

    switch (cmd_id) {
      // 0x0302 自定义控制器与机器人交互数据
      case referee::cmd_id::CUSTOM_ROBOT_DATA:
        std::copy(
          cursor + referee::DATA_START, cursor + referee::DATA_START + data_len,
          reinterpret_cast<uint8_t *>(&this->custom));
        break;

      // 0x0309 自定义控制器与机器人交互数据
      case referee::cmd_id::ROBOT_CUSTOM_DATA:
        std::copy(
          cursor + referee::DATA_START, cursor + referee::DATA_START + data_len,
          reinterpret_cast<uint8_t *>(&this->robot));
        break;

      // 0x0311 自定义客户端发送给机器人的自定义指令
      case referee::cmd_id::CLIENT_ROBOT_DATA:
        deserialize_custom_client_data(cursor + referee::DATA_START, data_len);
        break;

      default:
        break;
    }

    offset += frame_len;
  }
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

  this->keys.w = (data->keys & 0x0001);
  this->keys.s = (data->keys & 0x0002);
  this->keys.a = (data->keys & 0x0004);
  this->keys.d = (data->keys & 0x0008);
  this->keys.shift = (data->keys & 0x0010);
  this->keys.ctrl = (data->keys & 0x0020);
  this->keys.q = (data->keys & 0x0040);
  this->keys.e = (data->keys & 0x0080);
  this->keys.r = (data->keys & 0x0100);
  this->keys.f = (data->keys & 0x0200);
  this->keys.g = (data->keys & 0x0400);
  this->keys.z = (data->keys & 0x0800);
  this->keys.x = (data->keys & 0x1000);
  this->keys.c = (data->keys & 0x2000);
  this->keys.v = (data->keys & 0x4000);
  this->keys.b = (data->keys & 0x8000);

  this->mode = (data->mode_sw == 2)   ? VT03Mode::S
               : (data->mode_sw == 1) ? VT03Mode::N
                                      : VT03Mode::C;
}

bool VT03::deserialize_custom_client_data(const uint8_t * protobuf_data, size_t protobuf_size)
{
  this->custom_client.size = 0;
  this->custom_client.is_valid = false;
  this->custom_client.data.fill(0);

  size_t offset = 0;

  while (offset < protobuf_size) {
    uint64_t tag = 0;
    if (!decode_varint(protobuf_data, protobuf_size, offset, tag)) return false;

    const uint64_t field_number = tag >> 3;
    const uint8_t wire_type = static_cast<uint8_t>(tag & 0x07);

    if (field_number == 1 && wire_type == 2) {
      uint64_t payload_size = 0;
      if (!decode_varint(protobuf_data, protobuf_size, offset, payload_size)) return false;
      if (payload_size > this->custom_client.data.size()) return false;
      if (protobuf_size - offset < payload_size) return false;

      std::memcpy(
        this->custom_client.data.data(), protobuf_data + offset, static_cast<size_t>(payload_size));
      this->custom_client.size = static_cast<uint8_t>(payload_size);
      this->custom_client.is_valid = true;
      return true;
    }

    if (!skip_protobuf_field(protobuf_data, protobuf_size, offset, wire_type)) return false;
  }

  return false;
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
