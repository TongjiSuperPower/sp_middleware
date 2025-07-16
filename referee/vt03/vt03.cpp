#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
VT03::VT03(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

void VT03::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_.data(), buff_.size());
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
  }
}

void VT03::update(uint16_t size) { update(buff_.data(), size); }

void VT03::update(uint8_t * frame_start, uint16_t size)
{
  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    size_t frame_len = sizeof(VT03RemoteData);

    if (size < frame_len) return;
    if (!check_crc16(frame_start, frame_len)) return;

    update_remote(reinterpret_cast<VT03RemoteData *>(frame_start));

    // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
    update(frame_start + frame_len, size - frame_len);
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
      break;

    // 0x0304 键鼠遥控数据
    case referee::cmd_id::REMOTE_CONTROL:
      update_mouse_and_keys(
        reinterpret_cast<referee::RemoteControl *>(frame_start + referee::DATA_START));
      break;

    // 0x0309 自定义控制器与机器人交互数据
    case referee::cmd_id::ROBOT_CUSTOM_DATA:
      std::copy(
        frame_start + referee::DATA_START, frame_start + referee::DATA_START + data_len,
        reinterpret_cast<uint8_t *>(&this->robot));
      break;

    default:
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  update(frame_start + frame_len, size - frame_len);
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

  this->mode = (data->mode_sw == 2)   ? VT03Mode::S
               : (data->mode_sw == 1) ? VT03Mode::N
                                      : VT03Mode::C;
}

void VT03::update_mouse_and_keys(const referee::RemoteControl * data)
{
  this->mouse.vx = data->mouse_x / 32768.0f;
  this->mouse.vy = data->mouse_y / 32768.0f;
  this->mouse.vs = data->mouse_z / 32768.0f;
  this->mouse.left = data->left_button_down;
  this->mouse.right = data->right_button_down;

  this->keys.w = (data->keyboard_value & 0x0001);
  this->keys.s = (data->keyboard_value & 0x0002);
  this->keys.a = (data->keyboard_value & 0x0004);
  this->keys.d = (data->keyboard_value & 0x0008);
  this->keys.shift = (data->keyboard_value & 0x0010);
  this->keys.ctrl = (data->keyboard_value & 0x0020);
  this->keys.q = (data->keyboard_value & 0x0040);
  this->keys.e = (data->keyboard_value & 0x0080);
  this->keys.r = (data->keyboard_value & 0x0100);
  this->keys.f = (data->keyboard_value & 0x0200);
  this->keys.g = (data->keyboard_value & 0x0400);
  this->keys.z = (data->keyboard_value & 0x0800);
  this->keys.x = (data->keyboard_value & 0x1000);
  this->keys.c = (data->keyboard_value & 0x2000);
  this->keys.v = (data->keyboard_value & 0x4000);
  this->keys.b = (data->keyboard_value & 0x8000);
}

}  // namespace sp
