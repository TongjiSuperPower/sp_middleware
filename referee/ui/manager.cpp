#include "manager.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
constexpr size_t DATA_HEAD_LEN =
  sizeof(referee::RobotInteractionData) - sizeof(referee::RobotInteractionData::user_data);
constexpr size_t FIG_LEN = sizeof(referee::InteractionFigure);
constexpr size_t STR_LEN = sizeof(referee::ExtClientCustomCharacter::data);

uint16_t get_client_id(uint8_t robot_id)
{
  switch (robot_id) {
    case referee::robot_id::RED_HERO:
      return referee::client_id::RED_HERO_CLIENT;
    case referee::robot_id::RED_ENGINEER:
      return referee::client_id::RED_ENGINEER_CLIENT;
    case referee::robot_id::RED_STANDARD_3:
      return referee::client_id::RED_STANDARD_3_CLIENT;
    case referee::robot_id::RED_STANDARD_4:
      return referee::client_id::RED_STANDARD_4_CLIENT;
    case referee::robot_id::RED_STANDARD_5:
      return referee::client_id::RED_STANDARD_5_CLIENT;
    case referee::robot_id::RED_AERIAL:
      return referee::client_id::RED_AERIAL_CLIENT;
    case referee::robot_id::BLUE_HERO:
      return referee::client_id::BLUE_HERO_CLIENT;
    case referee::robot_id::BLUE_ENGINEER:
      return referee::client_id::BLUE_ENGINEER_CLIENT;
    case referee::robot_id::BLUE_STANDARD_3:
      return referee::client_id::BLUE_STANDARD_3_CLIENT;
    case referee::robot_id::BLUE_STANDARD_4:
      return referee::client_id::BLUE_STANDARD_4_CLIENT;
    case referee::robot_id::BLUE_STANDARD_5:
      return referee::client_id::BLUE_STANDARD_5_CLIENT;
    case referee::robot_id::BLUE_AERIAL:
      return referee::client_id::BLUE_AERIAL_CLIENT;
    default:
      return referee::client_id::REFEREE_SERVER;
  }
}

UI_Manager::UI_Manager()
{
  frame_.head.sof = referee::SOF;
  frame_.head.seq = 0;
  frame_.cmd_id = referee::cmd_id::ROBOT_INTERACTION_DATA;
}

size_t UI_Manager::size() const
{
  return referee::HEAD_LEN + referee::CMD_ID_LEN + frame_.head.data_len + referee::TAIL_LEN;
}

const uint8_t * UI_Manager::data() const { return reinterpret_cast<const uint8_t *>(&frame_); }

void UI_Manager::set_sender_id(uint8_t robot_id)
{
  frame_.data.sender_id = robot_id;
  frame_.data.receiver_id = get_client_id(robot_id);
}

void UI_Manager::pack(const ui::String * str)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::ExtClientCustomCharacter);

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::EXT_CLIENT_CUSTOM_CHARACTER;

  copy(str, 0);

  std::fill(frame_.data.user_data + FIG_LEN, frame_.data.user_data + FIG_LEN + STR_LEN, '\0');

  std::copy(
    str->str.data(), str->str.data() + std::min(str->str.size(), STR_LEN),
    frame_.data.user_data + FIG_LEN);

  apply_crc16();
}

void UI_Manager::pack(const ui::Element * e1)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN;

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE;

  copy(e1, 0);

  apply_crc16();
}

void UI_Manager::pack(const ui::Element * e1, const ui::Element * e2)
{
  frame_.head.data_len = DATA_HEAD_LEN + 2 * FIG_LEN;

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_2;

  copy(e1, 0);
  copy(e2, 1);

  apply_crc16();
}

void UI_Manager::pack(
  const ui::Element * e1, const ui::Element * e2, const ui::Element * e3, const ui::Element * e4,
  const ui::Element * e5)
{
  frame_.head.data_len = DATA_HEAD_LEN + 5 * FIG_LEN;

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_5;

  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);

  apply_crc16();
}

void UI_Manager::pack(
  const ui::Element * e1, const ui::Element * e2, const ui::Element * e3, const ui::Element * e4,
  const ui::Element * e5, const ui::Element * e6, const ui::Element * e7)
{
  frame_.head.data_len = DATA_HEAD_LEN + 7 * FIG_LEN;

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_7;

  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);
  copy(e6, 5);
  copy(e7, 6);

  apply_crc16();
}

void UI_Manager::copy(const ui::Element * e, size_t i)
{
  if (e == nullptr) {
    std::copy(
      reinterpret_cast<const uint8_t *>(&empty_),
      reinterpret_cast<const uint8_t *>(&empty_) + FIG_LEN, frame_.data.user_data + FIG_LEN * i);
  }
  else {
    std::copy(
      reinterpret_cast<const uint8_t *>(&e->data),
      reinterpret_cast<const uint8_t *>(&e->data) + FIG_LEN, frame_.data.user_data + FIG_LEN * i);
  }
}

void UI_Manager::apply_crc8() { frame_.head.crc8 = get_crc8(data(), 4); }

void UI_Manager::apply_crc16()
{
  auto crc16 = get_crc16(data(), size() - referee::TAIL_LEN);
  frame_.data.user_data[frame_.head.data_len - DATA_HEAD_LEN] = crc16 & 0xff;
  frame_.data.user_data[frame_.head.data_len - DATA_HEAD_LEN + 1] = crc16 >> 8;
}

}  // namespace sp
