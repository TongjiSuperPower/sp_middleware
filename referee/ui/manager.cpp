#include "manager.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
constexpr size_t FIG_SIZE = sizeof(referee::InteractionFigure);

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
  constexpr size_t STR_SIZE = sizeof(referee::ExtClientCustomCharacter::data);

  frame_.head.data_len = sizeof(referee::RobotInteractionData) -
                         sizeof(referee::RobotInteractionData::user_data) +
                         sizeof(referee::ExtClientCustomCharacter);

  apply_crc8();

  frame_.data.data_cmd_id = referee::data_cmd_id::EXT_CLIENT_CUSTOM_CHARACTER;

  std::copy(
    reinterpret_cast<const uint8_t *>(&str->data),
    reinterpret_cast<const uint8_t *>(&str->data) + FIG_SIZE, frame_.data.user_data);

  std::fill(frame_.data.user_data + FIG_SIZE, frame_.data.user_data + FIG_SIZE + STR_SIZE, '\0');

  std::copy(
    str->str.data(), str->str.data() + std::min(str->str.size(), STR_SIZE),
    frame_.data.user_data + FIG_SIZE);

  apply_crc16();
}

void UI_Manager::pack(
  const ui::Element * e1, const ui::Element * e2, const ui::Element * e3, const ui::Element * e4,
  const ui::Element * e5, const ui::Element * e6, const ui::Element * e7)
{
}

void UI_Manager::apply_crc8() { frame_.head.crc8 = get_crc8(data(), 4); }

void UI_Manager::apply_crc16()
{
  auto crc16 = get_crc16(data(), size() - referee::TAIL_LEN);
  frame_.data.user_data[frame_.head.data_len] = crc16 & 0xff;
  frame_.data.user_data[frame_.head.data_len + 1] = crc16 >> 8;
}

}  // namespace sp
