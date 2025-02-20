#include "manager.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
constexpr size_t DATA_HEAD_LEN =
  sizeof(referee::RobotInteractionData) - sizeof(referee::RobotInteractionData::user_data);
constexpr size_t FIG_LEN = sizeof(referee::InteractionFigure);
constexpr size_t MAX_STR_LEN = sizeof(referee::ExtClientCustomCharacter::data);

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
  empty_.operate_type = static_cast<uint8_t>(ui::OperateType::EMPTY);
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

void UI_Manager::delete_all()
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 2;
  apply_crc();
}

void UI_Manager::delete_layer(ui::Layer layer)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 1;
  frame_.data.user_data[1] = static_cast<uint8_t>(layer);
  apply_crc();
}

void UI_Manager::pack(const ui::String * str)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::ExtClientCustomCharacter);
  frame_.data.data_cmd_id = referee::data_cmd_id::EXT_CLIENT_CUSTOM_CHARACTER;

  auto data = str->str.data();
  auto str_len = std::min(str->str.size(), MAX_STR_LEN);
  std::fill(frame_.data.user_data + FIG_LEN, frame_.data.user_data + FIG_LEN + MAX_STR_LEN, 0);
  std::copy(data, data + str_len, frame_.data.user_data + FIG_LEN);

  copy(str, 0);
  apply_crc();
}

void UI_Manager::pack(const ui::ComposableElement * e1)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE;
  copy(e1, 0);
  apply_crc();
}

void UI_Manager::pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 2;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_2;
  copy(e1, 0);
  copy(e2, 1);
  apply_crc();
}

void UI_Manager::pack(
  const ui::ComposableElement * e1, const ui::ComposableElement * e2,
  const ui::ComposableElement * e3, const ui::ComposableElement * e4,
  const ui::ComposableElement * e5)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 5;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_5;
  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);
  apply_crc();
}

void UI_Manager::pack(
  const ui::ComposableElement * e1, const ui::ComposableElement * e2,
  const ui::ComposableElement * e3, const ui::ComposableElement * e4,
  const ui::ComposableElement * e5, const ui::ComposableElement * e6,
  const ui::ComposableElement * e7)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 7;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_7;
  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);
  copy(e6, 5);
  copy(e7, 6);
  apply_crc();
}

void UI_Manager::copy(const ui::Element * e, size_t i)
{
  auto fig = reinterpret_cast<const uint8_t *>((e == nullptr) ? &empty_ : &e->data);
  std::copy(fig, fig + FIG_LEN, frame_.data.user_data + FIG_LEN * i);
}

void UI_Manager::apply_crc()
{
  // 先计算crc8
  frame_.head.crc8 = get_crc8(data(), 4);

  // 再计算crc16
  auto tail = reinterpret_cast<uint8_t *>(&frame_.data) + frame_.head.data_len;
  auto crc16 = reinterpret_cast<uint16_t *>(tail);
  *crc16 = get_crc16(data(), size() - referee::TAIL_LEN);
}

}  // namespace sp
