#ifndef SP__UI_MANAGER_HPP
#define SP__UI_MANAGER_HPP

#include "element.hpp"
#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp
{
constexpr size_t UI_DATA_MAX_SIZE =
  referee::HEAD_LEN + referee::TAIL_LEN + sizeof(referee::RobotInteractionData);

class UI_Manager
{
public:
  size_t size = UI_DATA_MAX_SIZE;
  uint8_t data[UI_DATA_MAX_SIZE];

  void set_sender_id(uint8_t robot_id);
  void pack(const ui::String * str);
  void pack(
    const ui::Element * e1, const ui::Element * e2 = nullptr, const ui::Element * e3 = nullptr,
    const ui::Element * e4 = nullptr, const ui::Element * e5 = nullptr,
    const ui::Element * e6 = nullptr, const ui::Element * e7 = nullptr);

private:
  uint16_t sender_id_;
};

}  // namespace sp

#endif  // SP__UI_MANAGER_HPP