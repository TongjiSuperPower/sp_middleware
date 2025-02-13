#include "manager.hpp"

namespace sp
{
void UI_Manager::set_sender_id(uint8_t robot_id) { sender_id_ = robot_id; }

void UI_Manager::pack(const ui::String * str) {}

void UI_Manager::pack(
  const ui::Element * e1, const ui::Element * e2, const ui::Element * e3, const ui::Element * e4,
  const ui::Element * e5, const ui::Element * e6, const ui::Element * e7)
{
}

}  // namespace sp
