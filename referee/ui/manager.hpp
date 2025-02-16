#ifndef SP__UI_MANAGER_HPP
#define SP__UI_MANAGER_HPP

#include "element.hpp"
#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp
{
class UI_Manager
{
public:
  UI_Manager();

  size_t size() const;
  const uint8_t * data() const;

  void set_sender_id(uint8_t robot_id);
  void pack(const ui::String * str);
  void pack(
    const ui::Element * e1, const ui::Element * e2 = nullptr, const ui::Element * e3 = nullptr,
    const ui::Element * e4 = nullptr, const ui::Element * e5 = nullptr,
    const ui::Element * e6 = nullptr, const ui::Element * e7 = nullptr);

private:
  struct __attribute__((packed))
  {
    referee::FrameHeader head;
    uint16_t cmd_id;
    referee::RobotInteractionData data;  // crc16在内部
  } frame_;

  void apply_crc8();
  void apply_crc16();
};

}  // namespace sp

#endif  // SP__UI_MANAGER_HPP