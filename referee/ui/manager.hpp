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

  void delete_all();
  void delete_layer(ui::Layer layer);

  void pack(const ui::String * str);
  void pack(const ui::ComposableElement * e1);
  void pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2);
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4 = nullptr,
    const ui::ComposableElement * e5 = nullptr);
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4,
    const ui::ComposableElement * e5, const ui::ComposableElement * e6,
    const ui::ComposableElement * e7 = nullptr);

private:
  struct __attribute__((packed))
  {
    referee::FrameHeader head;
    uint16_t cmd_id;
    referee::RobotInteractionData data;
    // 绘制UI所需的user_data最大长度为105, 而其定义长度为112, 有剩余, 因此帧尾的crc16在data内部
  } frame_;

  referee::InteractionFigure empty_;

  void copy(const ui::Element * e, size_t i);
  void apply_crc();
};

}  // namespace sp

#endif  // SP__UI_MANAGER_HPP