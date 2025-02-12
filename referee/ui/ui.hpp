#ifndef SP__UI_HPP
#define SP__UI_HPP

#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp
{
constexpr size_t UI_DATA_MAX_SIZE =
  referee::HEAD_LEN + referee::TAIL_LEN + sizeof(referee::RobotInteractionData);

class UI
{
public:
  size_t size = UI_DATA_MAX_SIZE;
  uint8_t data[UI_DATA_MAX_SIZE];

  void set_sender_id(uint8_t robot_id);
  void pack(const referee::InteractionFigure * f1, const referee::InteractionFigure * f2);

private:
  uint16_t sender_id_;
};

// -------------------- 以下为图形类 --------------------

// 图形基类, 使用模板实现链式调用(Method Chaining)
template <typename T>
struct Figure
{
  sp::referee::InteractionFigure data;  // 只读!

  T & set_operate_type(uint8_t operate_type)
  {
    data.operate_type = operate_type;
    return static_cast<T &>(*this);
  }

  // 图层0～9
  T & set_layer(uint8_t layer)
  {
    data.layer = layer;
    return static_cast<T &>(*this);
  }

  T & set_color(uint8_t color)
  {
    data.color = color;
    return static_cast<T &>(*this);
  }

  // 线宽, 建议字体大小与线宽比例为10:1
  T & set_width(uint16_t width)
  {
    data.width = width;
    return static_cast<T &>(*this);
  }

  // 起点/圆心x坐标
  T & set_x(uint16_t x)
  {
    data.start_x = x;
    return static_cast<T &>(*this);
  }

  // 起点/圆心y坐标
  T & set_y(uint16_t y)
  {
    data.start_y = y;
    return static_cast<T &>(*this);
  }
};

struct Line : public Figure<Line>
{
  // 自动生成唯一图形名
  Line();

  // 终点x坐标
  Line & set_x2(uint16_t x2);

  // 终点y坐标
  Line & set_y2(uint16_t y2);
};

struct Rect : public Figure<Rect>
{
  // 自动生成唯一图形名
  Rect();

  // 对角顶点x坐标
  Rect & set_x2(uint16_t x2);

  // 对角顶点y坐标
  Rect & set_y2(uint16_t y2);
};

struct Circle : public Figure<Circle>
{
  // 自动生成唯一图形名
  Circle();

  // 半径
  Circle & set_r(uint16_t r);
};

struct Ellipse : public Figure<Ellipse>
{
  // 自动生成唯一图形名
  Ellipse();

  // x半轴长度
  Ellipse & set_rx(uint16_t rx);

  // y半轴长度
  Ellipse & set_ry(uint16_t ry);
};

struct Arc : public Figure<Arc>
{
  // 自动生成唯一图形名
  Arc();

  // 起始角度
  Arc & set_start_angle(uint16_t start_angle);

  // 终止角度
  Arc & set_end_angle(uint16_t end_angle);

  // x半轴长度
  Arc & set_rx(uint16_t rx);

  // y半轴长度
  Arc & set_ry(uint16_t ry);
};

}  // namespace sp

#endif  // SP__UI_HPP