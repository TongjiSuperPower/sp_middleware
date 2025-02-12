#include "ui.hpp"

namespace sp
{
// -------------------- 以下为图形类 --------------------

static void set_unique(sp::referee::InteractionFigure & data)
{
  static uint32_t id = 0;
  data.figure_name[0] = id & 0xff;
  data.figure_name[1] = (id >> 8) & 0xff;
  data.figure_name[2] = (id >> 16) & 0xff;
  id++;
}

Line::Line()
{
  set_unique(this->data);
  this->data.operate_type = 1;
  this->data.figure_type = 0;
}

Line & Line::set_x2(uint16_t x2)
{
  this->data.details_d = x2;
  return *this;
}

Line & Line::set_y2(uint16_t y2)
{
  this->data.details_e = y2;
  return *this;
}

Rect::Rect()
{
  set_unique(this->data);
  this->data.operate_type = 1;
  this->data.figure_type = 1;
}

Rect & Rect::set_x2(uint16_t x2)
{
  this->data.details_d = x2;
  return *this;
}

Rect & Rect::set_y2(uint16_t y2)
{
  this->data.details_e = y2;
  return *this;
}

Circle::Circle()
{
  set_unique(this->data);
  this->data.operate_type = 1;
  this->data.figure_type = 2;
}

Circle & Circle::set_r(uint16_t r)
{
  this->data.details_c = r;
  return *this;
}

Ellipse::Ellipse()
{
  set_unique(this->data);
  this->data.operate_type = 1;
  this->data.figure_type = 3;
}

Ellipse & Ellipse::set_rx(uint16_t rx)
{
  this->data.details_d = rx;
  return *this;
}

Ellipse & Ellipse::set_ry(uint16_t ry)
{
  this->data.details_e = ry;
  return *this;
}

Arc::Arc()
{
  set_unique(this->data);
  this->data.operate_type = 1;
  this->data.figure_type = 4;
}

Arc & Arc::set_start_angle(uint16_t start_angle)
{
  this->data.details_a = start_angle;
  return *this;
}

Arc & Arc::set_end_angle(uint16_t end_angle)
{
  this->data.details_b = end_angle;
  return *this;
}

Arc & Arc::set_rx(uint16_t rx)
{
  this->data.details_d = rx;
  return *this;
}

Arc & Arc::set_ry(uint16_t ry)
{
  this->data.details_e = ry;
  return *this;
}

}  // namespace sp