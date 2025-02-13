#include "element.hpp"

namespace sp::ui
{
static void set_unique(referee::InteractionFigure * data)
{
  static uint32_t id = 0;
  data->figure_name[0] = id & 0xff;
  data->figure_name[1] = (id >> 8) & 0xff;
  data->figure_name[2] = (id >> 16) & 0xff;
  id++;
}

static void set_int32(referee::InteractionFigure * data, int32_t value)
{
  data->details_c = value & 0x3FF;
  data->details_d = (value >> 10) & 0x7FF;
  data->details_e = (value >> 21) & 0x7FF;
}

Element::Element(FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y)
{
  set_unique(&data);
  set_operate_type(OperateType::ADD);
  set_figure_type(type);
  set_layer(layer);
  set_color(color);
  set_width(width);
  set_x(x);
  set_y(y);
}

void Element::set_operate_type(OperateType type) { data.operate_type = static_cast<uint8_t>(type); }
void Element::set_figure_type(FigureType type) { data.figure_type = static_cast<uint8_t>(type); }
void Element::set_layer(Layer layer) { data.layer = static_cast<uint8_t>(layer); }
void Element::set_color(Color color) { data.color = static_cast<uint8_t>(color); }
void Element::set_width(uint16_t width) { data.width = width; }
void Element::set_x(uint16_t x) { data.start_x = x; }
void Element::set_y(uint16_t y) { data.start_y = y; }

Line::Line(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: Element(FigureType::LINE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

void Line::set_x2(uint16_t x2) { data.details_d = x2; }
void Line::set_y2(uint16_t y2) { data.details_e = y2; }

Rectangle::Rectangle(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: Element(FigureType::RECTANGLE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

void Rectangle::set_x2(uint16_t x2) { data.details_d = x2; }
void Rectangle::set_y2(uint16_t y2) { data.details_e = y2; }

Circle::Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius)
: Element(FigureType::CIRCLE, layer, color, width, x, y)
{
  set_radius(radius);
}

void Circle::set_radius(uint16_t radius) { data.details_c = radius; }

Ellipse::Ellipse(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
  uint16_t y_radius)
: Element(FigureType::ELLIPSE, layer, color, width, x, y)
{
  set_x_radius(x_radius);
  set_y_radius(y_radius);
}

void Ellipse::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Ellipse::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

Arc::Arc(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t start_angle,
  uint16_t end_angle, uint16_t x_radius, uint16_t y_radius)
: Element(FigureType::ARC, layer, color, width, x, y)
{
  set_start_angle(start_angle);
  set_end_angle(end_angle);
  set_x_radius(x_radius);
  set_y_radius(y_radius);
}

void Arc::set_start_angle(uint16_t start_angle) { data.details_a = start_angle; }
void Arc::set_end_angle(uint16_t end_angle) { data.details_b = end_angle; }
void Arc::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Arc::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

Text::Text(
  FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y,
  uint16_t font_size)
: Element(type, layer, color, width, x, y)
{
  set_font_size(font_size);
}

void Text::set_font_size(uint16_t font_size) { data.details_a = font_size; }

Float::Float(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, float value)
: Text(FigureType::FLOAT, layer, color, width, x, y, font_size)
{
  set_value(value);
}

void Float::set_value(float value) { set_int32(&data, value * 1e3); }

Integer::Integer(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, int value)
: Text(FigureType::INTEGER, layer, color, width, x, y, font_size)
{
  set_value(value);
}

void Integer::set_value(int value) { set_int32(&data, value); }

String::String(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
  const std::string & value)
: Text(FigureType::STRING, layer, color, width, x, y, font_size)
{
  set_value(value);
}

void String::set_value(const std::string & value)
{
  if (value.size() > sizeof(referee::ExtClientCustomCharacter::data)) return;
  str = value;
  data.details_b = str.size();
}

}  // namespace sp::ui
