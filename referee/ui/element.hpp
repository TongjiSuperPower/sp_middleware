#ifndef SP__UI_ELEMENT_HPP
#define SP__UI_ELEMENT_HPP

#include <string>

#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp::ui
{
enum class OperateType : uint8_t
{
  EMPTY = 0,
  ADD = 1,
  MODIFY = 2,
  DELETE = 3
};

enum class FigureType : uint8_t
{
  LINE = 0,
  RECTANGLE = 1,
  CIRCLE = 2,
  ELLIPSE = 3,
  ARC = 4,
  FLOAT = 5,
  INTEGER = 6,
  STRING = 7
};

enum class Layer : uint8_t
{
  LAYER_0 = 0,
  LAYER_1 = 1,
  LAYER_2 = 2,
  LAYER_3 = 3,
  LAYER_4 = 4,
  LAYER_5 = 5,
  LAYER_6 = 6,
  LAYER_7 = 7,
  LAYER_8 = 8,
  LAYER_9 = 9
};

enum class Color : uint8_t
{
  RED_BLUE = 0,
  YELLOW = 1,
  GREEN = 2,
  ORANGE = 3,
  PURPLE = 4,
  PINK = 5,
  CYAN = 6,
  BLACK = 7,
  WHITE = 8
};

struct Element
{
  referee::InteractionFigure data;
  Element(FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y);
  void set_operate_type(OperateType type);
  void set_figure_type(FigureType type);
  void set_layer(Layer layer);
  void set_color(Color color);
  void set_width(uint16_t width);
  void set_x(uint16_t x);
  void set_y(uint16_t y);
};

struct ComposableElement : public Element
{
  using Element::Element;
};

struct Line : public ComposableElement
{
  Line(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);
  void set_y2(uint16_t y2);
};

struct Rectangle : public ComposableElement
{
  Rectangle(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);
  void set_y2(uint16_t y2);
};

struct Circle : public ComposableElement
{
  Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius);
  void set_radius(uint16_t radius);
};

struct Ellipse : public ComposableElement
{
  Ellipse(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
    uint16_t y_radius);
  void set_x_radius(uint16_t x_radius);
  void set_y_radius(uint16_t y_radius);
};

struct Arc : public ComposableElement
{
  Arc(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t start_angle,
    uint16_t end_angle, uint16_t x_radius, uint16_t y_radius);
  void set_start_angle(uint16_t start_angle);
  void set_end_angle(uint16_t end_angle);
  void set_x_radius(uint16_t x_radius);
  void set_y_radius(uint16_t y_radius);
};

struct Float : public ComposableElement
{
  Float(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    float value);
  void set_font_size(uint16_t font_size);
  void set_value(float value);
};

struct Integer : public ComposableElement
{
  Integer(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    int value);
  void set_font_size(uint16_t font_size);
  void set_value(int value);
};

struct String : public Element
{
  std::string str;
  String(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    const std::string & value);
  void set_font_size(uint16_t font_size);
  void set_value(const std::string & value);
};

}  // namespace sp::ui

#endif  // SP__UI_ELEMENT_HPP