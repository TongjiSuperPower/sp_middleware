#ifndef IO__PLOTTER_HPP
#define IO__PLOTTER_HPP

#include "usart.h"

namespace io
{
constexpr size_t MAX_FLOATS = 10;

class Plotter
{
public:
  Plotter(UART_HandleTypeDef * huart);

  template <typename... Floats>
  void plot(Floats... floats)
  {
    static_assert(1 <= sizeof...(floats) && sizeof...(floats) <= MAX_FLOATS);
    fill<0>(floats...);
    send();
  }

private:
#pragma pack(1)
  struct PlotFrame
  {
    uint8_t start[2] = {0xAA, 0xBB};
    uint8_t size;
    float data[MAX_FLOATS];
  };
#pragma pack()

  UART_HandleTypeDef * huart_;
  PlotFrame plot_frame_;

  template <size_t i, typename Float, typename... Floats>
  void fill(Float first, Floats... floats)
  {
    plot_frame_.data[i] = static_cast<float>(first);
    fill<i + 1>(floats...);
  }

  template <size_t i>
  void fill()
  {
    plot_frame_.size = 4 * i;
  }

  void send();
};

}  // namespace io

#endif  // IO__PLOTTER_HPP