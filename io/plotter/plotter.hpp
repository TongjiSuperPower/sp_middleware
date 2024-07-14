#ifndef IO__PLOTTER_HPP
#define IO__PLOTTER_HPP

#include <cstdint>

#include "usart.h"

namespace io
{
constexpr size_t MAX_FLOATS = 6;

class Plotter
{
public:
  Plotter(UART_HandleTypeDef * huart);

  void plot(float value1);
  void plot(float value1, float value2);
  void plot(float value1, float value2, float value3);
  void plot(float value1, float value2, float value3, float value4);
  void plot(float value1, float value2, float value3, float value4, float value5);
  void plot(float value1, float value2, float value3, float value4, float value5, float value6);

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

  void send();
};

}  // namespace io

#endif  // IO__PLOTTER_HPP