#ifndef SP__PLOTTER_HPP
#define SP__PLOTTER_HPP

#include "usart.h"

namespace sp
{
constexpr size_t PLOTTER_FLOAT_NUM = 6;

#pragma pack(1)
struct PlotFrame
{
  uint8_t start[2] = {0xAA, 0xBB};
  uint8_t size;
  float data[PLOTTER_FLOAT_NUM];
};
#pragma pack()

class Plotter
{
public:
  Plotter(UART_HandleTypeDef * huart, bool use_dma = true);

  void plot(float value1);
  void plot(float value1, float value2);
  void plot(float value1, float value2, float value3);
  void plot(float value1, float value2, float value3, float value4);
  void plot(float value1, float value2, float value3, float value4, float value5);
  void plot(float value1, float value2, float value3, float value4, float value5, float value6);

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  HAL_StatusTypeDef hal_status_;
  PlotFrame plot_frame_;

  void send();
};

}  // namespace sp

#endif  // SP__PLOTTER_HPP