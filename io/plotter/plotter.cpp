#include "plotter.hpp"

namespace sp
{
Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK)
{
}

void Plotter::plot(float value1)
{
  static_assert(PLOTTER_FLOAT_NUM >= 1);
  plot_frame_.size = 4 * 1;
  plot_frame_.data[0] = value1;
  send();
}

void Plotter::plot(float value1, float value2)
{
  static_assert(PLOTTER_FLOAT_NUM >= 2);
  plot_frame_.size = 4 * 2;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  send();
}

void Plotter::plot(float value1, float value2, float value3)
{
  static_assert(PLOTTER_FLOAT_NUM >= 3);
  plot_frame_.size = 4 * 3;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4)
{
  static_assert(PLOTTER_FLOAT_NUM >= 4);
  plot_frame_.size = 4 * 4;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4, float value5)
{
  static_assert(PLOTTER_FLOAT_NUM >= 5);
  plot_frame_.size = 4 * 5;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6)
{
  static_assert(PLOTTER_FLOAT_NUM >= 6);
  plot_frame_.size = 4 * 6;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7)
{
  static_assert(PLOTTER_FLOAT_NUM >= 7);
  plot_frame_.size = 4 * 7;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8)
{
  static_assert(PLOTTER_FLOAT_NUM >= 8);
  plot_frame_.size = 4 * 8;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9)
{
  static_assert(PLOTTER_FLOAT_NUM >= 9);
  plot_frame_.size = 4 * 9;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10)
{
  static_assert(PLOTTER_FLOAT_NUM >= 10);
  plot_frame_.size = 4 * 10;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  send();
}

void Plotter::send()
{
  if (use_dma_) {
    hal_status_ = HAL_UART_Transmit_DMA(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size);
  }
  else {
    hal_status_ = HAL_UART_Transmit(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size, 0xff);
  }
}

}  // namespace sp
