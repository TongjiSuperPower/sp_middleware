#include "plotter.hpp"

namespace io
{
Plotter::Plotter(UART_HandleTypeDef * huart) : huart_(huart) {}

void Plotter::send()
{
  // dismiss return
  HAL_UART_Transmit(
    huart_, (uint8_t *)&plot_frame_,
    sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size, 0xFFFF);
}

}  // namespace io
