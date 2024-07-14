#ifndef IO__WS2812_HPP
#define IO__WS2812_HPP

#include <cstdint>

#include "spi.h"

namespace io
{
class WS2812
{
public:
  WS2812(SPI_HandleTypeDef * hspi);
  void set(uint8_t r, uint8_t g, uint8_t b);

private:
  SPI_HandleTypeDef * hspi_;
  uint8_t buff_[24];
};

}  // namespace io

#endif  // IO__WS2812_HPP