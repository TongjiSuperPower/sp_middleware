#ifndef IO__WS2812_HPP
#define IO__WS2812_HPP

#include "spi.h"

namespace sp
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

}  // namespace sp

#endif  // IO__WS2812_HPP