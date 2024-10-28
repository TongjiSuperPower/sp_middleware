#include "ws2812.hpp"

namespace sp
{
constexpr uint8_t CODE0 = 0xC0;
constexpr uint8_t CODE1 = 0xF0;

WS2812::WS2812(SPI_HandleTypeDef * hspi) : hspi_(hspi) {}

void WS2812::set(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < 8; i++) {
    buff_[07 - i] = (((g >> i) & 0x01) ? CODE1 : CODE0) >> 1;
    buff_[15 - i] = (((r >> i) & 0x01) ? CODE1 : CODE0) >> 1;
    buff_[23 - i] = (((b >> i) & 0x01) ? CODE1 : CODE0) >> 1;
  }

  HAL_SPI_Transmit(hspi_, buff_, 24, 0xFFFF);  // dismiss return
}

}  // namespace sp
