#ifndef SP__ADC_HPP
#define SP__ADC_HPP

#include "adc.h"

namespace sp
{
constexpr uint32_t ADC_12BIT_MAX = 4096;
constexpr uint32_t ADC_VREF = 3.3;
constexpr float VOLTAGE_DIVIDER = (222.0 / 22.0);
constexpr float ADC12_TO_V_COEF = (ADC_VREF * VOLTAGE_DIVIDER / ADC_12BIT_MAX);
class adc
{
public:
  adc(ADC_HandleTypeDef * adc);
  void init();
  //更新电池电压值
  void update();
  //测量得到的电压值
  float voltage;

private:
  ADC_HandleTypeDef * adc_;
  uint32_t buff_;
};
}  // namespace sp

#endif