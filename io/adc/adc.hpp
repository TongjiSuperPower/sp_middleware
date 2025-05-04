#ifndef SP__ADC_HPP
#define SP__ADC_HPP

#include "adc.h"

namespace sp
{
class Adc
{
public:
  Adc(
    ADC_HandleTypeDef * adc, uint32_t channel_sqr, float verf, float voltage_divider,
    float voltage_offset);
  void init();
  //更新得到的电压值
  void update();
  //测量得到的电压值
  float voltage;

private:
  //adc口
  ADC_HandleTypeDef * adc_;
  //ADC转换位数的平方，如12位则2^12(4096)
  uint32_t channel_sqr_;
  //ADC参考电压
  float verf_;
  //电压分压比
  float voltage_divider_;
  //ADC转换值到电压的系数
  float adc12_to_v_coef_;
  //电压偏移值
  float voltage_offset_;
};
}  // namespace sp

#endif