#include "adc.hpp"

namespace sp
{
Adc::Adc(
  ADC_HandleTypeDef * adc, uint32_t channel_sqr, float verf, float voltage_divider,
  float voltage_offset)
//构造函数，初始化ADC口，ADC转换位数的平方，如12位则2^12(4096)，ADC参考电压，电压分压比，电压偏移值
: adc_(adc),
  channel_sqr_(channel_sqr),
  verf_(verf),
  voltage_divider_(voltage_divider),
  voltage_offset_(voltage_offset)
{
}

void Adc::init()
{
  //C板 四分频（不超频又不功耗太高）
  adc_->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  HAL_ADC_Init(adc_);
  //计算ADC转换值到电压的系数
  this->adc12_to_v_coef_ = this->verf_ * this->voltage_divider_ / this->channel_sqr_;
}

void Adc::update()
{
  //保险起见如果CUBE忘记配置Continue
  HAL_ADC_Start(adc_);
  this->voltage = HAL_ADC_GetValue(adc_) * this->adc12_to_v_coef_ + this->voltage_offset_;
}
}  // namespace sp
